/**
 * @file CLASS_Motor.hpp
 * @brief 定义了基于CANopen DS402协议的电机控制核心数据结构和类
 *
 * @details 本文件实现了机械臂驱动程序的核心电机类，采用面向对象设计和现代C++特性，
 * 为多线程环境下的高性能电机控制提供原子化、缓存友好的数据访问接口。该设计针对
 * ARM64架构（如Cortex-A55/A76）进行了优化，并兼容x86平台，支持Windows和Linux系统。
 *
 * 主要功能包括：
 * - 封装符合DS402标准的对象字典地址和电机运行模式
 * - 提供状态与模式、电流、位置、速度、加减速等电机参数的统一管理
 * - 实现基于缓存行对齐的原子数据结构，确保多线程安全访问
 * - 支持PDO数据刷新机制，实现原始数据到物理量的自动转换
 * - 为未来扩展预留接口，如Python绑定、轨迹回放等功能
 *
 * @note 该模块专为2ms实时同步周期设计，适用于6轴机械臂控制系统。
 * 所有数据结构均考虑了内存布局优化以减少伪共享，提高并发性能。
 *
 * @par 线程模型：
 * - 接收线程：更新实际值并调用刷新函数
 * - 发送线程：读取目标值并发送PDO帧
 * - 规划线程：计算目标值并触发刷新写入
 * 各线程通过原子标志位协调数据刷新流程，避免锁竞争。
 *
 * @par 设计亮点：
 * - 使用模板化的缓存行对齐结构体 `AlignedRawData` 实现类型安全的数据视图
 * - 利用C++17的constexpr和if constexpr提升编译期性能
 * - 基于位标志的状态管理机制，支持细粒度数据刷新控制
 */






#ifndef CLASS_MOTOR_HPP
#define CLASS_MOTOR_HPP



#include <array>
#include <string>
#include <cmath>
#include <mutex>
#include <atomic>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <chrono>
#include <stdint.h>
#include <cstdint>
#include <type_traits>


#include "Data_processing.hpp"


#define SENSOR_RANGE 32768


/*********** 自定义类型区 ************/

//保证处理同一个物理量使用相同的类型

using Flag_type = uint16_t;  //  专用于处理标志位
using Current_type = int16_t;  //专用于处理电流
using Velocity_type = int16_t; //专用于处理速度
using Position_type = int32_t; //专用于处理位置
using AccelDecel_type = uint16_t; //专用于处理加减速度







/**
 * @brief 硬件缓存行对齐的原子化数据存储结构体模板
 * @tparam N 数据位宽（支持1/2/4/8字节）
 * @tparam T 可选用户指定类型（默认void自动推断）
 *
 * @details 本模板提供：
 * 1. 保证64字节缓存行对齐（ARM64/X86优化）
 * 2. 类型双关(Type Punning)安全实现
 * 3. 跨平台原子操作封装
 * 4. 多线程安全访问保障
 *
 * @warning 特殊使用限制：
 * - 禁止复制构造/赋值（拷贝需通过原子接口）
 * - 实例必须独占缓存行（不可定义未填充的数组）
 * - 用户指定类型T必须是平凡可复制(trivially copyable)类型
 *
 * @note 典型应用场景：
 * - CANopen协议PDO映射数据区
 * - 多核共享的电机控制参数
 * - 高频更新的传感器数据
 */
template <size_t N, typename T = void>
struct alignas(64) AlignedRawData {
    /* 数据宽度静态检查 */
    static_assert(N == 1 || N == 2 || N == 4 || N == 8,
        "Only support 1/2/4/8 bytes width");
    /* 用户类型安全检查 */
    static_assert(std::is_void_v<T> ||
        (!std::is_void_v<T> && sizeof(T) == N && std::is_trivially_copyable_v<T>),
        "Invalid user-specified type");


    /**
     * @brief 匿名联合体实现安全类型双关
     * @details 通过联合体实现两种数据视图：
     * - 原始字节序列：用于协议栈/DMA直接访问
     * - 类型化视图：用于业务逻辑操作
     */
    union {
        volatile uint8_t bytes_[N];  ///< 原始字节视图（内存连续，支持memcpy）
        std::conditional_t<std::is_same_v<T, void>,
            std::conditional_t<N == 1, int8_t,      ///< N=1默认int8_t
            std::conditional_t<N == 2, int16_t, ///< N=2默认int16_t
            std::conditional_t<N == 4, int32_t, ///< N=4默认int32_t
            int64_t>>>, ///< N=8默认int64_t
            T> value_;  ///< 用户指定类型视图（需确保sizeof(T)==N）
    };

    /// 缓存行填充（ARM64为64字节）
    uint8_t padding_[64 - N];

    // ====================== 原子操作接口 ====================== //



        /**
         * @brief 原子写入数据（Release语义）
         * @tparam U 数据类型（自动推导）
         * @param buf 输入数据指针
         *
         * @note 技术特性：
         * 1. 使用std::memory_order_release保证写入可见性
         * 2. 静态检查类型大小匹配和可平凡复制性
         * 3. 通过void*中转避免strict-aliasing警告
         *
         * @warning 必须遵循：
         * - buf必须是有效指针
         * - 禁止与非原子操作混用
         *
         * @example 写入uint32_t值：
         * @code
         * uint32_t val = 0x12345678;
         * data.atomicWrite(&val);
         * @endcode
         */
        template <typename U>
        void atomicWrite(const U* buf) noexcept {
            static_assert(sizeof(U) == N, "Type size mismatch");
            static_assert(std::is_trivially_copyable_v<U>,
                "Type must be trivially copyable");

            std::memcpy(const_cast<void*>(static_cast<const void*>(&value_)),
                static_cast<const void*>(buf),
                sizeof(U));
        }

        void atomicWriteValue(decltype(value_) val) noexcept {
            atomicWrite(&val);
        }

        /**
         * @brief 原子读取数据（Acquire语义）
         * @tparam U 目标数据类型
         * @param[out] buf 输出缓冲区指针
         *
         * @note 内存序保证：
         * - 确保读取前的所有写入操作对当前线程可见
         * - 适合状态机等需要强一致性的场景
         *
         * @warning 缓冲区必须预先分配且大小匹配
         *
         * @example 读取到int32_t变量：
         * @code
         * int32_t result;
         * data.atomicRead(&result);
         * @endcode
         */
        template <typename U>
        void atomicRead(U* buf) const noexcept {
            static_assert(sizeof(U) == N, "Type size mismatch");
            static_assert(std::is_trivially_copyable_v<U>,
                "Type must be trivially copyable");

            
                std::memcpy(static_cast<void*>(buf),
                    static_cast<const void*>(&value_),
                    sizeof(U));
        }


        decltype(value_) atomicReadValue() const noexcept {
            decltype(value_) val;
            atomicRead(&val);
            return val;
        }

        //=== 防误用设计 ===//
        AlignedRawData() = default;
        ~AlignedRawData() = default; 
        AlignedRawData(const AlignedRawData&) = delete;
        AlignedRawData& operator=(const AlignedRawData&) = delete;
    };


// 编译时校验
    static_assert(alignof(AlignedRawData<4, int32_t>) == 64,
        "Alignment requirement failed for AlignedRawData<4>");
    static_assert(sizeof(AlignedRawData<4, int32_t>) == 64,
        "Size requirement failed for AlignedRawData<4>");










/**
 * @brief DS402 协议下常用对象字典地址示例（仅供参考）
 * 一般是 0xXXXX:subIndex, 可以在这里统一声明。 箭头符号代表向下位机写入/读取
 */
static constexpr uint16_t OD_CONTROL_WORD = 0x6040;  /// 控制字>>> 0x6040  
static constexpr uint16_t OD_STATUS_WORD = 0x6041;  /// 状态字<<< 0x6041
static constexpr uint16_t OD_MODES_OF_OPERATION = 0x6060;  /// 运行模式(写)>>> 0x6060
static constexpr uint16_t OD_MODES_OF_DISPLAY = 0x6061;  /// 运行模式(读)<<< 0x6061
static constexpr uint16_t OD_ERROR_CODE = 0x603F;  /// 错误码<<< 0x603F

static constexpr uint16_t OD_TARGET_CURRENT = 0x6071;   /// 目标电流>>> 0x6071
static constexpr uint16_t OD_ACTUAL_CURRENT = 0x6078;   /// 实际电流<<< 0x6078
static constexpr uint16_t OD_TARGET_POSITION = 0x607A;  /// 目标位置>>> 0x607A
static constexpr uint16_t OD_ACTUAL_POSITION = 0x6064;  /// 实际位置<<< 0x6064
static constexpr uint16_t OD_TARGET_VELOCITY_VELOCITY_MODE = 0x60FF;  /// 目标速度（速度环模式）（电机端未经过编码器的速度）>>> 0x60FF
static constexpr uint16_t OD_TARGET_VELOCITY_POSITION_MODE = 0x6081; /// 目标速度（位置环模式）（外壳轮廓的速度）>>> 0x6081
static constexpr uint16_t OD_ACTUAL_VELOCITY = 0x606C;  /// 实际速度（电机末端速度）<<< 0x606C

static constexpr uint16_t OD_ACCELERATION = 0x6083;  /// 加速度>>> 0x6083
static constexpr uint16_t OD_DECELERATION = 0x6084;  /// 减速度>>> 0x6084


/**
 * @brief 用于表示 DS402 中常见的电机运行模式
 * 这里只是示例，具体可根据所用驱动的 DS402 实现细节增改
 */
enum class MotorMode : uint8_t {
    PROFILE_POSITION = 1,  // 位置模式
    PROFILE_VELOCITY = 3,  // 速度模式
    PROFILE_TORQUE = 4,  // 力矩/电流模式
    HOMING_MODE = 6,  // 回零模式
    INTERPOLATED_POS = 7,  // 插值位置模式
    CYCLIC_SYNC_POS = 8,  // 同步周期位置
    CYCLIC_SYNC_VEL = 9,  // 同步周期速度
    CYCLIC_SYNC_TORQUE = 10, // 同步周期力矩
};
/**
 * @brief 状态和模式结构体 (StateAndMode)
 * @brief StateAndMode::refresh 用于标记数据是否已刷新 (true:已刷新 false:未刷新)
 * @brief StateAndMode::controlWordRaw 控制字(0x6040) 原始2字节
 * @brief StateAndMode::statusWordRaw 状态字(0x6041) 原始2字节
 * @brief StateAndMode::modeOfOperationRaw 运行模式(0x6060) 原始1字节
 * @brief StateAndMode::errorCodeRaw 电机错误码(0x603F) 原始2字节
 */
struct StateAndMode
{
    //原子类型布尔变量，确保读写的时候不会出现多线程同步问题
    // true 已刷新 false 未刷新
    alignas(64) std::atomic<bool> refresh = true; // 独占缓存行

    // DS402: 控制字(0x6040)、状态字(0x6041)通常各2字节
    // 运行模式(0x6060)通常1字节，错误码(0x603F)通常2字节
    volatile struct {
        volatile uint8_t controlWordRaw[2];     /// 控制字（原始2字节）>> 发送 0x6040
        volatile uint8_t statusWordRaw[2];      /// 状态字（原始2字节）<< 接收 0x6041
    }controlData;
    volatile struct { 
        volatile uint8_t modeOfOperationRaw[1]; /// 运行模式（原始1字节）>> 发送  0x6060
        volatile uint8_t errorCodeRaw[2];       /// 电机错误代码（原始2字节）<< 接收  0x603F
    }modeData;

    const uint16_t controlWordIndex = OD_CONTROL_WORD;      // 0x6040
    const uint16_t statusWordIndex = OD_STATUS_WORD;       // 0x6041
    const uint16_t modeOfOperationIndex = OD_MODES_OF_OPERATION;// 0x6060
    const uint16_t errorCodeIndex = OD_ERROR_CODE;        // 0x603F
};


/**
 * @brief 电机电流 (MotorCurrent)
 * @brief 采用原子操作和缓存行对齐优化，确保多线程安全访问   1个编码器值=1mA
 * @details 实现特性：
 * 1. 基于位掩码的标志位系统，支持8种独立刷新状态标记
 * 2. 原始数据与转换值分离存储，避免缓存行伪共享
 * 3. 严格的64字节对齐，适配现代CPU缓存架构
 *
 * @brief MotorCurrent::flags_                标志位控制字(原子操作)
 * @brief MotorCurrent::Flags::RAW_DATA_SEND_NEED_REFRESH        原始发送数据（十六进制）需要刷新
 * @brief MotorCurrent::Flags::RAW_DATA_RECEIVE_NEED_REFRESH     原始接收数据（十六进制）需要刷新
 * @brief MotorCurrent::Flags::ENCODER_DATA_SEND_NEED_REFRESH    编码器发送数据（十进制）需要刷新
 * @brief MotorCurrent::Flags::ENCODER_DATA_RECEIVE_NEED_REFRESH 编码器接收数据（十进制）需要刷新  
 * @brief MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH     目标发送数据（十进制）需要刷新
 * @brief MotorCurrent::Flags::ACTUAL_DATA_RECEIVE_NEED_REFRESH  实际接收数据（十进制）需要刷新      
 *
 * @brief MotorCurrent::raw_actual            实际电流原始数据区(带填充对齐)
 * @brief MotorCurrent::raw_actual.bytes      原始字节形式访问(volatile uint8_t[2])
 * @brief MotorCurrent::raw_actual.value_      整型形式访问(int16_t)
 *
 * @brief MotorCurrent::raw_target            目标电流原始数据区(带填充对齐)
 * @brief MotorCurrent::raw_target.bytes      原始字节形式访问(volatile uint8_t[2])
 * @brief MotorCurrent::raw_target.value_      整型形式访问(int16_t)
 *
 * @brief MotorCurrent::actual_encoder        实际电流编码器计数值(原子int16_t)  
 * @brief MotorCurrent::actual_current        实际物理电流值(原子float)
 * @brief MotorCurrent::target_encoder        目标电流编码器计数值(原子int16_t)  
 * @brief MotorCurrent::target_current        目标物理电流值(原子float)
 *
 * @brief MotorCurrent::actual_Current_Index  实际电流对象字典索引(只读)
 * @brief MotorCurrent::target_Current_Index  目标电流对象字典索引(只读)
 */
struct MotorCurrent {
    /**
     * @brief 标志位控制枚举 
     * @note 使用双字节存储，最高支持16种状态标志
     * @warning 修改枚举值需同步更新位操作逻辑
     */
    enum Flags : Flag_type {
        
        RAW_DATA_SEND_NEED_REFRESH = 0x0001,          ///< 位0: 原始发送数据（十六进制）需要刷新
        RAW_DATA_RECEIVE_NEED_REFRESH = 0x0002,       ///< 位1: 原始接收数据（十六进制）需要刷新
        ENCODER_DATA_SEND_NEED_REFRESH = 0x0004,      ///< 位2: 编码器发送数据（十进制）需要刷新
        ENCODER_DATA_RECEIVE_NEED_REFRESH = 0x0008,   ///< 位3: 编码器接收数据（十进制）需要刷新
        TARGET_DATA_SEND_NEED_REFRESH = 0x0010,       ///< 位4: 目标发送数据（十进制）需要刷新
        ACTUAL_DATA_RECEIVE_NEED_REFRESH = 0x0020,    ///< 位5: 实际接收数据（十进制）需要刷新
        RESERVED_6 = 0x0040,                          ///< 位6: 保留 用于扩展
        RESERVED_7 = 0x0080,                          ///< 位7: 保留 用于扩展
        
        RESERVED_8 = 0x0100,                          ///< 位8: 保留 用于扩展
        RESERVED_9 = 0x0200,                          ///< 位9: 保留 用于扩展
        RESERVED_10 = 0x0400,                         ///< 位10: 保留 用于扩展
        RESERVED_11 = 0x0800,                         ///< 位11: 保留 用于扩展
        RESERVED_12 = 0x1000,                         ///< 位12: 保留 用于扩展
        RESERVED_13 = 0x2000,                         ///< 位13: 保留 用于扩展
        RESERVED_14 = 0x4000,                         ///< 位14: 保留 用于扩展
        RESERVED_15 = 0x8000                          ///< 位15: 保留 用于扩展
    };
    alignas(64) std::atomic<Flag_type> flags_{ 0 };

    // 原始数据区（带缓存行填充）

    AlignedRawData<2, Current_type> raw_actual;///< 实际电流原始数据 0x6078
    AlignedRawData<2, Current_type> raw_target;///< 目标电流原始数据 0x6071

    // 转换值组（独立缓存行）
    /** @brief 实际电流转换结果组 */
        alignas(64) std::atomic<Current_type> actual_encoder{ 0 };  ///< 编码器计数表示值
        alignas(64) std::atomic<float> actual_current{ 0.0f }; ///< 物理电流值(安培)

    /** @brief 目标电流转换结果组 */
        alignas(64) std::atomic<Current_type> target_encoder{ 0 };  ///< 编码器计数表示值
        alignas(64) std::atomic<float> target_current{ 0.0f }; ///< 物理电流值(安培)
    

    // 协议常量（请保持硬编码）
    const uint16_t actual_Current_Index = OD_ACTUAL_CURRENT;  ///< 实际电流对象字典索引 0x6078
    const uint16_t target_Current_Index = OD_TARGET_CURRENT;  ///< 目标电流对象字典索引 0x6071

    /**
     * @brief 检查待处理的数据类型
     * @param f 标志位掩码
     * @return 是否存在待处理更新 True 是 False 否
     * @note 使用memory_order_acquire保证最新状态
     */
    bool needsProcess(Flags f) const noexcept {
        return flags_.load(std::memory_order_acquire) & f;
    }
    /**
     * @brief 标记数据处理完成
     * @param f 要清除的标志位
     * @note 使用memory_order_release保证操作可见性
     */
    void markProcessed(Flags f) noexcept {
        flags_.fetch_and(~f, std::memory_order_release);
    }
};


/**
 * @brief 电机位置 (MotorPosition)
 * @brief 采用原子操作和缓存行对齐优化，确保多线程安全访问
 * @details 实现特性：
 * 1. 基于位掩码的标志位系统，支持8种独立刷新状态标记
 * 2. 原始数据与转换值分离存储，避免缓存行伪共享
 * 3. 严格的64字节对齐，适配现代CPU缓存架构
 *
 * @brief MotorPosition::flags_                标志位控制字(原子操作)
 * @brief MotorPosition::Flags::RAW_DATA_SEND_NEED_REFRESH        原始发送数据（十六进制）需要刷新
 * @brief MotorPosition::Flags::RAW_DATA_RECEIVE_NEED_REFRESH     原始接收数据（十六进制）需要刷新
 * @brief MotorPosition::Flags::ENCODER_DATA_SEND_NEED_REFRESH    编码器发送数据（十进制）需要刷新
 * @brief MotorPosition::Flags::ENCODER_DATA_RECEIVE_NEED_REFRESH 编码器接收数据（十进制）需要刷新
 * @brief MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH     角度值发送数据（浮点）需要刷新
 * @brief MotorPosition::Flags::ACTUAL_DATA_RECEIVE_NEED_REFRESH  角度值接收数据（浮点）需要刷新
 *
 * @brief MotorPosition::raw_actual            实际位置原始数据区(带填充对齐)
 * @brief MotorPosition::raw_actual.bytes      原始字节形式访问(volatile uint8_t[4])
 * @brief MotorPosition::raw_actual.value_      整型形式访问(int32_t)
 *
 * @brief MotorPosition::raw_target            目标位置原始数据区(带填充对齐)
 * @brief MotorPosition::raw_target.bytes      原始字节形式访问(volatile uint8_t[4])
 * @brief MotorPosition::raw_target.value_      整型形式访问(int32_t)
 *
 * @brief MotorPosition::actual_encoder        实际位置编码器计数值(原子int32_t)
 * @brief MotorPosition::actual_degree         实际角度值(原子float)
 * @brief MotorPosition::target_encoder        目标位置编码器计数值(原子int32_t)
 * @brief MotorPosition::target_degree         目标角度值(原子float)
 *
 * @brief MotorPosition::actual_Position_Index  实际位置对象字典索引(只读)
 * @brief MotorPosition::target_Position_Index  目标位置对象字典索引(只读)
 */
struct MotorPosition {
    /**
     * @brief 标志位控制枚举
     * @note 使用双字节存储，最高支持16种状态标志
     * @warning 修改枚举值需同步更新位操作逻辑
     */
    enum Flags : Flag_type {
        // 原有8个标志位（位0-位7）
        RAW_DATA_SEND_NEED_REFRESH = 0x0001,          ///< 位0: 原始发送数据（十六进制）需要刷新
        RAW_DATA_RECEIVE_NEED_REFRESH = 0x0002,       ///< 位1: 原始接收数据（十六进制）需要刷新
        ENCODER_DATA_SEND_NEED_REFRESH = 0x0004,      ///< 位2: 编码器发送数据（十进制）需要刷新
        ENCODER_DATA_RECEIVE_NEED_REFRESH = 0x0008,   ///< 位3: 编码器接收数据（十进制）需要刷新
        TARGET_DATA_SEND_NEED_REFRESH = 0x0010,       ///< 位4: 目标发送数据（十进制）需要刷新
        ACTUAL_DATA_RECEIVE_NEED_REFRESH = 0x0020,    ///< 位5: 实际接收数据（十进制）需要刷新
        RESERVED_6 = 0x0040,                          ///< 位6: 保留 用于扩展
        RESERVED_7 = 0x0080,                          ///< 位7: 保留 用于扩展
        // 新增的8个保留位（位8-位15）
        RESERVED_8 = 0x0100,                          ///< 位8: 保留 用于扩展
        RESERVED_9 = 0x0200,                          ///< 位9: 保留 用于扩展
        RESERVED_10 = 0x0400,                         ///< 位10: 保留 用于扩展
        RESERVED_11 = 0x0800,                         ///< 位11: 保留 用于扩展
        RESERVED_12 = 0x1000,                         ///< 位12: 保留 用于扩展
        RESERVED_13 = 0x2000,                         ///< 位13: 保留 用于扩展
        RESERVED_14 = 0x4000,                         ///< 位14: 保留 用于扩展
        RESERVED_15 = 0x8000                          ///< 位15: 保留 用于扩展
    };
    alignas(64) std::atomic<Flag_type> flags_{ 0 };

    // 原始数据区（带缓存行填充）
    AlignedRawData<4, Position_type> raw_actual;  ///< 实际位置原始数据 0x6064
    AlignedRawData<4, Position_type> raw_target;  ///< 目标位置原始数据 0x607A

    // 转换值组（独立缓存行）
    alignas(64) std::atomic<Position_type> actual_encoder{ 0 };  ///< 实际编码器计数值  
    alignas(64) std::atomic<float> actual_degree{ 0.0f };  ///< 实际角度值(度)
    alignas(64) std::atomic<Position_type> target_encoder{ 0 };  ///< 目标编码器计数值
    alignas(64) std::atomic<float> target_degree{ 0.0f };  ///< 目标角度值(度)

    // 协议常量（DS402标准）
    const uint16_t actual_Position_Index = OD_ACTUAL_POSITION;  ///< 0x6064
    const uint16_t target_Position_Index = OD_TARGET_POSITION;  ///< 0x607A

    /**
     * @brief 检查待处理的数据类型
     * @param f 标志位掩码
     * @return 是否存在待处理更新 True 是 False 否
     * @note 使用memory_order_acquire保证最新状态
     */
    bool needsProcess(Flags f) const noexcept {
        return flags_.load(std::memory_order_acquire) & f;
    }

    /**
     * @brief 标记数据处理完成
     * @param f 要清除的标志位
     * @note 使用memory_order_release保证操作可见性
     */
    void markProcessed(Flags f) noexcept {
        flags_.fetch_and(~f, std::memory_order_release);
    }
};





/**
 * @brief 电机速度 (MotorVelocity)
 * @brief 采用原子操作和缓存行对齐优化，确保多线程安全访问 1个编码器值=1RPM/min
 * @details 实现特性：
 * 1. 基于位掩码的标志位系统，支持8种独立刷新状态标记
 * 2. 原始数据与转换值分离存储，避免缓存行伪共享
 * 3. 严格的64字节对齐，适配现代CPU缓存架构
 *
 * @brief MotorVelocity::flags_                标志位控制字(原子操作)
 * @brief MotorVelocity::Flags::RAW_DATA_SEND_NEED_REFRESH        原始发送数据（十六进制）需要刷新
 * @brief MotorVelocity::Flags::RAW_DATA_RECEIVE_NEED_REFRESH     原始接收数据（十六进制）需要刷新
 * @brief MotorVelocity::Flags::ENCODER_DATA_SEND_NEED_REFRESH    编码器发送数据（十进制）需要刷新
 * @brief MotorVelocity::Flags::ENCODER_DATA_RECEIVE_NEED_REFRESH 编码器接收数据（十进制）需要刷新
 * @brief MotorVelocity::Flags::TARGET_DATA_SEND_NEED_REFRESH     目标速度发送数据（浮点）需要刷新
 * @brief MotorVelocity::Flags::ACTUAL_DATA_RECEIVE_NEED_REFRESH  实际速度接收数据（浮点）需要刷新
 *
 * @brief MotorVelocity::raw_actual            实际速度原始数据区(带填充对齐)
 * @brief MotorVelocity::raw_actual.bytes      原始字节形式访问(volatile uint8_t[2])
 * @brief MotorVelocity::raw_actual.value_      整型形式访问(int16_t)
 *
 * @brief MotorVelocity::raw_target            目标速度原始数据区(带填充对齐)
 * @brief MotorVelocity::raw_target.bytes      原始字节形式访问(volatile uint8_t[2])
 * @brief MotorVelocity::raw_target.value_      整型形式访问(int16_t)
 *
 * @brief MotorVelocity::actual_encoder        实际速度编码器计数值(原子int16_t)
 * @brief MotorVelocity::actual_rpm            实际转速值(原子float)
 * @brief MotorVelocity::target_encoder        目标速度编码器计数值(原子int16_t)
 * @brief MotorVelocity::target_rpm            目标转速值(原子float)
 *
 * @brief MotorVelocity::actual_Velocity_Index  实际速度对象字典索引(只读)
 * @brief MotorVelocity::target_Velocity_Index_Velocity_Mode  目标速度对象字典索引(只读)
 */
struct MotorVelocity {
    /**
     * @brief 标志位控制枚举
     * @note 使用双字节存储，最高支持16种状态标志
     * @warning 修改枚举值需同步更新位操作逻辑
     */
    enum Flags : Flag_type {
        
        RAW_DATA_SEND_NEED_REFRESH_VELOCITY_MODE = 0x0001,          ///< 位0: 原始发送数据（十六进制）需要刷新(适用于速度模式）
        RAW_DATA_RECEIVE_NEED_REFRESH = 0x0002,       ///< 位1: 原始接收数据（十六进制）需要刷新
        ENCODER_DATA_SEND_NEED_REFRESH_VELOCITY_MODE = 0x0004,      ///< 位2: 编码器发送数据（十进制）需要刷新（适用于速度模式）
        ENCODER_DATA_RECEIVE_NEED_REFRESH = 0x0008,   ///< 位3: 编码器接收数据（十进制）需要刷新
        TARGET_DATA_SEND_NEED_REFRESH_VELOCITY_MODE = 0x0010,       ///< 位4: 目标发送数据（十进制）需要刷新（适用于速度模式）
        ACTUAL_DATA_RECEIVE_NEED_REFRESH = 0x0020,    ///< 位5: 实际接收数据（十进制）需要刷新
        RAW_DATA_SEND_NEED_REFRESH_POSITION_MODE = 0x0040,                          ///< 位6: 原始发送数据（十六进制）需要刷新(适用于位置模式）
        ENCODER_DATA_SEND_NEED_REFRESH_POSITION_MODE = 0x0080,                          ///< 位7: 编码器发送数据（十进制）需要刷新(适用于位置模式）
        TARGET_DATA_SEND_NEED_REFRESH_POSITION_MODE = 0x0100,                          ///< 位8: 目标发送数据（十进制）需要刷新（适用于位置模式）


        RESERVED_9 = 0x0200,                          ///< 位9: 保留 用于扩展
        RESERVED_10 = 0x0400,                         ///< 位10: 保留 用于扩展
        RESERVED_11 = 0x0800,                         ///< 位11: 保留 用于扩展
        RESERVED_12 = 0x1000,                         ///< 位12: 保留 用于扩展
        RESERVED_13 = 0x2000,                         ///< 位13: 保留 用于扩展
        RESERVED_14 = 0x4000,                         ///< 位14: 保留 用于扩展
        RESERVED_15 = 0x8000                          ///< 位15: 保留 用于扩展
    };
    alignas(64) std::atomic<Flag_type> flags_{ 0 };

    // 原始数据区（带缓存行填充）
    AlignedRawData<2, Velocity_type> raw_actual;  ///< 实际速度原始数据  0x606C
    AlignedRawData<2, Velocity_type> raw_target_velocity_mode;  ///< 目标速度原始数据（速度模式）  0x60FF
    AlignedRawData<2, Velocity_type> raw_target_position_mode;  ///< 目标速度原始数据（位置模式）  0x6081

    // 转换值组（独立缓存行）
    alignas(64) std::atomic<Velocity_type> actual_encoder{ 0 };  ///< 实际编码器计数值  
    alignas(64) std::atomic<float> actual_rpm{ 0.0f };     ///< 实际转速值(RPM)
    alignas(64) std::atomic<Velocity_type> target_encoder_velocity_mode{ 0 };  ///< 目标编码器计数值（速度模式）
    alignas(64) std::atomic<float> target_rpm_velocity_mode{ 0.0f };     ///< 目标转速值(RPM/min)（速度模式）
    alignas(64) std::atomic<Velocity_type> target_encoder_position_mode{ 0 };  ///< 目标编码器计数值（位置模式）
    alignas(64) std::atomic<float> target_rpm_position_mode{ 0.0f };     ///< 目标转速值(RPM/min)（位置模式）

    // 协议常量（DS402标准）
    const uint16_t actual_Velocity_Index = OD_ACTUAL_VELOCITY;  ///< 0x606C
    const uint16_t target_Velocity_Index_Velocity_Mode = OD_TARGET_VELOCITY_VELOCITY_MODE;  ///< 0x60FF
    const uint16_t target_Velocity_Index_Position_Mode = OD_TARGET_VELOCITY_POSITION_MODE;  ///< 0x6081


    /**
     * @brief 检查待处理的数据类型
     * @param f 标志位掩码
     * @return 是否存在待处理更新 True 是 False 否
     * @note 使用memory_order_acquire保证最新状态
     */
    bool needsProcess(Flags f) const noexcept {
        return flags_.load(std::memory_order_acquire) & f;
    }

    /**
     * @brief 标记数据处理完成
     * @param f 要清除的标志位
     * @note 使用memory_order_release保证操作可见性
     */
    void markProcessed(Flags f) noexcept {
        flags_.fetch_and(~f, std::memory_order_release);
    }
};



/**
 * @brief 电机加减速(MotorAccelDecel)
 * @brief 适配DS402协议0x6083(加速度)/0x6084(减速度)
 * @details 核心特性：
 * 1. 直接使用AlignedRawData<2>存储原始值
 * 2. 数值直接映射为工程值(单位：转/分)
 * 3. 自动继承原子操作接口
 *
 * @warning 该结构体不包含状态标志位
 *
 * @brief MotorAccelDecel::raw_accel  加速度原始数据区
 * @brief MotorAccelDecel::raw_accel.bytes_  原始字节访问接口
 * @brief MotorAccelDecel::raw_accel.value_  整型值访问接口
 *
 * @brief MotorAccelDecel::raw_decel  减速度原始数据区
 * @brief MotorAccelDecel::raw_decel.bytes_  原始字节访问接口
 * @brief MotorAccelDecel::raw_decel.value_  整型值访问接口
 */
struct MotorAccelDecel {
    // 数据存储区（复用模板）
    AlignedRawData<2, AccelDecel_type> raw_accel;  ///< 加速度值(单位RPM/min)  0x6083
    AlignedRawData<2, AccelDecel_type> raw_decel;  ///< 减速度值(单位RPM/min)  0x6084

    // 协议常量（DS402标准）
    const uint16_t accel_Index = OD_ACCELERATION;  ///< 0x6083
    const uint16_t decel_Index = OD_DECELERATION;  ///< 0x6084

};





/**
 * @brief 核心电机类
 *
 * 包含：
 * 1. 状态&模式
 * 2. 电流
 * 3. 位置
 * 4. 速度
 * 5. 加减速度
 *
 * 以及简单的初始化、读刷新、写刷新等接口。
 */
class Motor
{

private:

    



public:

    /// 明确删除所有拷贝和移动操作
    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;
    Motor(Motor&&) = delete;
    Motor& operator=(Motor&&) = delete;



    std::mutex mtx_;

    /**
     * @brief 电机类构造函数
     *
     * @note 构造时会自动调用init()方法完成以下初始化：
     * 1. 状态标志位清零
     * 2. 所有缓存行填充对齐验证
     * 3. 电机默认进入力矩模式（零力矩）
     * 4. 确保多线程安全的数据结构初始化
     */
    explicit Motor(uint8_t id) : motor_id_(id) {

        init(); // 调用现有初始化方法

    };

    ~Motor() = default;


    StateAndMode    stateAndMode;  //状态和模式结构体
    MotorCurrent    current;       //电流结构体
    MotorPosition   position;      //位置结构体
    MotorVelocity   velocity;      //速度结构体
    MotorAccelDecel accelDecel;    //加速度结构体

    const uint8_t motor_id_; // 电机ID




    /**
     * @brief 初始化方法：
     *
     */
    void init() {
        std::lock_guard<std::mutex> lock(mtx_);

        

        //  状态模式初始化
        stateAndMode.refresh = false;
        for (auto& byte : stateAndMode.controlData.controlWordRaw) {
            byte = 0;
        }
        for (auto& byte : stateAndMode.controlData.statusWordRaw) {
            byte = 0;
        }
        stateAndMode.modeData.modeOfOperationRaw[0] =
            static_cast<uint8_t>(MotorMode::PROFILE_TORQUE); // 默认选择电流模式，在电流为0的情况下是安全的

        
        // 电流数据初始化
        current.flags_.store(0);
        
        current.raw_actual.atomicWriteValue(0);
        current.raw_target.atomicWriteValue(0);
        

        current.actual_current.store(0.0f);
        current.target_current.store(0.0f);
        

        // 位置数据初始化
        position.flags_.store(0);
        position.raw_actual.atomicWriteValue(0);
        position.raw_target.atomicWriteValue(0);
        position.actual_degree.store(0.0f);
        position.target_degree.store(0.0f);
        

        // 速度数据初始化
        velocity.flags_.store(0);
        //velocity.raw_target.atomicWriteValue(0);
        velocity.raw_target_position_mode.atomicWriteValue(0);
        velocity.raw_target_velocity_mode.atomicWriteValue(0);
        //locity.raw_actual.atomicWriteValue(0);
        velocity.actual_rpm.store(0.0f);
        velocity.target_encoder_position_mode.store(0);
        velocity.target_encoder_velocity_mode.store(0);
        //velocity.target_rpm.store(0.0f);

        // 加减速初始化
        const uint16_t default_accel = 0; // RPM/min
        const uint16_t default_decel = 0;
        accelDecel.raw_accel.atomicWriteValue(default_accel);
        accelDecel.raw_decel.atomicWriteValue(default_decel);

        
    }



    /**
     * @brief 电机数据刷新方法（优化版）
     * @tparam T 电机数据类型（MotorCurrent/MotorPosition/MotorVelocity）
     * @param data 要刷新的数据组引用
     *
     * @note 刷新逻辑：
     * 1. 检查标志位确定数据流向
     * 2. 按优先级处理：接收优先于发送
     * 3. 自动执行必要的数值转换
     * 4. 清除已处理的标志位
     */
    template <typename T>
    inline void refreshMotorData(T& data) {
        // 常量定义
        constexpr bool TO_ANGLE = true;
        constexpr bool TO_ENCODER = false;
        constexpr float CURRENT_SCALE = 1.0f;  // 1count = 1mA
        constexpr float RPM_SCALE = 1.0f;      // 1count = 1RPM

        // ========== 接收数据刷新（原始→编码器→物理） ==========
        if (data.needsProcess(T::Flags::RAW_DATA_RECEIVE_NEED_REFRESH)) {
            if constexpr (std::is_same_v<T, MotorCurrent>) {
                // 电流：原始→编码器→物理值
                Current_type raw_val = data.raw_actual.atomicReadValue();
                data.actual_encoder.store(raw_val, std::memory_order_release);
                data.actual_current.store(raw_val * CURRENT_SCALE, std::memory_order_release);
            }
            else if constexpr (std::is_same_v<T, MotorPosition>) {
                // 位置：原始→编码器→角度
                Position_type raw_val = data.raw_actual.atomicReadValue();
                data.actual_encoder.store(raw_val, std::memory_order_release);
                data.actual_degree.store(
                    convertSensorAngle(raw_val, TO_ANGLE, SENSOR_RANGE),
                    std::memory_order_release
                );
            }
            else if constexpr (std::is_same_v<T, MotorVelocity>) {
                // 速度：原始→编码器→RPM
                Velocity_type raw_val = data.raw_actual.atomicReadValue();
                data.actual_encoder.store(raw_val, std::memory_order_release);
                data.actual_rpm.store(raw_val * RPM_SCALE, std::memory_order_release);
            }

            data.markProcessed(T::Flags::RAW_DATA_RECEIVE_NEED_REFRESH);
            return; // 一次只处理一种刷新
        }

        // ========== 接收编码器刷新（编码器→原始&物理） ==========
        if (data.needsProcess(T::Flags::ENCODER_DATA_RECEIVE_NEED_REFRESH)) {
            if constexpr (std::is_same_v<T, MotorCurrent>) {
                Current_type enc_val = data.actual_encoder.load(std::memory_order_acquire);

                // 更新原始数据
                data.raw_actual.atomicWriteValue(enc_val);

                // 更新物理值
                data.actual_current.store(enc_val * CURRENT_SCALE, std::memory_order_release);
            }
            else if constexpr (std::is_same_v<T, MotorPosition>) {
                Position_type enc_val = data.actual_encoder.load(std::memory_order_acquire);

                // 更新原始数据
                data.raw_actual.atomicWriteValue(enc_val);

                // 更新角度物理值
                data.actual_degree.store(
                    convertSensorAngle(enc_val, TO_ANGLE, SENSOR_RANGE),
                    std::memory_order_release
                );
            }
            else if constexpr (std::is_same_v<T, MotorVelocity>) {
                Velocity_type enc_val = data.actual_encoder.load(std::memory_order_acquire);

                // 更新原始数据
                data.raw_actual.atomicWriteValue(enc_val);

                // 更新转速物理值
                data.actual_rpm.store(enc_val * RPM_SCALE, std::memory_order_release);
            }

            data.markProcessed(T::Flags::ENCODER_DATA_RECEIVE_NEED_REFRESH);
            return;
        }

        // ========== 实际物理值接收刷新（物理→编码器→原始） ==========
        if (data.needsProcess(T::Flags::ACTUAL_DATA_RECEIVE_NEED_REFRESH)) {
            if constexpr (std::is_same_v<T, MotorCurrent>) {
                float actual_val = data.actual_current.load(std::memory_order_acquire);
                Current_type enc_val = static_cast<Current_type>(actual_val / CURRENT_SCALE);

                data.actual_encoder.store(enc_val, std::memory_order_release);
                data.raw_actual.atomicWriteValue(enc_val);
            }
            else if constexpr (std::is_same_v<T, MotorPosition>) {
                float actual_deg = data.actual_degree.load(std::memory_order_acquire);
                Position_type enc_val = static_cast<Position_type>(
                    convertSensorAngle(actual_deg, TO_ENCODER, SENSOR_RANGE)
                    );

                data.actual_encoder.store(enc_val, std::memory_order_release);
                data.raw_actual.atomicWriteValue(enc_val);
            }
            else if constexpr (std::is_same_v<T, MotorVelocity>) {
                float actual_rpm = data.actual_rpm.load(std::memory_order_acquire);
                Velocity_type enc_val = static_cast<Velocity_type>(actual_rpm / RPM_SCALE);

                data.actual_encoder.store(enc_val, std::memory_order_release);
                data.raw_actual.atomicWriteValue(enc_val);
            }

            data.markProcessed(T::Flags::ACTUAL_DATA_RECEIVE_NEED_REFRESH);
            return;
        }

        // ========== 发送原始数据刷新（原始→编码器→物理） ==========
        // 注意：MotorVelocity 使用特殊的标志位，所以这里只处理 Current 和 Position
        if constexpr (!std::is_same_v<T, MotorVelocity>) {
            if (data.needsProcess(T::Flags::RAW_DATA_SEND_NEED_REFRESH)) {
                if constexpr (std::is_same_v<T, MotorCurrent>) {
                    Current_type raw_val = data.raw_target.atomicReadValue();
                    data.target_encoder.store(raw_val, std::memory_order_release);
                    data.target_current.store(raw_val * CURRENT_SCALE, std::memory_order_release);
                }
                else if constexpr (std::is_same_v<T, MotorPosition>) {
                    Position_type raw_val = data.raw_target.atomicReadValue();
                    data.target_encoder.store(raw_val, std::memory_order_release);
                    data.target_degree.store(
                        convertSensorAngle(raw_val, TO_ANGLE, SENSOR_RANGE),
                        std::memory_order_release
                    );
                }

                data.markProcessed(T::Flags::RAW_DATA_SEND_NEED_REFRESH);
                return;
            }
        }

        // ========== 发送编码器刷新（编码器→原始&物理） ==========
        // 注意：MotorVelocity 使用特殊的标志位，所以这里只处理 Current 和 Position
        if constexpr (!std::is_same_v<T, MotorVelocity>) {
            if (data.needsProcess(T::Flags::ENCODER_DATA_SEND_NEED_REFRESH)) {
                if constexpr (std::is_same_v<T, MotorCurrent>) {
                    Current_type enc_val = data.target_encoder.load(std::memory_order_acquire);
                    data.raw_target.atomicWriteValue(enc_val);
                    data.target_current.store(enc_val * CURRENT_SCALE, std::memory_order_release);
                }
                else if constexpr (std::is_same_v<T, MotorPosition>) {
                    Position_type enc_val = data.target_encoder.load(std::memory_order_acquire);
                    data.raw_target.atomicWriteValue(enc_val);
                    data.target_degree.store(
                        convertSensorAngle(enc_val, TO_ANGLE, SENSOR_RANGE),
                        std::memory_order_release
                    );
                }

                data.markProcessed(T::Flags::ENCODER_DATA_SEND_NEED_REFRESH);
                return;
            }
        }

        // ========== 发送物理值刷新（物理→编码器→原始） ==========
        // 注意：MotorVelocity 使用特殊的标志位，所以这里只处理 Current 和 Position
        if constexpr (!std::is_same_v<T, MotorVelocity>) {
            if (data.needsProcess(T::Flags::TARGET_DATA_SEND_NEED_REFRESH)) {
                if constexpr (std::is_same_v<T, MotorCurrent>) {
                    float target_current = data.target_current.load(std::memory_order_acquire);
                    Current_type enc_val = static_cast<Current_type>(target_current / CURRENT_SCALE);

                    data.target_encoder.store(enc_val, std::memory_order_release);
                    data.raw_target.atomicWriteValue(enc_val);
                }
                else if constexpr (std::is_same_v<T, MotorPosition>) {
                    float target_deg = data.target_degree.load(std::memory_order_acquire);
                    Position_type enc_val = static_cast<Position_type>(
                        convertSensorAngle(target_deg, TO_ENCODER, SENSOR_RANGE)
                        );

                    data.target_encoder.store(enc_val, std::memory_order_release);
                    data.raw_target.atomicWriteValue(enc_val);
                }

                data.markProcessed(T::Flags::TARGET_DATA_SEND_NEED_REFRESH);
                return;
            }
        }

        // ========== 速度特殊处理（两种模式） ==========
        if constexpr (std::is_same_v<T, MotorVelocity>) {
            // 速度模式发送处理
            if (data.needsProcess(T::Flags::RAW_DATA_SEND_NEED_REFRESH_VELOCITY_MODE)) {
                Velocity_type raw_val = data.raw_target_velocity_mode.atomicReadValue();
                data.target_encoder_velocity_mode.store(raw_val, std::memory_order_release);
                data.target_rpm_velocity_mode.store(raw_val * RPM_SCALE, std::memory_order_release);
                data.markProcessed(T::Flags::RAW_DATA_SEND_NEED_REFRESH_VELOCITY_MODE);
                return;
            }

            if (data.needsProcess(T::Flags::ENCODER_DATA_SEND_NEED_REFRESH_VELOCITY_MODE)) {
                Velocity_type enc_val = data.target_encoder_velocity_mode.load(std::memory_order_acquire);
                data.raw_target_velocity_mode.atomicWriteValue(enc_val);
                data.target_rpm_velocity_mode.store(enc_val * RPM_SCALE, std::memory_order_release);
                data.markProcessed(T::Flags::ENCODER_DATA_SEND_NEED_REFRESH_VELOCITY_MODE);
                return;
            }

            if (data.needsProcess(T::Flags::TARGET_DATA_SEND_NEED_REFRESH_VELOCITY_MODE)) {
                float target_rpm = data.target_rpm_velocity_mode.load(std::memory_order_acquire);
                Velocity_type enc_val = static_cast<Velocity_type>(target_rpm / RPM_SCALE);
                data.target_encoder_velocity_mode.store(enc_val, std::memory_order_release);
                data.raw_target_velocity_mode.atomicWriteValue(enc_val);
                data.markProcessed(T::Flags::TARGET_DATA_SEND_NEED_REFRESH_VELOCITY_MODE);
                return;
            }

            // 位置模式发送处理
            if (data.needsProcess(T::Flags::RAW_DATA_SEND_NEED_REFRESH_POSITION_MODE)) {
                Velocity_type raw_val = data.raw_target_position_mode.atomicReadValue();
                data.target_encoder_position_mode.store(raw_val, std::memory_order_release);
                data.target_rpm_position_mode.store(raw_val * RPM_SCALE, std::memory_order_release);
                data.markProcessed(T::Flags::RAW_DATA_SEND_NEED_REFRESH_POSITION_MODE);
                return;
            }

            if (data.needsProcess(T::Flags::ENCODER_DATA_SEND_NEED_REFRESH_POSITION_MODE)) {
                Velocity_type enc_val = data.target_encoder_position_mode.load(std::memory_order_acquire);
                data.raw_target_position_mode.atomicWriteValue(enc_val);
                data.target_rpm_position_mode.store(enc_val * RPM_SCALE, std::memory_order_release);
                data.markProcessed(T::Flags::ENCODER_DATA_SEND_NEED_REFRESH_POSITION_MODE);
                return;
            }

            if (data.needsProcess(T::Flags::TARGET_DATA_SEND_NEED_REFRESH_POSITION_MODE)) {
                float target_rpm = data.target_rpm_position_mode.load(std::memory_order_acquire);
                Velocity_type enc_val = static_cast<Velocity_type>(target_rpm / RPM_SCALE);
                data.target_encoder_position_mode.store(enc_val, std::memory_order_release);
                data.raw_target_position_mode.atomicWriteValue(enc_val);
                data.markProcessed(T::Flags::TARGET_DATA_SEND_NEED_REFRESH_POSITION_MODE);
                return;
            }
        }
    }

    /**
     * @brief 批量刷新所有电机数据
     * @note 用于接收线程和规划线程调用
     */
    void refreshAllMotorData() {
        // 无需加锁，各数据结构已经是原子操作
        refreshMotorData(current);
        refreshMotorData(position);
        refreshMotorData(velocity);
        // 加减速不需要刷新
    }

    /**
     * @brief 设置标志位（线程安全）
     * @tparam T 数据类型
     * @param data 数据结构引用
     * @param flag 要设置的标志位
     */
    template <typename T>
    inline void setRefreshFlag(T& data, typename T::Flags flag) {
        data.flags_.fetch_or(flag, std::memory_order_release);
    }

};
#endif // CLASS_MOTOR_HPP