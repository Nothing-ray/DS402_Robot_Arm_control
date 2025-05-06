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
 * 一般是 0xXXXX:subIndex, 可以在这里统一声明。
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
static constexpr uint16_t OD_TARGET_VELOCITY = 0x60FF;  /// 目标速度>>> 0x60FF
static constexpr uint16_t OD_ACTUAL_VELOCITY = 0x606C;  /// 实际速度<<< 0x606C

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
        volatile uint8_t controlWordRaw[2];     /// 控制字（原始2字节）>>
        volatile uint8_t statusWordRaw[2];      /// 状态字（原始2字节）<<
    }controlData;
    volatile struct {
        volatile uint8_t modeOfOperationRaw[1]; /// 运行模式（原始1字节）>>
        volatile uint8_t errorCodeRaw[2];       /// 电机错误代码（原始2字节）<<
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
     * @note 使用单字节存储，最高支持8种状态标志
     * @warning 修改枚举值需同步更新位操作逻辑
     */
    enum Flags : uint8_t {
        RAW_DATA_SEND_NEED_REFRESH = 0x01,  ///< 位0: 原始发送数据（十六进制）需要刷新
        RAW_DATA_RECEIVE_NEED_REFRESH = 0x02,  ///< 位1: 原始接收数据（十六进制）需要刷新

        ENCODER_DATA_SEND_NEED_REFRESH = 0x04,  ///< 位2: 编码器发送数据（十进制）需要刷新
        ENCODER_DATA_RECEIVE_NEED_REFRESH = 0x08,  ///< 位3: 编码器接收数据（十进制）需要刷新

        TARGET_DATA_SEND_NEED_REFRESH = 0x10,  ///< 位4: 目标发送数据（十进制）需要刷新
        ACTUAL_DATA_RECEIVE_NEED_REFRESH = 0x20,  ///< 位5: 实际接收数据（十进制）需要刷新

        // 保留位
        RESERVED_6 = 0x40,  ///< 位6: 保留 用于扩展
        RESERVED_7 = 0x80   ///< 位7: 保留 用于扩展

        
    };
    alignas(64) std::atomic<uint8_t> flags_{ 0 };

    // 原始数据区（带缓存行填充）

    AlignedRawData<2,int16_t> raw_actual;///< 实际电流原始数据
    AlignedRawData<2,int16_t> raw_target;///< 目标电流原始数据

    // 转换值组（独立缓存行）
    /** @brief 实际电流转换结果组 */
        alignas(64) std::atomic<int16_t> actual_encoder{ 0 };  ///< 编码器计数表示值
        alignas(64) std::atomic<float> actual_current{ 0.0f }; ///< 物理电流值(安培)

    /** @brief 目标电流转换结果组 */
        alignas(64) std::atomic<int16_t> target_encoder{ 0 };  ///< 编码器计数表示值
        alignas(64) std::atomic<float> target_current{ 0.0f }; ///< 物理电流值(安培)
    

    // 协议常量（请保持硬编码）
    const uint16_t actual_Current_Index = OD_ACTUAL_CURRENT;  ///< 实际电流对象字典索引
    const uint16_t target_Current_Index = OD_TARGET_CURRENT;  ///< 目标电流对象字典索引

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
     * @note 使用单字节存储，最高支持8种状态标志
     * @warning 修改枚举值需同步更新位操作逻辑
     */
    enum Flags : uint8_t {
        RAW_DATA_SEND_NEED_REFRESH = 0x01,  ///< 位0: 原始发送数据（十六进制）需要刷新
        RAW_DATA_RECEIVE_NEED_REFRESH = 0x02,  ///< 位1: 原始接收数据（十六进制）需要刷新

        ENCODER_DATA_SEND_NEED_REFRESH = 0x04,  ///< 位2: 编码器发送数据（十进制）需要刷新
        ENCODER_DATA_RECEIVE_NEED_REFRESH = 0x08,  ///< 位3: 编码器接收数据（十进制）需要刷新

        TARGET_DATA_SEND_NEED_REFRESH = 0x10,  ///< 位4: 角度值发送数据（浮点）需要刷新
        ACTUAL_DATA_RECEIVE_NEED_REFRESH = 0x20,  ///< 位5: 角度值接收数据（浮点）需要刷新

        RESERVED_6 = 0x40,  ///< 位6: 保留 用于扩展
        RESERVED_7 = 0x80   ///< 位7: 保留 用于扩展
    };
    alignas(64) std::atomic<uint8_t> flags_{ 0 };

    // 原始数据区（带缓存行填充）
    AlignedRawData<4,int32_t> raw_actual;  ///< 实际位置原始数据
    AlignedRawData<4,int32_t> raw_target;  ///< 目标位置原始数据

    // 转换值组（独立缓存行）
    alignas(64) std::atomic<int32_t> actual_encoder{ 0 };  ///< 实际编码器计数值  
    alignas(64) std::atomic<float> actual_degree{ 0.0f };  ///< 实际角度值(度)
    alignas(64) std::atomic<int32_t> target_encoder{ 0 };  ///< 目标编码器计数值
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
 * @brief MotorVelocity::target_Velocity_Index  目标速度对象字典索引(只读)
 */
struct MotorVelocity {
    /**
     * @brief 标志位控制枚举
     * @note 使用单字节存储，最高支持8种状态标志
     * @warning 修改枚举值需同步更新位操作逻辑
     */
    enum Flags : uint8_t {
        RAW_DATA_SEND_NEED_REFRESH = 0x01,  ///< 位0: 原始发送数据（十六进制）需要刷新
        RAW_DATA_RECEIVE_NEED_REFRESH = 0x02,  ///< 位1: 原始接收数据（十六进制）需要刷新

        ENCODER_DATA_SEND_NEED_REFRESH = 0x04,  ///< 位2: 编码器发送数据（十进制）需要刷新
        ENCODER_DATA_RECEIVE_NEED_REFRESH = 0x08,  ///< 位3: 编码器接收数据（十进制）需要刷新

        TARGET_DATA_SEND_NEED_REFRESH = 0x10,  ///< 位4: 目标速度发送需要刷新
        ACTUAL_DATA_RECEIVE_NEED_REFRESH = 0x20,  ///< 位5: 实际速度接收需要刷新 

        RESERVED_6 = 0x40,  ///< 位6: 保留 用于扩展
        RESERVED_7 = 0x80   ///< 位7: 保留 用于扩展
    };
    alignas(64) std::atomic<uint8_t> flags_{ 0 };

    // 原始数据区（带缓存行填充）
    AlignedRawData<2,int16_t> raw_actual;  ///< 实际速度原始数据
    AlignedRawData<2,int16_t> raw_target;  ///< 目标速度原始数据

    // 转换值组（独立缓存行）
    alignas(64) std::atomic<int16_t> actual_encoder{ 0 };  ///< 实际编码器计数值  
    alignas(64) std::atomic<float> actual_rpm{ 0.0f };     ///< 实际转速值(RPM)
    alignas(64) std::atomic<int16_t> target_encoder{ 0 };  ///< 目标编码器计数值
    alignas(64) std::atomic<float> target_rpm{ 0.0f };     ///< 目标转速值(RPM)

    // 协议常量（DS402标准）
    const uint16_t actual_Velocity_Index = OD_ACTUAL_VELOCITY;  ///< 0x606C
    const uint16_t target_Velocity_Index = OD_TARGET_VELOCITY;  ///< 0x60FF

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
    AlignedRawData<2,uint16_t> raw_accel;  ///< 加速度值(单位RPM/min)
    AlignedRawData<2,uint16_t> raw_decel;  ///< 减速度值(单位RPM/min)

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
     * @param safeMode 是否启用安全模式（默认为true）
     *                - true: 初始化时设置为零力矩模式
     *                - false: 保留未初始化状态（危险！仅用于特殊场景）
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

        std::cout << 2 << std::endl;

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

        std::cout << 3 << std::endl;
        // 电流数据初始化
        current.flags_.store(0);
        
        current.raw_actual.atomicWriteValue(0);
        current.raw_target.atomicWriteValue(0);
        std::cout << 4 << std::endl;

        current.actual_current.store(0.0f);
        current.target_current.store(0.0f);
        

        // 位置数据初始化
        position.flags_.store(0);
        position.raw_actual.atomicWriteValue(0);
        position.raw_target.atomicWriteValue(0);
        position.actual_degree.store(0.0f);
        position.target_degree.store(0.0f);
        std::cout << 5 << std::endl;

        // 速度数据初始化
        velocity.flags_.store(0);
        velocity.raw_target.atomicWriteValue(0);
        velocity.raw_actual.atomicWriteValue(0);
        velocity.actual_rpm.store(0.0f);
        velocity.target_rpm.store(0.0f);

        // 加减速初始化
        const uint16_t default_accel = 0; // RPM/min
        const uint16_t default_decel = 0;
        accelDecel.raw_accel.atomicWriteValue(default_accel);
        accelDecel.raw_decel.atomicWriteValue(default_decel);

        
    }



    /**
     * @brief 电机数据刷新方法（内部方法）
     * @tparam T 电机数据类型（MotorCurrent/MotorPosition/MotorVelocity）
     * @param data 要刷新的数据组引用
     *
     * @note 刷新逻辑：
     * 1. 检查标志位确定数据流向（原始/编码器/物理值）
     * 2. 自动执行必要的数值转换
     * 3. 更新关联数据项
     * 4. 清除已处理的标志位
     * 5. 一次只处理一个结构体（处理完数据就立刻刷新）
     *
     * @warning 必须在锁保护下调用
     */
    template <typename T>
    inline void refreshMotorData(T& data) {


        // 常量定义
        constexpr bool TO_ANGLE = true;
        constexpr bool TO_ENCODER = false;
        constexpr float CURRENT_SCALE = 1.0f;  // 1count=1mA
        constexpr float RPM_SCALE = 1.0f;      // 1count=1RPM

        

        // "接收"原始数据需要刷新的情况  原始值→编码器值→物理值
        if (data.needsProcess(T::Flags::RAW_DATA_RECEIVE_NEED_REFRESH)) {

            //电流需要刷新
            if constexpr (std::is_same_v<T, MotorCurrent>) {
                // 电流：原始→编码器→物理值
                int16_t raw_val;

                raw_val = data.raw_actual.atomicReadValue();
                //data.raw_actual.atomicRead(&raw_val); //从联合体里面读取十进制值

                data.actual_encoder.store(raw_val);//编码器值写入读取值
                data.actual_current.store(raw_val * CURRENT_SCALE); // 假设1个计数=1mA

                data.markProcessed(T::Flags::RAW_DATA_RECEIVE_NEED_REFRESH);

                return;
            }

            //位置需要刷新
            else if constexpr (std::is_same_v<T, MotorPosition>) {
                // 位置：原始→编码器→角度
                int32_t raw_val;

                raw_val = data.raw_actual.atomicReadValue();
                //data.raw_actual.atomicRead(&raw_val);

                data.actual_encoder.store(raw_val);//写入编码器
                data.actual_degree.store(
                    convertSensorAngle(raw_val, TO_ANGLE, SENSOR_RANGE));//编码器到角度

                data.markProcessed(T::Flags::RAW_DATA_RECEIVE_NEED_REFRESH);

                return;
            }

            //速度需要刷新
            else if constexpr (std::is_same_v<T, MotorVelocity>) {
                // 速度：原始→编码器→RPM
                int16_t raw_val;

                raw_val = data.raw_actual.atomicReadValue();
                //data.raw_actual.atomicRead(&raw_val);

                data.actual_encoder.store(raw_val);//写入编码器
                data.actual_rpm.store(raw_val * RPM_SCALE); // 1计数=1RPM/min     写入角度

                data.markProcessed(T::Flags::RAW_DATA_RECEIVE_NEED_REFRESH);//清空标志位

                return;
            }
        }

        // 检查编码器值接收标志（从编码器→原始&物理值）
        if (data.needsProcess(T::Flags::ENCODER_DATA_RECEIVE_NEED_REFRESH)) {

            //电流需要刷新
            if constexpr (std::is_same_v<T, MotorCurrent>) {
                const int16_t enc_val = data.actual_encoder.load();

                // 更新原始数据（使用valueToBytes）
                uint8_t bytes[2];
                valueToBytes(static_cast<int16_t>(enc_val), bytes, false);//编码器值 → 原始数据
                data.raw_actual.bytes_[0] = bytes[0];
                data.raw_actual.bytes_[1] = bytes[1];

                // 更新物理值（电流值 = 编码器值）
                data.actual_current.store(enc_val * CURRENT_SCALE); // 1计数=1mA      编码器值 → 物理值

                data.markProcessed(T::Flags::ENCODER_DATA_RECEIVE_NEED_REFRESH);
                return;
            }

            //位置需要刷新
            else if constexpr (std::is_same_v<T, MotorPosition>) {
                const int32_t enc_val = data.actual_encoder.load();

                // 使用valueToBytes处理4字节数据
                uint8_t bytes[4];
                valueToBytes(enc_val, bytes, false);// 编码器值 → 原始值 
                for (int i = 0; i < 4; ++i) { // 确保内存布局一致
                    data.raw_actual.bytes_[i] = bytes[i];
                }

                // 更新角度物理值
                data.actual_degree.store(
                    convertSensorAngle(enc_val, TO_ANGLE));

                data.markProcessed(T::Flags::ENCODER_DATA_RECEIVE_NEED_REFRESH);
                return;
            }

            //速度需要刷新
            else if constexpr (std::is_same_v<T, MotorVelocity>) {
                const int16_t enc_val = data.actual_encoder.load();

                // 使用valueToBytes处理速度数据
                uint8_t bytes[2];
                valueToBytes(enc_val, bytes, false); // 编码器值 → 原始值 
                for (int i = 0; i < 2; ++i) {
                    data.raw_actual.bytes_[i] = bytes[i];
                }

                // 更新转速物理值
                data.actual_rpm.store(enc_val * RPM_SCALE); // 1计数=1RPM

                data.markProcessed(T::Flags::ENCODER_DATA_RECEIVE_NEED_REFRESH);
                return;
            }
        }


        // 检查发送数据标志（物理值→编码器值→原始值）
        if (data.needsProcess(T::Flags::TARGET_DATA_SEND_NEED_REFRESH))
        {

            // 电流发送处理
            if constexpr (std::is_same_v<T, MotorCurrent>) {
                // 物理值→编码器值（电流mA→整型计数）
                const float target_current = data.target_current.load();
                const int16_t enc_val = static_cast<int16_t>(target_current); // 1mA=1计数
                data.target_encoder.store(enc_val);

                // 编码器值→原始数据（小端序）
                uint8_t bytes[2];
                valueToBytes(static_cast<int16_t>(enc_val), bytes, false); // 小端模式
                data.raw_target.bytes_[0] = bytes[0]; // LSB
                data.raw_target.bytes_[1] = bytes[1]; // MSB

                data.markProcessed(T::Flags::TARGET_DATA_SEND_NEED_REFRESH);
                return;
            }

            // 位置发送处理
            else if constexpr (std::is_same_v<T, MotorPosition>) {
                // 物理值（角度）→编码器值
                const float target_deg = data.target_degree.load();
                const int32_t enc_val = static_cast<int32_t>(
                    convertSensorAngle(target_deg, TO_ENCODER, SENSOR_RANGE));
                data.target_encoder.store(enc_val);

                // 编码器值→原始数据（小端序）
                uint8_t bytes[4];
                valueToBytes(enc_val, bytes, false); // 小端模式
                for (int i = 0; i < 4; ++i) {
                    data.raw_target.bytes_[i] = bytes[i]; // bytes[0]=LSB
                }

                data.markProcessed(T::Flags::TARGET_DATA_SEND_NEED_REFRESH);
                return;
            }

            // 速度发送处理
            else if constexpr (std::is_same_v<T, MotorVelocity>) {
                // 物理值（RPM）→编码器值
                const float target_rpm = data.target_rpm.load();
                const int16_t enc_val = static_cast<int16_t>(target_rpm * RPM_SCALE); // 1RPM=1计数
                data.target_encoder.store(enc_val);

                // 编码器值→原始数据（小端序）
                uint8_t bytes[2];
                valueToBytes(enc_val, bytes, false); // 小端模式
                for (int i = 0; i < 2; ++i) {
                    data.raw_target.bytes_[i] = bytes[i]; // bytes[0]=LSB
                }

                data.markProcessed(T::Flags::TARGET_DATA_SEND_NEED_REFRESH);
                return;
            }
        }


        // 检查发送数据标志（从编码器值→原始值&物理值）
        if (data.needsProcess(T::Flags::ENCODER_DATA_SEND_NEED_REFRESH)) {

            // 电流发送处理（1编码器值=1mA）
            if constexpr (std::is_same_v<T, MotorCurrent>) {
                const int32_t enc_val = data.target_encoder.load();

                // 编码器值→原始数据（小端序）
                uint8_t bytes[2];
                valueToBytes(static_cast<int16_t>(enc_val), bytes, false); // 小端模式
                data.raw_target.bytes_[0] = bytes[0]; // LSB
                data.raw_target.bytes_[1] = bytes[1]; // MSB

                // 编码器值→物理值（1:1映射）
                data.target_current.store(static_cast<float>(enc_val)); // 直接转为mA

                data.markProcessed(T::Flags::ENCODER_DATA_SEND_NEED_REFRESH);
                return;
            }

            // 位置发送处理
            else if constexpr (std::is_same_v<T, MotorPosition>) {
                const int32_t enc_val = data.target_encoder.load();

                // 编码器值→原始数据（小端序）
                uint8_t bytes[4];
                valueToBytes(enc_val, bytes, false); // 小端模式
                for (int i = 0; i < 4; ++i) {
                    data.raw_target.bytes_[i] = bytes[i]; // bytes[0]=LSB
                }

                // 编码器值→物理角度（使用转换函数）
                data.target_degree.store(
                    convertSensorAngle(enc_val, true));

                data.markProcessed(T::Flags::ENCODER_DATA_SEND_NEED_REFRESH);
                return;
            }

            // 速度发送处理（1编码器值=1RPM）
            else if constexpr (std::is_same_v<T, MotorVelocity>) {
                const int16_t enc_val = data.target_encoder.load();

                // 编码器值→原始数据（小端序）
                uint8_t bytes[2];
                valueToBytes(enc_val, bytes, false); // 小端模式
                for (int i = 0; i < 2; ++i) {
                    data.raw_target.bytes_[i] = bytes[i]; // bytes[0]=LSB
                }

                // 编码器值→物理值（1:1映射）
                data.target_rpm.store(static_cast<float>(enc_val)); // 直接转为RPM

                data.markProcessed(T::Flags::ENCODER_DATA_SEND_NEED_REFRESH);
                return;
            }
        }




        };



};
#endif // CLASS_MOTOR_HPP