#ifndef CLASS_MOTOR_HPP
#define CLASS_MOTOR_HPP

#include <cstdint>
#include <array>
#include <string>
#include <cmath>
#include <mutex>
#include <atomic>



/*
**
* @brief 硬件对齐的原始数据存储结构体模板
* @tparam N 数据位宽（仅允许2 / 4 / 8字节）
*
*@warning 每个实例必须单独占用完整缓存行，不得连续声明未填充的结构体数组
*/
template <size_t N>
struct AlignedRawData {
    // 静态断言确保只支持关键位宽
    static_assert(N == 2 || N == 4 || N == 8,
        "Only support 2/4/8 bytes width");
    // 匿名联合体实现类型双关（Type Punning）
    union {
        volatile uint8_t bytes_[N];  ///< 原始字节视图（适用于DMA/协议栈）
        std::conditional_t<N == 2, int16_t,
            std::conditional_t<N == 4, int32_t,
            int64_t>> value_;        ///< 有符号整型视图（业务逻辑使用）
    };
    /// 显式填充保证独占完整缓存行（ARM64为64字节）
    uint8_t padding_[64 - N];
    // ========================= 原子操作接口 ========================= //
    /**
     * @brief 原子写入数据（Release语义）
     * @tparam T 数据类型（自动推导位宽）
     * @param buf 输入数据指针
     *
     * @note 典型场景:
     * - CAN报文接收线程调用
     * - DMA传输完成中断中调用
     *
     * @code
     * uint16_t val = 0x1234;
     * can_data.atomicWrite(&val); // 2字节写入
     * @endcode
     */
    template <typename T>
    void atomicWrite(const T* buf) volatile noexcept {
        static_assert(sizeof(T) == N, "Type size mismatch");
        __atomic_store_n(&value_,
            *reinterpret_cast<const decltype(value_)*>(buf),
            __ATOMIC_RELEASE);
    }
    /**
     * @brief 原子读取数据（Acquire语义）
     * @tparam T 数据类型（必须与实例位宽匹配）
     * @param[out] buf 输出缓冲区指针
     *
     * @note 内存序保证:
     * - 确保读取前所有先前的写入操作对当前线程可见
     * - 适合用于控制循环中的状态读取
     */
    template <typename T>
    void atomicRead(T* buf) const volatile noexcept {
        static_assert(sizeof(T) == N, "Type size mismatch");
        *reinterpret_cast<decltype(value_)*>(buf) =
            __atomic_load_n(&value_, __ATOMIC_ACQUIRE);
    }
};
















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
        uint8_t controlWordRaw[2];     /// 控制字（原始2字节）>>
        uint8_t statusWordRaw[2];      /// 状态字（原始2字节）<<
    };
    volatile struct {
        uint8_t modeOfOperationRaw[1]; /// 运行模式（原始1字节）>>
        uint8_t errorCodeRaw[2];       /// 电机错误代码（原始2字节）<<
    };

    const uint16_t controlWordIndex = OD_CONTROL_WORD;      // 0x6040
    const uint16_t statusWordIndex = OD_STATUS_WORD;       // 0x6041
    const uint16_t modeOfOperationIndex = OD_MODES_OF_OPERATION;// 0x6060
    const uint16_t errorCodeIndex = OD_ERROR_CODE;        // 0x603F
};


/**
 * @brief 电机电流 (MotorCurrent)
 * @brief 采用原子操作和缓存行对齐优化，确保多线程安全访问
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
 * @brief MotorCurrent::raw_actual.value      整型形式访问(int16_t)
 *
 * @brief MotorCurrent::raw_target            目标电流原始数据区(带填充对齐)
 * @brief MotorCurrent::raw_target.bytes      原始字节形式访问(volatile uint8_t[2])
 * @brief MotorCurrent::raw_target.value      整型形式访问(int16_t)
 *
 * @brief MotorCurrent::actual_encoder        实际电流编码器计数值(原子int32_t)  // 修正实际值组注释为独立成员
 * @brief MotorCurrent::actual_current        实际物理电流值(原子float)
 * @brief MotorCurrent::target_encoder        目标电流编码器计数值(原子int32_t)  // 修正目标值组注释为独立成员
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
        ACTUAL_DATA_SEND_NEED_REFRESH = 0x20,  ///< 位5: 实际接收数据（十进制）需要刷新

        // 保留位
        RESERVED_6 = 0x40,  ///< 位6: 保留 用于扩展
        RESERVED_7 = 0x80   ///< 位7: 保留 用于扩展

        
    };
    alignas(64) std::atomic<uint8_t> flags_{ 0 };

    // 原始数据区（带缓存行填充）

    AlignedRawData<2> raw_actual;///< 实际电流原始数据
    AlignedRawData<2> raw_target;///< 实际电流原始数据

    // 转换值组（独立缓存行）
    /** @brief 实际电流转换结果组 */
        alignas(64) std::atomic<int32_t> actual_encoder{ 0 };  ///< 编码器计数表示值
        alignas(64) std::atomic<float> actual_current{ 0.0f }; ///< 物理电流值(安培)

    /** @brief 目标电流转换结果组 */
        alignas(64) std::atomic<int32_t> target_encoder{ 0 };  ///< 编码器计数表示值
        alignas(64) std::atomic<float> target_current{ 0.0f }; ///< 物理电流值(安培)
    

    // 协议常量（请保持硬编码）
    const uint16_t actual_Current_Index = OD_ACTUAL_CURRENT;  ///< 实际电流对象字典索引
    const uint16_t target_Current_Index = OD_TARGET_CURRENT;  ///< 目标电流对象字典索引

    /**
     * @brief 检查待处理的数据类型
     * @param f 标志位掩码
     * @return 是否存在待处理更新
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
 * @brief MotorPosition::Flags::DEGREE_DATA_SEND_NEED_REFRESH     角度值发送数据（浮点）需要刷新
 * @brief MotorPosition::Flags::DEGREE_DATA_RECEIVE_NEED_REFRESH  角度值接收数据（浮点）需要刷新
 *
 * @brief MotorPosition::raw_actual            实际位置原始数据区(带填充对齐)
 * @brief MotorPosition::raw_actual.bytes      原始字节形式访问(volatile uint8_t[4])
 * @brief MotorPosition::raw_actual.value      整型形式访问(int32_t)
 *
 * @brief MotorPosition::raw_target            目标位置原始数据区(带填充对齐)
 * @brief MotorPosition::raw_target.bytes      原始字节形式访问(volatile uint8_t[4])
 * @brief MotorPosition::raw_target.value      整型形式访问(int32_t)
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

        DEGREE_DATA_SEND_NEED_REFRESH = 0x10,  ///< 位4: 角度值发送数据（浮点）需要刷新
        DEGREE_DATA_RECEIVE_NEED_REFRESH = 0x20,  ///< 位5: 角度值接收数据（浮点）需要刷新

        RESERVED_6 = 0x40,  ///< 位6: 保留 用于扩展
        RESERVED_7 = 0x80   ///< 位7: 保留 用于扩展
    };
    alignas(64) std::atomic<uint8_t> flags_{ 0 };

    // 原始数据区（带缓存行填充）
    AlignedRawData<4> raw_actual;  ///< 实际位置原始数据
    AlignedRawData<4> raw_target;  ///< 目标位置原始数据

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
     * @return 是否存在待处理更新
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
 * @brief 采用原子操作和缓存行对齐优化，确保多线程安全访问
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
 * @brief MotorVelocity::Flags::RPM_DATA_SEND_NEED_REFRESH        RPM值发送数据（浮点）需要刷新
 * @brief MotorVelocity::Flags::RPM_DATA_RECEIVE_NEED_REFRESH     RPM值接收数据（浮点）需要刷新
 *
 * @brief MotorVelocity::raw_actual            实际速度原始数据区(带填充对齐)
 * @brief MotorVelocity::raw_actual.bytes      原始字节形式访问(volatile uint8_t[4])
 * @brief MotorVelocity::raw_actual.value      整型形式访问(int32_t)
 *
 * @brief MotorVelocity::raw_target            目标速度原始数据区(带填充对齐)
 * @brief MotorVelocity::raw_target.bytes      原始字节形式访问(volatile uint8_t[4])
 * @brief MotorVelocity::raw_target.value      整型形式访问(int32_t)
 *
 * @brief MotorVelocity::actual_encoder        实际速度编码器计数值(原子int32_t)
 * @brief MotorVelocity::actual_rpm            实际转速值(原子float)
 * @brief MotorVelocity::target_encoder        目标速度编码器计数值(原子int32_t)
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

        RPM_DATA_SEND_NEED_REFRESH = 0x10,  ///< 位4: RPM值发送数据（浮点）需要刷新
        RPM_DATA_RECEIVE_NEED_REFRESH = 0x20,  ///< 位5: RPM值接收数据（浮点）需要刷新

        RESERVED_6 = 0x40,  ///< 位6: 保留 用于扩展
        RESERVED_7 = 0x80   ///< 位7: 保留 用于扩展
    };
    alignas(64) std::atomic<uint8_t> flags_{ 0 };

    // 原始数据区（带缓存行填充）
    AlignedRawData<4> raw_actual;  ///< 实际速度原始数据
    AlignedRawData<4> raw_target;  ///< 目标速度原始数据

    // 转换值组（独立缓存行）
    alignas(64) std::atomic<int32_t> actual_encoder{ 0 };  ///< 实际编码器计数值  
    alignas(64) std::atomic<float> actual_rpm{ 0.0f };     ///< 实际转速值(RPM)
    alignas(64) std::atomic<int32_t> target_encoder{ 0 };  ///< 目标编码器计数值
    alignas(64) std::atomic<float> target_rpm{ 0.0f };     ///< 目标转速值(RPM)

    // 协议常量（DS402标准）
    const uint16_t actual_Velocity_Index = OD_ACTUAL_VELOCITY;  ///< 0x606C
    const uint16_t target_Velocity_Index = OD_TARGET_VELOCITY;  ///< 0x60FF

    /**
     * @brief 检查待处理的数据类型
     * @param f 标志位掩码
     * @return 是否存在待处理更新
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
 * 1. 直接使用AlignedRawData<4>存储原始值
 * 2. 数值直接映射为工程值(单位：转/秒²)
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
    AlignedRawData<4> raw_accel;  ///< 加速度值(单位r/s²)
    AlignedRawData<4> raw_decel;  ///< 减速度值(单位r/s²)

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
public:
    Motor() = default;
    ~Motor() = default;

    /**
     * @brief 初始化方法：
     *  - 暂时不实现，可在此将所有原始数据清零，或将模式置为安全模式等。
     */
    void init()
    {
        // 此处根据需求对所有子结构的数据做一次安全初始化
        // 例如:
        for (auto& b : stateAndMode.controlWordRaw) { b = 0; }
        for (auto& b : stateAndMode.statusWordRaw) { b = 0; }
        for (auto& b : stateAndMode.modeOfOperationRaw) { b = 0; }
        for (auto& b : stateAndMode.errorCodeRaw) { b = 0; }

        for (auto& b : current.actual_CurrentRaw) { b = 0; }
        for (auto& b : current.target_CurrentRaw) { b = 0; }
        current.actual_Current = 0.0f;
        current.target_Current = 0.0f;

        for (auto& b : position.actualPositionRaw) { b = 0; }
        for (auto& b : position.targetPositionRaw) { b = 0; }
        position.actual_PositionDeg = 0.0f;
        position.target_PositionDeg = 0.0f;
        position.actual_PositionCnt = 0;
        position.target_PositionCnt = 0;

        for (auto& b : velocity.actualVelocityRaw) { b = 0; }
        for (auto& b : velocity.targetVelocityRaw) { b = 0; }
        velocity.actualVelocityRPM = 0.0f;
        velocity.targetVelocityRPM = 0.0f;
        velocity.actualVelocityCnt = 0;
        velocity.targetVelocityCnt = 0;

        for (auto& b : accelDecel.accelRaw) { b = 0; }
        for (auto& b : accelDecel.decelRaw) { b = 0; }
        accelDecel.accelValue = 0;
        accelDecel.decelValue = 0;

        // ...
    }

    /**
     * @brief 读刷新方法
     *
     * 一般在“接收线程”对 PDO/SDO 收到的原始数据 (uint8_t[]) 做映射后，
     * 写入到本地 Motor 类对应字段。然后这里可以进行额外的数值换算，比如把 Raw 转换成实际值。
     */
    void readRefresh()
    {
        std::lock_guard<std::mutex> lock(mtx_);

        // (示例) 将 raw 的控制字/状态字/模式等转换成需要的 int 或枚举：
        // controlWord = (uint16_t)( (stateAndMode.controlWordRaw[1] << 8) |
        //                            stateAndMode.controlWordRaw[0] );
        // ...
        // modeOfOperation = (MotorMode)(stateAndMode.modeOfOperationRaw[0]);

        // 电流
        {
            // 低字节在 [0], 高字节在 [1]，这只是举例；若是小端就要相反。
            int16_t actualI = static_cast<int16_t>(
                (current.actual_CurrentRaw[1] << 8) | current.actual_CurrentRaw[0]);
            int16_t targetI = static_cast<int16_t>(
                (current.target_CurrentRaw[1] << 8) | current.target_CurrentRaw[0]);

            // 转换为物理量(仅示例，如驱动文档规定 1个计数=1mA)
            current.actual_Current = static_cast<float>(actualI);
            current.target_Current = static_cast<float>(targetI);
        }

        // 位置
        {
            // 假设 raw[0] 为最低字节
            int32_t actualPos = ((int32_t)position.actualPositionRaw[3] << 24) |
                ((int32_t)position.actualPositionRaw[2] << 16) |
                ((int32_t)position.actualPositionRaw[1] << 8) |
                ((int32_t)position.actualPositionRaw[0]);
            int32_t targetPos = ((int32_t)position.targetPositionRaw[3] << 24) |
                ((int32_t)position.targetPositionRaw[2] << 16) |
                ((int32_t)position.targetPositionRaw[1] << 8) |
                ((int32_t)position.targetPositionRaw[0]);

            position.actual_PositionCnt = actualPos;
            position.target_PositionCnt = targetPos;
            // 转成角度(仅示例：假设 65536 对应 360 度)
            position.actual_PositionDeg = actualPos * (360.0f / 65536.0f);
            position.target_PositionDeg = targetPos * (360.0f / 65536.0f);
        }

        // 速度 (同理)
        {
            int32_t actualVel = ((int32_t)velocity.actualVelocityRaw[3] << 24) |
                ((int32_t)velocity.actualVelocityRaw[2] << 16) |
                ((int32_t)velocity.actualVelocityRaw[1] << 8) |
                ((int32_t)velocity.actualVelocityRaw[0]);
            int32_t targetVel = ((int32_t)velocity.targetVelocityRaw[3] << 24) |
                ((int32_t)velocity.targetVelocityRaw[2] << 16) |
                ((int32_t)velocity.targetVelocityRaw[1] << 8) |
                ((int32_t)velocity.targetVelocityRaw[0]);

            velocity.actualVelocityCnt = actualVel;
            velocity.targetVelocityCnt = targetVel;
            // 仅示例：假设 1计数=1rpm
            velocity.actualVelocityRPM = static_cast<float>(actualVel);
            velocity.targetVelocityRPM = static_cast<float>(targetVel);
        }

        // 加减速度
        {
            uint32_t accel = ((uint32_t)accelDecel.accelRaw[3] << 24) |
                ((uint32_t)accelDecel.accelRaw[2] << 16) |
                ((uint32_t)accelDecel.accelRaw[1] << 8) |
                ((uint32_t)accelDecel.accelRaw[0]);
            uint32_t decel = ((uint32_t)accelDecel.decelRaw[3] << 24) |
                ((uint32_t)accelDecel.decelRaw[2] << 16) |
                ((uint32_t)accelDecel.decelRaw[1] << 8) |
                ((uint32_t)accelDecel.decelRaw[0]);

            accelDecel.accelValue = accel;
            accelDecel.decelValue = decel;
        }
    }

    /**
     * @brief 写刷新方法
     *
     * - 典型用法是在定时器线程中，根据 Motor 对象中 targetXXX 的值组装要发送的 SDO/PDO 数据
     * - 并写入到 motorXXRaw[] 中，然后调用实际的发送函数
     */
    void writeRefresh()
    {
        std::lock_guard<std::mutex> lock(mtx_);

        // 例如：将 targetPositionDeg 转回 raw[0..3]
        // 并由上位机的发送线程实际发送 SDO/PDO
        // 这里只是演示
        int32_t tarPos = position.target_PositionCnt;
        position.targetPositionRaw[0] = (uint8_t)(tarPos & 0xFF);
        position.targetPositionRaw[1] = (uint8_t)((tarPos >> 8) & 0xFF);
        position.targetPositionRaw[2] = (uint8_t)((tarPos >> 16) & 0xFF);
        position.targetPositionRaw[3] = (uint8_t)((tarPos >> 24) & 0xFF);

        // 同理 targetVelocity, targetCurrent, accel, decel 等都可以写回
        // ...
    }

public:


    StateAndMode    stateAndMode;  //状态和模式结构体
    MotorCurrent    current;       //电流结构体
    MotorPosition   position;      //位置结构体
    MotorVelocity   velocity;      //速度结构体
    MotorAccelDecel accelDecel;    //

private:
    // 用于 readRefresh / writeRefresh 保护的互斥量
    std::mutex mtx_;
};

#endif // CLASS_MOTOR_HPP