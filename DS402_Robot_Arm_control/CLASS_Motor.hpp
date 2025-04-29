#ifndef CLASS_MOTOR_HPP
#define CLASS_MOTOR_HPP

#include <cstdint>
#include <array>
#include <string>
#include <cmath>
#include <mutex>
#include <atomic>



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
        (sizeof(T) == N && std::is_trivially_copyable_v<T>),
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
     * 1. 使用__ATOMIC_RELEASE保证写入可见性
     * 2. 静态检查类型大小匹配
     * 3. 严格内存访问（避免strict-aliasing违规）
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
    void atomicWrite(const U* buf) volatile noexcept {
        static_assert(sizeof(U) == N, "Type size mismatch");
        static_assert(std::is_trivially_copyable_v<U>,
            "Type must be trivially copyable");
        __atomic_store_n(&value_,
            *static_cast<const decltype(value_)*>(static_cast<const void*>(buf)),
            __ATOMIC_RELEASE);
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
     * @warning 缓冲区必须预先分配
     *
     * @example 读取到int32_t变量：
     * @code
     * int32_t result;
     * data.atomicRead(&result);
     * @endcode
     */
    template <typename U>
    void atomicRead(U* buf) const volatile noexcept {
        static_assert(sizeof(U) == N, "Type size mismatch");
        *static_cast<decltype(value_)*>(static_cast<void*>(buf)) =
            __atomic_load_n(&value_, __ATOMIC_ACQUIRE);
    }

    /* 禁用拷贝构造/赋值（保证操作原子性） */
    AlignedRawData(const AlignedRawData&) = delete;
    AlignedRawData& operator=(const AlignedRawData&) = delete;
};

// 编译时校验
static_assert(sizeof(AlignedRawData<4>) == 64,
    "Cache line size must be 64 bytes");
static_assert(alignof(AlignedRawData<4>) >= 64,
    "Minimum alignment requirement failed");









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

    AlignedRawData<2,int16_t> raw_actual;///< 实际电流原始数据
    AlignedRawData<2,int16_t> raw_target;///< 实际电流原始数据

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
    AlignedRawData<2,int16_t> raw_actual;  ///< 实际速度原始数据
    AlignedRawData<4,int16_t> raw_target;  ///< 目标速度原始数据

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
public:
    Motor() = default;
    ~Motor() = default;

    /**
     * @brief 初始化方法：
     *  
     */
    void init() {
        //  状态模式初始化
        stateAndMode.refresh = false;
        memset(stateAndMode.controlWordRaw, 0, sizeof(stateAndMode.controlWordRaw));
        memset(stateAndMode.statusWordRaw, 0, sizeof(stateAndMode.statusWordRaw));
        stateAndMode.modeOfOperationRaw[0] =
            static_cast<uint8_t>(MotorMode::PROFILE_TORQUE); // 默认选择电流模式，在电流为0的情况下是安全的

        // 电流数据初始化
        current.flags_.store(0);
        current.raw_actual.atomicWrite<uint16_t>(0);
        current.raw_target.atomicWrite<uint16_t>(0);
        current.actual_current.store(0.0f);
        current.target_current.store(0.0f);

        // 位置数据初始化
        position.flags_.store(0);
        position.raw_actual.atomicWrite<int32_t>(0);
        position.raw_target.atomicWrite<int32_t>(0);
        position.actual_degree.store(0.0f);

        // 速度数据初始化
        velocity.flags_.store(0);
        velocity.raw_target.atomicWrite<int32_t>(0);

        // 加减速初始化
        const uint32_t default_accel = 0; // RPM/min
        const uint32_t default_decel = 0;
        accelDecel.raw_accel.atomicWrite<uint32_t>(&default_accel);
        accelDecel.raw_decel.atomicWrite<uint32_t>(&default_decel);
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
    MotorAccelDecel accelDecel;    //加速度结构体

private:
    // 用于 readRefresh / writeRefresh 保护的互斥量
    std::mutex mtx_;
};

#endif // CLASS_MOTOR_HPP