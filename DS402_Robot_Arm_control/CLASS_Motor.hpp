#ifndef CLASS_MOTOR_HPP
#define CLASS_MOTOR_HPP

#include <cstdint>
#include <array>
#include <string>
#include <cmath>
#include <mutex>

/**
 * @brief DS402 协议下常用对象字典地址示例（仅供参考）
 * 一般是 0xXXXX:subIndex, 可以在这里统一声明。
 */
static constexpr uint16_t OD_CONTROL_WORD = 0x6040;  /// 控制字>>> 0x6040
static constexpr uint16_t OD_STATUS_WORD = 0x6041;  /// 状态字<<< 0x6041
static constexpr uint16_t OD_MODES_OF_OPERATION = 0x6060;  /// 运行模式(写)>>> 0x6060
static constexpr uint16_t OD_MODES_OF_DISPLAY = 0x6061;  /// 运行模式(读)<<< 0x6061
static constexpr uint16_t OD_ERROR_CODE = 0x603F;  /// 错误码<<< 0x603F

static constexpr uint16_t OD_TARGET_CURRENT = 0x6071;  /// 目标电流>>> 0x6071
static constexpr uint16_t OD_ACTUAL_CURRENT = 0x6078;  /// 实际电流<<< 0x6078
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
 * @brief 状态和模式结构体
 *
 * - 这里仅示例添加了控制字、状态字、运行模式、错误码等
 * - 由于用户要求：即使逻辑上是 S16/U16，也用 uint8_t 数组保存原始值
 * - “数据表”等可以根据需要扩展为 std::array 或 std::vector
 */
struct StateAndMode
{
    // DS402: 控制字(0x6040)、状态字(0x6041)通常各2字节
    // 运行模式(0x6060)通常1字节，错误码(0x603F)通常2字节
    // 以下只是示例，把原始字节都放在这里
    volatile uint8_t controlWordRaw[2];       /// 控制字（原始2字节）>>
    volatile uint8_t statusWordRaw[2];        /// 状态字（原始2字节）<<
    volatile uint8_t modeOfOperationRaw[1];   /// 运行模式（原始1字节）>>
    volatile uint8_t errorCodeRaw[2];         /// 电机错误代码（原始2字节）<<

    // 
    const uint16_t controlWordIndex = OD_CONTROL_WORD;      // 0x6040
    const uint16_t statusWordIndex = OD_STATUS_WORD;       // 0x6041
    const uint16_t modeOfOperationIndex = OD_MODES_OF_OPERATION;// 0x6060
    const uint16_t errorCodeIndex = OD_ERROR_CODE;        // 0x603F
};

/**
 * @brief 电流结构体
 *
 * - 实际通常是 S16 / U16，但这里按要求使用 uint8_t 数组存储原始值
 * - 同理“值的类型”就仅在注释中说明
 */
struct MotorCurrent
{
    // 原始实际电流；例如 S16(2字节)
    volatile uint8_t actualCurrentRaw[2];
    // 原始目标电流；例如 S16(2字节)
    volatile uint8_t targetCurrentRaw[2];

    // 经过换算得到的“编码器值”或“实际数值”等，可在 readRefresh() 中更新
    //实际电流（换算后）<<
    float actualCurrent = 0.0f;
    //目标电流（换算后）>>
    float targetCurrent = 0.0f;

    // 实际电流字典 0x6078
    const uint16_t actualCurrentIndex = OD_ACTUAL_CURRENT; 
    // 目标电流字典 0x6071
    const uint16_t targetCurrentIndex = OD_TARGET_CURRENT; 
};

/**
 * @brief 位置结构体
 *
 * - 位置通常是 S32，因此这里以 4字节 原始数据表示
 */
struct MotorPosition
{
    volatile uint8_t actualPositionRaw[4];
    volatile uint8_t targetPositionRaw[4];

    float actualPositionDeg = 0.0f;  /// 实际值(角度)>>
    float targetPositionDeg = 0.0f;  /// 目标值(角度)<<

    // 编码器值（十进制），仅示例
    int32_t actualPositionCnt = 0;
    int32_t targetPositionCnt = 0;

    // 对应对象字典地址
    const uint16_t actualPositionIndex = OD_ACTUAL_POSITION; // 0x6064
    const uint16_t targetPositionIndex = OD_TARGET_POSITION; // 0x607A
};

/**
 * @brief 速度结构体
 *
 * - 速度也经常是 S32，但不少驱动仅用 S16 表示。这里示例以 4字节存储
 */
struct MotorVelocity
{
    volatile uint8_t actualVelocityRaw[4];//实际的
    volatile uint8_t targetVelocityRaw[4];

    float actualVelocityRPM = 0.0f; ///< 实际速度 (rpm)
    float targetVelocityRPM = 0.0f; ///< 目标速度 (rpm)

    int32_t actualVelocityCnt = 0;    ///< 编码器计数值
    int32_t targetVelocityCnt = 0;

    // 对应对象字典地址
    const uint16_t actualVelocityIndex = OD_ACTUAL_VELOCITY; //0x606C
    const uint16_t targetVelocityIndex = OD_TARGET_VELOCITY; //0x60FF
};

/**
 * @brief 加减速度结构体
 *
 * - DS402 常见是 U32，用于配置加速度 / 减速度
 */
struct MotorAccelDecel
{
    volatile uint8_t accelRaw[4];
    volatile uint8_t decelRaw[4];

    uint32_t accelValue = 0;
    uint32_t decelValue = 0;

    // 对应对象字典地址
    const uint16_t accelIndex = OD_ACCELERATION;   //0x6083
    const uint16_t decelIndex = OD_DECELERATION;   //0x6084
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

        for (auto& b : current.actualCurrentRaw) { b = 0; }
        for (auto& b : current.targetCurrentRaw) { b = 0; }
        current.actualCurrent = 0.0f;
        current.targetCurrent = 0.0f;

        for (auto& b : position.actualPositionRaw) { b = 0; }
        for (auto& b : position.targetPositionRaw) { b = 0; }
        position.actualPositionDeg = 0.0f;
        position.targetPositionDeg = 0.0f;
        position.actualPositionCnt = 0;
        position.targetPositionCnt = 0;

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
                (current.actualCurrentRaw[1] << 8) | current.actualCurrentRaw[0]);
            int16_t targetI = static_cast<int16_t>(
                (current.targetCurrentRaw[1] << 8) | current.targetCurrentRaw[0]);

            // 转换为物理量(仅示例，如驱动文档规定 1个计数=1mA)
            current.actualCurrent = static_cast<float>(actualI);
            current.targetCurrent = static_cast<float>(targetI);
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

            position.actualPositionCnt = actualPos;
            position.targetPositionCnt = targetPos;
            // 转成角度(仅示例：假设 65536 对应 360 度)
            position.actualPositionDeg = actualPos * (360.0f / 65536.0f);
            position.targetPositionDeg = targetPos * (360.0f / 65536.0f);
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
        int32_t tarPos = position.targetPositionCnt;
        position.targetPositionRaw[0] = (uint8_t)(tarPos & 0xFF);
        position.targetPositionRaw[1] = (uint8_t)((tarPos >> 8) & 0xFF);
        position.targetPositionRaw[2] = (uint8_t)((tarPos >> 16) & 0xFF);
        position.targetPositionRaw[3] = (uint8_t)((tarPos >> 24) & 0xFF);

        // 同理 targetVelocity, targetCurrent, accel, decel 等都可以写回
        // ...
    }

public:
    // 下面是几个子结构，根据需求可以设为 public 或 private
    StateAndMode    stateAndMode;
    MotorCurrent    current;
    MotorPosition   position;
    MotorVelocity   velocity;
    MotorAccelDecel accelDecel;

private:
    // 用于 readRefresh / writeRefresh 保护的互斥量
    std::mutex mtx_;
};

#endif // CLASS_MOTOR_HPP