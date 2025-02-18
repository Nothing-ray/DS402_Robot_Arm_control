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
    std::atomic<bool> refresh = true;

    // DS402: 控制字(0x6040)、状态字(0x6041)通常各2字节
    // 运行模式(0x6060)通常1字节，错误码(0x603F)通常2字节
    volatile uint8_t controlWordRaw[2];       /// 控制字（原始2字节）>>
    volatile uint8_t statusWordRaw[2];        /// 状态字（原始2字节）<<
    volatile uint8_t modeOfOperationRaw[1];   /// 运行模式（原始1字节）>>
    volatile uint8_t errorCodeRaw[2];         /// 电机错误代码（原始2字节）<<

    const uint16_t controlWordIndex = OD_CONTROL_WORD;      // 0x6040
    const uint16_t statusWordIndex = OD_STATUS_WORD;       // 0x6041
    const uint16_t modeOfOperationIndex = OD_MODES_OF_OPERATION;// 0x6060
    const uint16_t errorCodeIndex = OD_ERROR_CODE;        // 0x603F
};

/**
 * @brief 电流结构体 (MotorCurrent)
 * @brief MotorCurrent::send_refresh    发送数据刷新 (true:已刷新 false:未刷新)
 * @brief MotorCurrent::receive_refresh 接收数据刷新 (true:已刷新 false:未刷新)
 * @brief MotorCurrent::actual_CurrentRaw  实际电流原始2字节 (S16)
 * @brief MotorCurrent::target_CurrentRaw  目标电流原始2字节 (S16)
 * @brief MotorCurrent::actual_Current_Encoder  实际电流换算后编码器值
 * @brief MotorCurrent::target_Current_Encoder  目标电流换算后编码器值
 * @brief MotorCurrent::actual_Current     实际电流(浮点)
 * @brief MotorCurrent::target_Current     目标电流(浮点)
 */
struct MotorCurrent
{
    // true 已刷新 false 未刷新 更改完变量以后记得刷新
    std::atomic<bool> send_refresh = true;    //发送数据的间接变量的刷新
    std::atomic<bool> receive_refresh = true; //接收数据的间接变量的刷新 

    // 原始实际电流；S16(2字节)<<
    volatile uint8_t actual_CurrentRaw[2];
    // 原始目标电流；S16(2字节)>>
    volatile uint8_t target_CurrentRaw[2];

    // 原始实际电流经过换算后的编码器的值 刷新
    int32_t actual_Current_Encoder = 0;
    // 原始目标电流经过换算后的编码器的值 刷新
    int32_t target_Current_Encoder = 0;

    // 实际电流（换算后）  刷新
    float actual_Current = 0.0f;
    // 目标电流（换算后）  刷新
    float target_Current = 0.0f;

    const uint16_t actual_Current_Index = OD_ACTUAL_CURRENT;
    const uint16_t target_Current_Index = OD_TARGET_CURRENT;
};

/**
 * @brief 位置结构体 (MotorPosition)
 * @brief MotorPosition::send_refresh    发送数据刷新 (true:已刷新 false:未刷新)
 * @brief MotorPosition::receive_refresh 接收数据刷新 (true:已刷新 false:未刷新)
 * @brief MotorPosition::actualPositionRaw  实际位置原始4字节 (S32)
 * @brief MotorPosition::targetPositionRaw  目标位置原始4字节 (S32)
 * @brief MotorPosition::actual_PositionDeg  实际位置换算后角度
 * @brief MotorPosition::target_PositionDeg  目标位置换算后角度
 * @brief MotorPosition::actual_PositionCnt  实际位置编码器计数
 * @brief MotorPosition::target_PositionCnt  目标位置编码器计数
 */
struct MotorPosition
{
    // true 已刷新 false 未刷新 更改完变量以后记得刷新
    std::atomic<bool> send_refresh = true;    //发送数据的间接变量的刷新
    std::atomic<bool> receive_refresh = true; //接收数据的间接变量的刷新

    volatile uint8_t actualPositionRaw[4]; //实际位置的原始数据>> 
    volatile uint8_t targetPositionRaw[4]; //目标位置的原始数据<< 

    float actual_PositionDeg = 0.0f;  /// 实际值(角度)
    float target_PositionDeg = 0.0f;  /// 目标值(角度)

    int32_t actual_PositionCnt = 0;  //实际位置的编码器值
    int32_t target_PositionCnt = 0;

    const uint16_t actual_Position_Index = OD_ACTUAL_POSITION; // 0x6064
    const uint16_t target_Position_Index = OD_TARGET_POSITION; // 0x607A
};

/**
 * @brief 速度结构体 (MotorVelocity)
 * @brief MotorVelocity::send_refresh    发送数据刷新 (true:已刷新 false:未刷新)
 * @brief MotorVelocity::receive_refresh 接收数据刷新 (true:已刷新 false:未刷新)
 * @brief MotorVelocity::actualVelocityRaw  实际速度原始4字节 (S32)
 * @brief MotorVelocity::targetVelocityRaw  目标速度原始4字节 (S32)
 * @brief MotorVelocity::actualVelocityRPM   实际速度(rpm)
 * @brief MotorVelocity::targetVelocityRPM   目标速度(rpm)
 * @brief MotorVelocity::actualVelocityCnt   实际速度的编码器值
 * @brief MotorVelocity::targetVelocityCnt   目标速度的编码器值
 */
struct MotorVelocity
{
    // true 已刷新 false 未刷新 更改完变量以后记得刷新
    std::atomic<bool> send_refresh = true;    //发送数据的间接变量的刷新
    std::atomic<bool> receive_refresh = true; //接收数据的间接变量的刷新

    volatile uint8_t actualVelocityRaw[4];//实际的
    volatile uint8_t targetVelocityRaw[4];

    float actualVelocityRPM = 0.0f; ///< 实际速度 (rpm)
    float targetVelocityRPM = 0.0f; ///< 目标速度 (rpm)

    int32_t actualVelocityCnt = 0;    // 编码器实际速度值
    int32_t targetVelocityCnt = 0;    // 编码器目标速度值

    const uint16_t actualVelocityIndex = OD_ACTUAL_VELOCITY; //0x606C
    const uint16_t targetVelocityIndex = OD_TARGET_VELOCITY; //0x60FF
};

/**
 * @brief 加减速度结构体 (MotorAccelDecel)
 * @brief MotorAccelDecel::accelRaw  加速度原始4字节 (U32)
 * @brief MotorAccelDecel::decelRaw  减速度原始4字节 (U32)
 * @brief MotorAccelDecel::accelValue  解析后的加速度数值
 * @brief MotorAccelDecel::decelValue  解析后的减速度数值
 */
struct MotorAccelDecel
{
    volatile uint8_t accelRaw[4];
    volatile uint8_t decelRaw[4];

    uint32_t accelValue = 0;
    uint32_t decelValue = 0;

    const uint16_t accelIndex = OD_ACCELERATION; //0x6083
    const uint16_t decelIndex = OD_DECELERATION; //0x6084
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