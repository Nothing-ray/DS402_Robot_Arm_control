#ifndef CLASS_MOTOR_HPP
#define CLASS_MOTOR_HPP

#include <cstdint>
#include <array>
#include <string>
#include <cmath>
#include <mutex>

/**
 * @brief DS402 Э���³��ö����ֵ��ַʾ���������ο���
 * һ���� 0xXXXX:subIndex, ����������ͳһ������
 */
static constexpr uint16_t OD_CONTROL_WORD = 0x6040;  /// ������>>> 0x6040
static constexpr uint16_t OD_STATUS_WORD = 0x6041;  /// ״̬��<<< 0x6041
static constexpr uint16_t OD_MODES_OF_OPERATION = 0x6060;  /// ����ģʽ(д)>>> 0x6060
static constexpr uint16_t OD_MODES_OF_DISPLAY = 0x6061;  /// ����ģʽ(��)<<< 0x6061
static constexpr uint16_t OD_ERROR_CODE = 0x603F;  /// ������<<< 0x603F

static constexpr uint16_t OD_TARGET_CURRENT = 0x6071;  /// Ŀ�����>>> 0x6071
static constexpr uint16_t OD_ACTUAL_CURRENT = 0x6078;  /// ʵ�ʵ���<<< 0x6078
static constexpr uint16_t OD_TARGET_POSITION = 0x607A;  /// Ŀ��λ��>>> 0x607A
static constexpr uint16_t OD_ACTUAL_POSITION = 0x6064;  /// ʵ��λ��<<< 0x6064
static constexpr uint16_t OD_TARGET_VELOCITY = 0x60FF;  /// Ŀ���ٶ�>>> 0x60FF
static constexpr uint16_t OD_ACTUAL_VELOCITY = 0x606C;  /// ʵ���ٶ�<<< 0x606C

static constexpr uint16_t OD_ACCELERATION = 0x6083;  /// ���ٶ�>>> 0x6083
static constexpr uint16_t OD_DECELERATION = 0x6084;  /// ���ٶ�>>> 0x6084


/**
 * @brief ���ڱ�ʾ DS402 �г����ĵ������ģʽ
 * ����ֻ��ʾ��������ɸ������������� DS402 ʵ��ϸ������
 */
enum class MotorMode : uint8_t {
    PROFILE_POSITION = 1,  // λ��ģʽ
    PROFILE_VELOCITY = 3,  // �ٶ�ģʽ
    PROFILE_TORQUE = 4,  // ����/����ģʽ
    HOMING_MODE = 6,  // ����ģʽ
    INTERPOLATED_POS = 7,  // ��ֵλ��ģʽ
    CYCLIC_SYNC_POS = 8,  // ͬ������λ��
    CYCLIC_SYNC_VEL = 9,  // ͬ�������ٶ�
    CYCLIC_SYNC_TORQUE = 10, // ͬ����������
};

/**
 * @brief ״̬��ģʽ�ṹ��
 *
 * - �����ʾ������˿����֡�״̬�֡�����ģʽ���������
 * - �����û�Ҫ�󣺼�ʹ�߼����� S16/U16��Ҳ�� uint8_t ���鱣��ԭʼֵ
 * - �����ݱ��ȿ��Ը�����Ҫ��չΪ std::array �� std::vector
 */
struct StateAndMode
{
    // DS402: ������(0x6040)��״̬��(0x6041)ͨ����2�ֽ�
    // ����ģʽ(0x6060)ͨ��1�ֽڣ�������(0x603F)ͨ��2�ֽ�
    // ����ֻ��ʾ������ԭʼ�ֽڶ���������
    volatile uint8_t controlWordRaw[2];       /// �����֣�ԭʼ2�ֽڣ�>>
    volatile uint8_t statusWordRaw[2];        /// ״̬�֣�ԭʼ2�ֽڣ�<<
    volatile uint8_t modeOfOperationRaw[1];   /// ����ģʽ��ԭʼ1�ֽڣ�>>
    volatile uint8_t errorCodeRaw[2];         /// ���������루ԭʼ2�ֽڣ�<<

    // 
    const uint16_t controlWordIndex = OD_CONTROL_WORD;      // 0x6040
    const uint16_t statusWordIndex = OD_STATUS_WORD;       // 0x6041
    const uint16_t modeOfOperationIndex = OD_MODES_OF_OPERATION;// 0x6060
    const uint16_t errorCodeIndex = OD_ERROR_CODE;        // 0x603F
};

/**
 * @brief �����ṹ��
 *
 * - ʵ��ͨ���� S16 / U16�������ﰴҪ��ʹ�� uint8_t ����洢ԭʼֵ
 * - ͬ��ֵ�����͡��ͽ���ע����˵��
 */
struct MotorCurrent
{
    // ԭʼʵ�ʵ��������� S16(2�ֽ�)
    volatile uint8_t actualCurrentRaw[2];
    // ԭʼĿ����������� S16(2�ֽ�)
    volatile uint8_t targetCurrentRaw[2];

    // ��������õ��ġ�������ֵ����ʵ����ֵ���ȣ����� readRefresh() �и���
    //ʵ�ʵ����������<<
    float actualCurrent = 0.0f;
    //Ŀ������������>>
    float targetCurrent = 0.0f;

    // ʵ�ʵ����ֵ� 0x6078
    const uint16_t actualCurrentIndex = OD_ACTUAL_CURRENT; 
    // Ŀ������ֵ� 0x6071
    const uint16_t targetCurrentIndex = OD_TARGET_CURRENT; 
};

/**
 * @brief λ�ýṹ��
 *
 * - λ��ͨ���� S32����������� 4�ֽ� ԭʼ���ݱ�ʾ
 */
struct MotorPosition
{
    volatile uint8_t actualPositionRaw[4];
    volatile uint8_t targetPositionRaw[4];

    float actualPositionDeg = 0.0f;  /// ʵ��ֵ(�Ƕ�)>>
    float targetPositionDeg = 0.0f;  /// Ŀ��ֵ(�Ƕ�)<<

    // ������ֵ��ʮ���ƣ�����ʾ��
    int32_t actualPositionCnt = 0;
    int32_t targetPositionCnt = 0;

    // ��Ӧ�����ֵ��ַ
    const uint16_t actualPositionIndex = OD_ACTUAL_POSITION; // 0x6064
    const uint16_t targetPositionIndex = OD_TARGET_POSITION; // 0x607A
};

/**
 * @brief �ٶȽṹ��
 *
 * - �ٶ�Ҳ������ S32���������������� S16 ��ʾ������ʾ���� 4�ֽڴ洢
 */
struct MotorVelocity
{
    volatile uint8_t actualVelocityRaw[4];//ʵ�ʵ�
    volatile uint8_t targetVelocityRaw[4];

    float actualVelocityRPM = 0.0f; ///< ʵ���ٶ� (rpm)
    float targetVelocityRPM = 0.0f; ///< Ŀ���ٶ� (rpm)

    int32_t actualVelocityCnt = 0;    ///< ����������ֵ
    int32_t targetVelocityCnt = 0;

    // ��Ӧ�����ֵ��ַ
    const uint16_t actualVelocityIndex = OD_ACTUAL_VELOCITY; //0x606C
    const uint16_t targetVelocityIndex = OD_TARGET_VELOCITY; //0x60FF
};

/**
 * @brief �Ӽ��ٶȽṹ��
 *
 * - DS402 ������ U32���������ü��ٶ� / ���ٶ�
 */
struct MotorAccelDecel
{
    volatile uint8_t accelRaw[4];
    volatile uint8_t decelRaw[4];

    uint32_t accelValue = 0;
    uint32_t decelValue = 0;

    // ��Ӧ�����ֵ��ַ
    const uint16_t accelIndex = OD_ACCELERATION;   //0x6083
    const uint16_t decelIndex = OD_DECELERATION;   //0x6084
};


/**
 * @brief ���ĵ����
 *
 * ������
 * 1. ״̬&ģʽ
 * 2. ����
 * 3. λ��
 * 4. �ٶ�
 * 5. �Ӽ��ٶ�
 *
 * �Լ��򵥵ĳ�ʼ������ˢ�¡�дˢ�µȽӿڡ�
 */
class Motor
{
public:
    Motor() = default;
    ~Motor() = default;

    /**
     * @brief ��ʼ��������
     *  - ��ʱ��ʵ�֣����ڴ˽�����ԭʼ�������㣬��ģʽ��Ϊ��ȫģʽ�ȡ�
     */
    void init()
    {
        // �˴���������������ӽṹ��������һ�ΰ�ȫ��ʼ��
        // ����:
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
     * @brief ��ˢ�·���
     *
     * һ���ڡ������̡߳��� PDO/SDO �յ���ԭʼ���� (uint8_t[]) ��ӳ���
     * д�뵽���� Motor ���Ӧ�ֶΡ�Ȼ��������Խ��ж������ֵ���㣬����� Raw ת����ʵ��ֵ��
     */
    void readRefresh()
    {
        std::lock_guard<std::mutex> lock(mtx_);

        // (ʾ��) �� raw �Ŀ�����/״̬��/ģʽ��ת������Ҫ�� int ��ö�٣�
        // controlWord = (uint16_t)( (stateAndMode.controlWordRaw[1] << 8) |
        //                            stateAndMode.controlWordRaw[0] );
        // ...
        // modeOfOperation = (MotorMode)(stateAndMode.modeOfOperationRaw[0]);

        // ����
        {
            // ���ֽ��� [0], ���ֽ��� [1]����ֻ�Ǿ���������С�˾�Ҫ�෴��
            int16_t actualI = static_cast<int16_t>(
                (current.actualCurrentRaw[1] << 8) | current.actualCurrentRaw[0]);
            int16_t targetI = static_cast<int16_t>(
                (current.targetCurrentRaw[1] << 8) | current.targetCurrentRaw[0]);

            // ת��Ϊ������(��ʾ�����������ĵ��涨 1������=1mA)
            current.actualCurrent = static_cast<float>(actualI);
            current.targetCurrent = static_cast<float>(targetI);
        }

        // λ��
        {
            // ���� raw[0] Ϊ����ֽ�
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
            // ת�ɽǶ�(��ʾ�������� 65536 ��Ӧ 360 ��)
            position.actualPositionDeg = actualPos * (360.0f / 65536.0f);
            position.targetPositionDeg = targetPos * (360.0f / 65536.0f);
        }

        // �ٶ� (ͬ��)
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
            // ��ʾ�������� 1����=1rpm
            velocity.actualVelocityRPM = static_cast<float>(actualVel);
            velocity.targetVelocityRPM = static_cast<float>(targetVel);
        }

        // �Ӽ��ٶ�
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
     * @brief дˢ�·���
     *
     * - �����÷����ڶ�ʱ���߳��У����� Motor ������ targetXXX ��ֵ��װҪ���͵� SDO/PDO ����
     * - ��д�뵽 motorXXRaw[] �У�Ȼ�����ʵ�ʵķ��ͺ���
     */
    void writeRefresh()
    {
        std::lock_guard<std::mutex> lock(mtx_);

        // ���磺�� targetPositionDeg ת�� raw[0..3]
        // ������λ���ķ����߳�ʵ�ʷ��� SDO/PDO
        // ����ֻ����ʾ
        int32_t tarPos = position.targetPositionCnt;
        position.targetPositionRaw[0] = (uint8_t)(tarPos & 0xFF);
        position.targetPositionRaw[1] = (uint8_t)((tarPos >> 8) & 0xFF);
        position.targetPositionRaw[2] = (uint8_t)((tarPos >> 16) & 0xFF);
        position.targetPositionRaw[3] = (uint8_t)((tarPos >> 24) & 0xFF);

        // ͬ�� targetVelocity, targetCurrent, accel, decel �ȶ�����д��
        // ...
    }

public:
    // �����Ǽ����ӽṹ���������������Ϊ public �� private
    StateAndMode    stateAndMode;
    MotorCurrent    current;
    MotorPosition   position;
    MotorVelocity   velocity;
    MotorAccelDecel accelDecel;

private:
    // ���� readRefresh / writeRefresh �����Ļ�����
    std::mutex mtx_;
};

#endif // CLASS_MOTOR_HPP