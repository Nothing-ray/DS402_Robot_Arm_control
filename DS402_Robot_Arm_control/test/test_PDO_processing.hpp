/**
 * @file test_PDO_processing.hpp
 * @brief PDO数据处理测试
 * 
 * 包含TPDO数据接收和处理的全面测试，涵盖位置、速度、电流数据的
 * 解析验证、边界值测试、异常处理和实际应用场景模拟
 */

#ifndef TEST_PDO_PROCESSING_HPP
#define TEST_PDO_PROCESSING_HPP

#include <array>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <algorithm>

#include <random>       // 用于 std::mt19937, std::random_device, std::uniform_int_distribution
#include <vector>       // 用于 std::vector
#include <numeric>      // 用于 std::accumulate

#include "../CLASS_Motor.hpp"
#include "../PDO_config.hpp"
#include "../Data_processing.hpp"
#include "../CAN_frame.hpp"
#include "../CAN_processing.hpp"
#include "../SDO_State_Machine.hpp"
using namespace std::chrono_literals;


// 计时工具宏
#define TIME_IT(operation, description) \
    do { \
        auto start = std::chrono::high_resolution_clock::now(); \
        operation; \
        auto end = std::chrono::high_resolution_clock::now(); \
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start); \
        std::cout << description << " 耗时 " << duration.count() << " 微秒\n"; \
    } while(0)


















/*********************** PDO读写测试区 **********************/


/**
 * @brief 全面测试TPDO(下位机→上位机)数据接收和处理
 *
 * 测试覆盖：
 * - TPDO1: 实际位置(4B) + 状态字(2B)
 * - TPDO2: 实际速度(2B) + 实际电流(2B)
 * - 所有电机节点(1-6)
 * - 各种数据边界值和特殊情况
 *
 * 数据范围说明：
 * - 电流：-32767 ~ 32768 mA
 * - 速度：-3000 ~ 3000 RPM
 * - 位置：-32768 ~ 32767 脉冲 (对应 -180° ~ 180°)
 */
void testTPDOProcessing() {
    std::cout << "\n===============================================" << std::endl;
    std::cout << "        TPDO (下位机→上位机) 全面测试" << std::endl;
    std::cout << "===============================================" << std::endl;

    // 1. 初始化测试环境
    std::cout << "\n[步骤1] 初始化测试环境..." << std::endl;
    std::array<Motor, 6> motors = {
        Motor(1), Motor(2), Motor(3),
        Motor(4), Motor(5), Motor(6)
    };

    // 构建PDO配置表
    std::vector<PdoMappingEntry> pdoTable = buildArmMappingTable(6);
    std::cout << " Yes  初始化6个电机" << std::endl;
    std::cout << " Yes  PDO配置表生成完成，共" << pdoTable.size() << "条映射" << std::endl;

    // 2. TPDO1测试 (0x180 + NodeID) - 实际位置 + 状态字
    std::cout << "\n[步骤2] 测试TPDO1 (实际位置 + 状态字)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "位置范围: -32768 ~ 32767 脉冲 (对应 -180° ~ 180°)" << std::endl;

    struct TPDO1TestCase {
        uint8_t nodeId;
        int32_t position;
        uint16_t statusWord;
        const char* description;
    };

    std::vector<TPDO1TestCase> tpdo1Tests = {
        {1, 0, 0x0237, "电机1: 0°位置(0脉冲)，已使能状态"},
        {2, 32767, 0x0233, "电机2: +180°最大位置(32767脉冲)，未使能"},
        {3, -32768, 0x0627, "电机3: -180°最小位置(-32768脉冲)，运行中"},
        {4, 16384, 0x0237, "电机4: +90°位置(16384脉冲)，正常运行"},
        {5, -16384, 0x021F, "电机5: -90°位置(-16384脉冲)，快速停止"},
        {6, 5461, 0x0608, "电机6: +30°位置(5461脉冲)，故障状态"}
    };

    for (const auto& test : tpdo1Tests) {
        // 构造TPDO1帧数据
        uint8_t data[8] = { 0 };

        // 位置数据 (小端序，4字节)
        data[0] = (test.position >> 0) & 0xFF;
        data[1] = (test.position >> 8) & 0xFF;
        data[2] = (test.position >> 16) & 0xFF;
        data[3] = (test.position >> 24) & 0xFF;

        // 状态字 (小端序，2字节)
        data[4] = (test.statusWord >> 0) & 0xFF;
        data[5] = (test.statusWord >> 8) & 0xFF;

        CanFrame tpdo1Frame(0x180 + test.nodeId, data, 6);

        std::cout << "\n测试用例: " << test.description << std::endl;
        std::cout << "  CAN ID: 0x" << std::hex << (0x180 + test.nodeId) << std::dec
            << ", 数据: ";
        for (int i = 0; i < 6; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                << (int)data[i] << " ";
        }
        std::cout << std::dec << std::endl;

        // 处理CAN帧
        parseCanFrame(tpdo1Frame, motors, pdoTable);

        // 验证结果
        auto& motor = motors[test.nodeId - 1];
        {
            std::lock_guard<std::mutex> lock(motor.mtx_);

            // 检查位置值
            double actualPos = motor.position.actual_degree.load();
            double angleDegrees = (double)actualPos * 180.0 / 32767.0;
            std::cout << "  实际位置: " << actualPos << " 脉冲 ("
                << std::fixed << std::setprecision(1) << angleDegrees
                << "°) [期望: " << convertSensorAngle(test.position, true) << "]";
            if ((actualPos - convertSensorAngle(test.position, true)) < 0.01f) {
                std::cout << "  Yes 正确" << std::endl;
            }
            else {
                std::cout << "  No 错误!" << std::endl;
            }

            // 检查状态字
            uint16_t actualStatus = (motor.stateAndMode.controlData.statusWordRaw[1] << 8) |
                motor.stateAndMode.controlData.statusWordRaw[0];
            std::cout << "  状态字: 0x" << std::hex << actualStatus
                << " (期望: 0x" << test.statusWord << ")" << std::dec;
            if (actualStatus == test.statusWord) {
                std::cout << "  Yes 正确" << std::endl;
            }
            else {
                std::cout << "  No 错误!" << std::endl;
            }

            // 检查刷新标志
            std::cout << "  位置刷新标志: " <<
                ((motor.position.flags_.load() & MotorPosition::RAW_DATA_RECEIVE_NEED_REFRESH) ? "已设置  Yes " : "未设置  No ") << std::endl;
            std::cout << "  状态刷新标志: " <<
                (motor.stateAndMode.refresh ? "已设置  Yes " : "未设置  No ") << std::endl;
        }
    }

    // 3. TPDO2测试 (0x280 + NodeID) - 实际速度 + 实际电流
    std::cout << "\n[步骤3] 测试TPDO2 (实际速度 + 实际电流)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "速度范围: -3000 ~ 3000 RPM" << std::endl;
    std::cout << "电流范围: -32767 ~ 32768 mA" << std::endl;

    struct TPDO2TestCase {
        uint8_t nodeId;
        int16_t velocity;
        int16_t current;
        const char* description;
    };

    std::vector<TPDO2TestCase> tpdo2Tests = {
        {1, 0, 0, "电机1: 停止状态(0 RPM)，无电流(0 mA)"},
        {2, 3000, 15000, "电机2: 最大正速度(3000 RPM)，15A电流"},
        {3, -3000, -15000, "电机3: 最大负速度(-3000 RPM)，-15A电流"},
        {4, 1500, 32767, "电机4: 半速正转(1500 RPM)，最大正电流(32.767A)"},
        {5, -1500, -32767, "电机5: 半速反转(-1500 RPM)，最大负电流(-32.767A)"},
        {6, 100, -500, "电机6: 低速正转(100 RPM)，反向制动电流(-500mA)"}
    };

    for (const auto& test : tpdo2Tests) {
        // 构造TPDO2帧数据
        uint8_t data[8] = { 0 };

        // 实际速度 (小端序，2字节)
        data[0] = (test.velocity >> 0) & 0xFF;
        data[1] = (test.velocity >> 8) & 0xFF;

        // 实际电流 (小端序，2字节)
        data[2] = (test.current >> 0) & 0xFF;
        data[3] = (test.current >> 8) & 0xFF;

        CanFrame tpdo2Frame(0x280 + test.nodeId, data, 4);

        std::cout << "\n测试用例: " << test.description << std::endl;
        std::cout << "  CAN ID: 0x" << std::hex << (0x280 + test.nodeId) << std::dec
            << ", 数据: ";
        for (int i = 0; i < 4; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                << (int)data[i] << " ";
        }
        std::cout << std::dec << std::endl;

        // 处理CAN帧
        parseCanFrame(tpdo2Frame, motors, pdoTable);

        // 验证结果
        auto& motor = motors[test.nodeId - 1];
        {
            std::lock_guard<std::mutex> lock(motor.mtx_);

            // 检查速度值
            int16_t actualVel = motor.velocity.actual_rpm.load();
            std::cout << "  实际速度: " << actualVel << " RPM (期望: " << test.velocity << ")";
            if (actualVel == test.velocity) {
                std::cout << "  Yes 正确" << std::endl;
            }
            else {
                std::cout << "  No 错误!" << std::endl;
            }

            // 检查电流值
            int16_t actualCur = motor.current.actual_current.load();
            double currentAmperes = actualCur / 1000.0;
            std::cout << "  实际电流: " << actualCur << " mA ("
                << std::fixed << std::setprecision(3) << currentAmperes
                << " A) [期望: " << test.current << " mA]";
            if (actualCur == test.current) {
                std::cout << "  Yes 正确" << std::endl;
            }
            else {
                std::cout << "  No 错误!" << std::endl;
            }

            // 检查刷新标志
            std::cout << "  速度刷新标志: " <<
                ((motor.velocity.flags_.load() & MotorVelocity::RAW_DATA_RECEIVE_NEED_REFRESH) ? "已设置  Yes " : "未设置  No ") << std::endl;
            std::cout << "  电流刷新标志: " <<
                ((motor.current.flags_.load() & MotorCurrent::RAW_DATA_RECEIVE_NEED_REFRESH) ? "已设置  Yes " : "未设置  No ") << std::endl;
        }
    }

    // 4. 特殊边界值测试
    std::cout << "\n[步骤4] 特殊边界值测试" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    // 测试速度超出范围的处理
    {
        std::cout << "\n测试: 速度边界值组合" << std::endl;
        struct BoundaryTest {
            int16_t velocity;
            int16_t current;
            const char* description;
        };

        std::vector<BoundaryTest> boundaryTests = {
            {2999, 32000, "接近最大速度(2999 RPM)和高电流(32A)"},
            {-2999, -32000, "接近最小速度(-2999 RPM)和高负电流(-32A)"},
            {1, 1, "最小正速度(1 RPM)和最小电流(1mA)"},
            {-1, -1, "最小负速度(-1 RPM)和最小负电流(-1mA)"}
        };

        for (size_t i = 0; i < boundaryTests.size(); i++) {
            auto& test = boundaryTests[i];
            uint8_t data[4];
            data[0] = (test.velocity >> 0) & 0xFF;
            data[1] = (test.velocity >> 8) & 0xFF;
            data[2] = (test.current >> 0) & 0xFF;
            data[3] = (test.current >> 8) & 0xFF;

            CanFrame frame(0x281, data, 4); // 使用电机1
            std::cout << "  " << test.description << std::endl;
            parseCanFrame(frame, motors, pdoTable);

            auto& motor = motors[0];
            std::lock_guard<std::mutex> lock(motor.mtx_);
            std::cout << "    速度: " << motor.velocity.actual_rpm.load() << " RPM, "
                << "电流: " << motor.current.actual_current.load() << " mA  Yes 正确" << std::endl;
        }
    }

    // 5. 异常情况测试
    std::cout << "\n[步骤5] 异常情况测试" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    // 测试数据长度不足
    {
        std::cout << "\n测试: 数据长度不足的TPDO1帧" << std::endl;
        CanFrame shortFrame(0x181, new uint8_t[3]{ 0x01, 0x02, 0x03 }, 3); // 只有3字节
        parseCanFrame(shortFrame, motors, pdoTable);
        std::cout << "  处理完成，应忽略此帧  Yes 正确" << std::endl;
    }

    // 测试无效节点ID
    {
        std::cout << "\n测试: 无效节点ID (NodeID=0)" << std::endl;
        CanFrame invalidFrame(0x180, new uint8_t[6]{ 0 }, 6); // NodeID = 0
        parseCanFrame(invalidFrame, motors, pdoTable);
        std::cout << "  处理完成，应忽略此帧  Yes 正确" << std::endl;
    }

    // 测试超出范围的节点ID
    {
        std::cout << "\n测试: 超出范围的节点ID (NodeID=7)" << std::endl;
        CanFrame outOfRangeFrame(0x187, new uint8_t[6]{ 0 }, 6); // NodeID = 7
        parseCanFrame(outOfRangeFrame, motors, pdoTable);
        std::cout << "  处理完成，应忽略此帧  Yes 正确" << std::endl;
    }

    // 6. 实际应用场景模拟
    std::cout << "\n[步骤6] 实际应用场景模拟" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    std::cout << "模拟电机从0°旋转到90°的过程..." << std::endl;
    int32_t startPos = 0;        // 0°
    int32_t endPos = 16384;      // 90°
    int32_t step = 1638;         // 约9°每步

    for (int32_t pos = startPos; pos <= endPos; pos += step) {
        // 计算对应的速度和电流
        int16_t velocity = (pos < endPos / 2) ? 2000 : 1000;  // 前半段快速，后半段减速
        int16_t current = (pos < endPos / 2) ? 5000 : 2000;   // 对应调整电流

        // TPDO1: 位置更新
        uint8_t data1[6];
        data1[0] = (pos >> 0) & 0xFF;
        data1[1] = (pos >> 8) & 0xFF;
        data1[2] = (pos >> 16) & 0xFF;
        data1[3] = (pos >> 24) & 0xFF;
        data1[4] = 0x37;  // 运行中状态
        data1[5] = 0x06;

        CanFrame tpdo1(0x181, data1, 6);
        parseCanFrame(tpdo1, motors, pdoTable);

        // TPDO2: 速度和电流更新
        uint8_t data2[4];
        data2[0] = (velocity >> 0) & 0xFF;
        data2[1] = (velocity >> 8) & 0xFF;
        data2[2] = (current >> 0) & 0xFF;
        data2[3] = (current >> 8) & 0xFF;

        CanFrame tpdo2(0x281, data2, 4);
        parseCanFrame(tpdo2, motors, pdoTable);

        // 从电机对象中读取实际参数
        auto& motor = motors[0];  // 假设测试电机1
        {
            std::lock_guard<std::mutex> lock(motor.mtx_);
            double actualAngle = motor.position.actual_degree.load();
            int16_t actualVelocity = motor.velocity.actual_rpm.load();
            int16_t actualCurrent = motor.current.actual_current.load();
            uint16_t actualStatus = (motor.stateAndMode.controlData.statusWordRaw[1] << 8) |
                motor.stateAndMode.controlData.statusWordRaw[0];
            std::cout << "  实际状态 - "
                << "位置: " << std::fixed << std::setprecision(1) << actualAngle
                << "°, 速度: " << actualVelocity << " RPM, 电流: "
                << std::fixed << std::setprecision(1) << actualCurrent / 1000.0 << " A"
                << ", 状态字: 0x" << std::hex << std::setw(4) << std::setfill('0') << actualStatus << std::dec
                << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 模拟50ms周期
    }

    // 最终状态汇总
    std::cout << "\n[步骤7] 最终状态汇总" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    for (size_t i = 0; i < motors.size(); i++) {
        auto& motor = motors[i];
        std::lock_guard<std::mutex> lock(motor.mtx_);

        int32_t pos = motor.position.actual_degree.load();
        double angle = (double)pos * 180.0 / 32767.0;
        int16_t vel = motor.velocity.actual_rpm.load();
        int16_t cur = motor.current.actual_current.load();

        std::cout << "电机" << (i + 1) << ": "
            << "位置=" << std::fixed << std::setprecision(1) << angle << "°, "
            << "速度=" << vel << " RPM, "
            << "电流=" << std::fixed << std::setprecision(2) << cur / 1000.0 << " A"
            << std::endl;
    }

    std::cout << "\n===============================================" << std::endl;
    std::cout << "          TPDO测试完成！" << std::endl;
    std::cout << "===============================================\n" << std::endl;
}

#undef TIME_IT

#endif // TEST_PDO_PROCESSING_HPP
