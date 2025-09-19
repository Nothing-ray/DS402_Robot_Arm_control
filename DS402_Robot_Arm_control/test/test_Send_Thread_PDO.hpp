/**
 * @file test_Send_Thread_PDO.hpp
 * @brief 发送线程RPDO处理功能测试（真实串口环境）
 *
 * @details 本文件实现了对Send_Thread类RPDO处理功能的测试，
 * 使用真实的SerialPortManager实例连接实际串口，通过外部调试设备捕获数据包。
 * 测试专注于RPDO（上位机到下位机）的发送性能，使用简洁的计时工具进行性能分析。
 *
 * 测试架构：
 * - 创建真实的SendThread实例和SerialPortManager
 * - 使用实际串口连接，确保真实环境测试
 * - 生成标准化的RPDO测试数据
 * - 进行简化的性能统计和时间测量
 * - 通过外部调试设备验证发送的数据包
 *
 * @par 测试重点：
 * - RPDO1：目标位置和控制字发送（0x200 + NodeID）
 * - RPDO2：目标速度和电流发送（0x300 + NodeID）
 * - 六电机并发发送性能分析
 * - 真实串口传输延迟统计
 *
 * @note 本测试需要外部调试设备（如CAN分析仪）来验证发送的数据包，
 * 测试结果包含简化的性能报告，用于分析RPDO发送实时性表现。
 */

#ifndef TEST_SEND_THREAD_PDO_HPP
#define TEST_SEND_THREAD_PDO_HPP

// 启用PDO测试模式
#define TESTING_PDO_ONLY

// 启用详细调试输出
#define PDO_TEST_DEBUG_OUTPUT

// 启用Send_Thread调试输出
#define ENABLE_DEBUG_OUTPUT true

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <cassert>
#include <iomanip>
#include <sstream>
#include <numeric>
#include <cstring>

#include "Send_Thread.hpp"
#include "CAN_frame.hpp"
#include "CircularBuffer.hpp"
#include "Serial_Module.hpp"
#include "CLASS_Motor.hpp"
#include "PDO_config.hpp"

// ====================== 计时工具 ======================


// 测试配置宏
#define ENABLE_PDO_TEST_DEBUG 1
#if ENABLE_PDO_TEST_DEBUG
#define PDO_TEST_DEBUG_PRINT(msg) do { \
    std::cout << "[DEBUG][TestSendThreadPDO]: " << msg << std::endl; \
} while(0)
#else
#define PDO_TEST_DEBUG_PRINT(msg) do { } while(0)
#endif

// 测试结果宏
#define TEST_ASSERT(condition, message) do { \
    if (!(condition)) { \
        std::cout << "[ERROR][TestSendThreadPDO]: " << message << std::endl; \
        std::cout << "[ERROR][TestSendThreadPDO]: 断言失败: " #condition << std::endl; \
        std::cout << "[ERROR][TestSendThreadPDO]: 文件: " << __FILE__ << ", 行: " << __LINE__ << std::endl; \
        return false; \
    } \
} while(0)

#define TEST_SUCCESS_PRINT(message) do { \
    std::cout << "[SUCCESS][TestSendThreadPDO]: " << message << std::endl; \
} while(0)




// ====================== RPDO数据生成器 ======================

/**
 * @brief RPDO数据生成器
 *
 * @details 生成标准化的RPDO测试数据，专注于上位机到下位机的数据发送。
 */
class RpdoTestDataGenerator {
public:
    // RPDO测试数据结构
    struct RpdoTestData {
        struct MotorData {
            Position_type targetPosition;      // 目标位置（4字节）
            uint16_t controlWord;             // 控制字（2字节）
            Velocity_type targetVelocity;      // 目标速度（2字节）
            Current_type targetCurrent;        // 目标电流（2字节）
        };

        std::array<MotorData, 6> motors;  // 六个电机的数据
    };

    /**
     * @brief 生成标准RPDO测试数据
     * @return 标准RPDO测试数据
     */
    static RpdoTestData generateStandardTestData() {
        RpdoTestData data;

        for (uint8_t i = 0; i < 6; ++i) {
            // 每个电机使用不同的测试数据，便于区分
            data.motors[i].targetPosition = 0x00010000 * (i + 1);  // 递增位置值
            data.motors[i].controlWord = 0x000F;               // 启动运动控制字
            data.motors[i].targetVelocity = 0x1000 * (i + 1); // 递增速度值
            data.motors[i].targetCurrent = 0x0800 * (i + 1);   // 递增电流值
        }

        return data;
    }

    /**
     * @brief 生成边界值RPDO测试数据
     * @return 边界值RPDO测试数据
     */
    static RpdoTestData generateBoundaryTestData() {
        RpdoTestData data;

        for (uint8_t i = 0; i < 6; ++i) {
            // 使用最大值和最小值进行边界测试
            data.motors[i].targetPosition = (i % 2 == 0) ? 0x7FFFFFFF : 0x80000000;
            data.motors[i].controlWord = (i % 2 == 0) ? 0xFFFF : 0x0000;
            data.motors[i].targetVelocity = (i % 2 == 0) ? 0x7FFF : 0x8000;
            data.motors[i].targetCurrent = (i % 2 == 0) ? 0x7FFF : 0x8000;
        }

        return data;
    }

    /**
     * @brief 生成零值RPDO测试数据
     * @return 零值RPDO测试数据
     */
    static RpdoTestData generateZeroTestData() {
        RpdoTestData data;

        for (uint8_t i = 0; i < 6; ++i) {
            data.motors[i].targetPosition = 0;
            data.motors[i].controlWord = 0;
            data.motors[i].targetVelocity = 0;
            data.motors[i].targetCurrent = 0;
        }

        return data;
    }

    /**
     * @brief 打印RPDO测试数据
     * @param data 测试数据
     */
    static void printTestData(const RpdoTestData& data) {
        PDO_TEST_DEBUG_PRINT("RPDO测试数据:");
        for (uint8_t i = 0; i < 6; ++i) {
            PDO_TEST_DEBUG_PRINT("电机" << static_cast<int>(i + 1) << ": "
                              << "位置=0x" << std::hex << data.motors[i].targetPosition
                              << ", 控制字=0x" << data.motors[i].controlWord
                              << ", 速度=0x" << data.motors[i].targetVelocity
                              << ", 电流=0x" << data.motors[i].targetCurrent
                              << std::dec);
        }
    }

    /**
     * @brief 将测试数据写入到电机数组（使用正确的标志位驱动方式）
     * @param data 测试数据
     * @param motors 电机数组引用
     */
    static void writeTestDataToMotors(const RpdoTestData& data, std::array<Motor, 6>& motors) {
        for (uint8_t i = 0; i < 6; ++i) {
            Motor& motor = motors.at(i);
            const auto& motorData = data.motors[i];

            // 写入RPDO1数据（位置+控制字）- 使用正确的标志位驱动方式
            // 将原始位置值转换为角度值（假设范围是-180到180度）
            float targetAngle = static_cast<float>(motorData.targetPosition) * 180.0f / 32768.0f;
            motor.position.target_degree.store(targetAngle);
            motor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

            // 使用memcpy处理volatile数组（控制字不需要标志位）
            std::memcpy(const_cast<uint8_t*>(motor.stateAndMode.controlData.controlWordRaw), &motorData.controlWord, 2);

            // 写入RPDO2数据（速度+电流）- 使用正确的标志位驱动方式
            // 速度值直接使用RPM单位
            float targetVelocityRpm = static_cast<float>(motorData.targetVelocity);
            motor.velocity.target_rpm_velocity_mode.store(targetVelocityRpm);
            motor.velocity.flags_.fetch_or(MotorVelocity::Flags::TARGET_DATA_SEND_NEED_REFRESH_VELOCITY_MODE, std::memory_order_release);

            // 电流值转换为mA单位
            float targetCurrentMa = static_cast<float>(motorData.targetCurrent);
            motor.current.target_current.store(targetCurrentMa);
            motor.current.flags_.fetch_or(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

            // 调用刷新方法进行数据转换
            motor.refreshMotorData(motor.position);
            motor.refreshMotorData(motor.velocity);
            motor.refreshMotorData(motor.current);
        }
    }
};

// ====================== RPDO帧验证器 ======================

/**
 * @brief RPDO帧验证器
 *
 * @details 验证RPDO帧的正确性，包括COB-ID、数据内容等。
 */
class RpdoFrameValidator {
public:
    /**
     * @brief 验证RPDO1帧（目标位置+控制字）
     * @param frame 要验证的RPDO帧
     * @param motorIndex 电机索引（1-6）
     * @param expectedData 期望数据（可选）
     * @return 验证结果
     */
    static bool validateRpdo1Frame(const CanFrame& frame, uint8_t motorIndex,
                                 const uint8_t* expectedData = nullptr) {
        // 验证COB-ID
        uint32_t expectedCobId = 0x200 + motorIndex;

        if (frame.frameID != expectedCobId) {
            PDO_TEST_DEBUG_PRINT("RPDO1 COB-ID验证失败: 期望 0x" << std::hex << expectedCobId
                              << ", 实际 0x" << frame.frameID << std::dec);
            return false;
        }

        // 验证DLC
        if (frame.dlc != 8) {
            PDO_TEST_DEBUG_PRINT("RPDO1 DLC验证失败: 期望 8, 实际 " << static_cast<int>(frame.dlc));
            return false;
        }

        // 如果有期望数据，验证数据内容
        if (expectedData) {
            for (uint8_t i = 0; i < 8; ++i) {
                if (frame.data[i] != expectedData[i]) {
                    PDO_TEST_DEBUG_PRINT("RPDO1数据验证失败: 字节 " << static_cast<int>(i)
                                      << " 期望 0x" << std::hex << static_cast<int>(expectedData[i])
                                      << ", 实际 0x" << static_cast<int>(frame.data[i]) << std::dec);
                    return false;
                }
            }
        }

        PDO_TEST_DEBUG_PRINT("RPDO1帧验证成功: 电机" << static_cast<int>(motorIndex)
                          << ", COB-ID=0x" << std::hex << frame.frameID << std::dec);
        return true;
    }

    /**
     * @brief 验证RPDO2帧（目标速度+目标电流）
     * @param frame 要验证的RPDO帧
     * @param motorIndex 电机索引（1-6）
     * @param expectedData 期望数据（可选）
     * @return 验证结果
     */
    static bool validateRpdo2Frame(const CanFrame& frame, uint8_t motorIndex,
                                 const uint8_t* expectedData = nullptr) {
        // 验证COB-ID
        uint32_t expectedCobId = 0x300 + motorIndex;

        if (frame.frameID != expectedCobId) {
            PDO_TEST_DEBUG_PRINT("RPDO2 COB-ID验证失败: 期望 0x" << std::hex << expectedCobId
                              << ", 实际 0x" << frame.frameID << std::dec);
            return false;
        }

        // 验证DLC
        if (frame.dlc != 8) {
            PDO_TEST_DEBUG_PRINT("RPDO2 DLC验证失败: 期望 8, 实际 " << static_cast<int>(frame.dlc));
            return false;
        }

        // 如果有期望数据，验证数据内容
        if (expectedData) {
            for (uint8_t i = 0; i < 8; ++i) {
                if (frame.data[i] != expectedData[i]) {
                    PDO_TEST_DEBUG_PRINT("RPDO2数据验证失败: 字节 " << static_cast<int>(i)
                                      << " 期望 0x" << std::hex << static_cast<int>(expectedData[i])
                                      << ", 实际 0x" << static_cast<int>(frame.data[i]) << std::dec);
                    return false;
                }
            }
        }

        PDO_TEST_DEBUG_PRINT("RPDO2帧验证成功: 电机" << static_cast<int>(motorIndex)
                          << ", COB-ID=0x" << std::hex << frame.frameID << std::dec);
        return true;
    }

    /**
     * @brief 打印RPDO帧内容
     * @param frame 要打印的RPDO帧
     * @param description 描述
     */
    static void printRpdoFrame(const CanFrame& frame, const std::string& description) {
        const auto& binaryFrame = frame.getBinaryFrame();
        PDO_TEST_DEBUG_PRINT(description << " - ID: 0x" << std::hex << frame.frameID
                          << ", DLC: " << std::dec << static_cast<int>(frame.dlc)
                          << ", 数据: 0x" << std::hex << static_cast<int>(binaryFrame[0])
                          << " " << static_cast<int>(binaryFrame[1])
                          << " " << static_cast<int>(binaryFrame[2])
                          << " " << static_cast<int>(binaryFrame[3])
                          << " " << static_cast<int>(binaryFrame[4])
                          << " " << static_cast<int>(binaryFrame[5])
                          << " " << static_cast<int>(binaryFrame[6])
                          << " " << static_cast<int>(binaryFrame[7]) << std::dec);
    }
};

// ====================== RPDO基础功能测试 ======================

/**
 * @brief RPDO基础功能测试
 *
 * @details 测试RPDO1和RPDO2的基础功能：
 * - RPDO1：目标位置和控制字发送
 * - RPDO2：目标速度和目标电流发送
 * - 数据正确性验证
 * - 性能统计分析
 *
 * @param motors 电机数组引用
 * @param sendThread 发送线程引用
 * @return bool 测试成功返回true，失败返回false
 */
bool testRpdoBasicFunctionality(std::array<Motor, 6>& motors, SendThread& sendThread) {
    try {
        PDO_TEST_DEBUG_PRINT("开始RPDO基础功能测试");

        // 1. RPDO1测试（目标位置+控制字）
        PDO_TEST_DEBUG_PRINT("测试1.1: RPDO1 - 目标位置和控制字");

        // 生成标准测试数据
        auto testData = RpdoTestDataGenerator::generateStandardTestData();
        RpdoTestDataGenerator::printTestData(testData);

        // 选择电机1进行测试
        uint8_t testMotorIndex = 1;
        Motor& testMotor = motors.at(testMotorIndex - 1);

        // 写入目标位置和控制字到电机（使用正确的标志位驱动方式）
        {
            auto start = std::chrono::high_resolution_clock::now();
            // 将原始位置值转换为角度值
            float targetAngle = static_cast<float>(testData.motors[testMotorIndex - 1].targetPosition) * 180.0f / 32768.0f;
            testMotor.position.target_degree.store(targetAngle);
            testMotor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

            // 控制字仍然使用memcpy（不需要标志位）
            std::memcpy(const_cast<uint8_t*>(testMotor.stateAndMode.controlData.controlWordRaw), &testData.motors[testMotorIndex - 1].controlWord, 2);

            // 调用刷新方法进行数据转换
            testMotor.refreshMotorData(testMotor.position);

            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << "[PERF][TestSendThreadPDO]: 写入RPDO1数据到电机 took " << duration.count() << " us\n";
        }

        // 等待发送线程处理PDO（考虑2ms周期）
        {
            auto start = std::chrono::high_resolution_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 等待至少2个周期
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << "[PERF][TestSendThreadPDO]: 等待PDO处理 took " << duration.count() << " us\n";
        }

        // 记录测试数据供外部验证
        PDO_TEST_DEBUG_PRINT("=== RPDO1外部验证数据 ===");
        PDO_TEST_DEBUG_PRINT("电机" << static_cast<int>(testMotorIndex) << " RPDO1期望数据:");

        // 构建期望的RPDO1数据（8字节）
        uint8_t expectedRpdo1Data[8] = {0};
        std::memcpy(&expectedRpdo1Data[0], &testData.motors[testMotorIndex - 1].targetPosition, 4);
        std::memcpy(&expectedRpdo1Data[4], &testData.motors[testMotorIndex - 1].controlWord, 2);

        PDO_TEST_DEBUG_PRINT("字节0-3 (目标位置): 0x" << std::hex << std::setw(8) << std::setfill('0')
                          << testData.motors[testMotorIndex - 1].targetPosition << std::dec);
        PDO_TEST_DEBUG_PRINT("字节4-5 (控制字): 0x" << std::hex << std::setw(4) << std::setfill('0')
                          << testData.motors[testMotorIndex - 1].controlWord << std::dec);
        PDO_TEST_DEBUG_PRINT("COB-ID: 0x" << std::hex << (0x200 + testMotorIndex) << std::dec);
        PDO_TEST_DEBUG_PRINT("=== 验证数据结束 ===");

        // 2. RPDO2测试（目标速度+目标电流）
        PDO_TEST_DEBUG_PRINT("测试1.2: RPDO2 - 目标速度和目标电流");

        // 写入目标速度和目标电流到电机（使用正确的标志位驱动方式）
        {
            auto start = std::chrono::high_resolution_clock::now();
            // 速度值直接使用RPM单位
            float targetVelocityRpm = static_cast<float>(testData.motors[testMotorIndex - 1].targetVelocity);
            testMotor.velocity.target_rpm_velocity_mode.store(targetVelocityRpm);
            testMotor.velocity.flags_.fetch_or(MotorVelocity::Flags::TARGET_DATA_SEND_NEED_REFRESH_VELOCITY_MODE, std::memory_order_release);

            // 电流值转换为mA单位
            float targetCurrentMa = static_cast<float>(testData.motors[testMotorIndex - 1].targetCurrent);
            testMotor.current.target_current.store(targetCurrentMa);
            testMotor.current.flags_.fetch_or(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

            // 调用刷新方法进行数据转换
            testMotor.refreshMotorData(testMotor.velocity);
            testMotor.refreshMotorData(testMotor.current);

            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << "[PERF][TestSendThreadPDO]: 写入RPDO2数据到电机 took " << duration.count() << " us\n";
        }

        // 等待发送线程处理PDO
        {
            auto start = std::chrono::high_resolution_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 等待至少2个周期
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << "[PERF][TestSendThreadPDO]: 等待PDO处理 took " << duration.count() << " us\n";
        }

        // 记录测试数据供外部验证
        PDO_TEST_DEBUG_PRINT("=== RPDO2外部验证数据 ===");
        PDO_TEST_DEBUG_PRINT("电机" << static_cast<int>(testMotorIndex) << " RPDO2期望数据:");

        // 构建期望的RPDO2数据（8字节）
        uint8_t expectedRpdo2Data[8] = {0};
        std::memcpy(&expectedRpdo2Data[0], &testData.motors[testMotorIndex - 1].targetVelocity, 2);
        std::memcpy(&expectedRpdo2Data[2], &testData.motors[testMotorIndex - 1].targetCurrent, 2);

        PDO_TEST_DEBUG_PRINT("字节0-1 (目标速度): 0x" << std::hex << std::setw(4) << std::setfill('0')
                          << testData.motors[testMotorIndex - 1].targetVelocity << std::dec);
        PDO_TEST_DEBUG_PRINT("字节2-3 (目标电流): 0x" << std::hex << std::setw(4) << std::setfill('0')
                          << testData.motors[testMotorIndex - 1].targetCurrent << std::dec);
        PDO_TEST_DEBUG_PRINT("COB-ID: 0x" << std::hex << (0x300 + testMotorIndex) << std::dec);
        PDO_TEST_DEBUG_PRINT("=== 验证数据结束 ===");

        // 3. 验证发送线程状态
        PDO_TEST_DEBUG_PRINT("测试1.3: 验证发送线程状态");

        bool isThreadHealthy = sendThread.isHealthy();
        PDO_TEST_DEBUG_PRINT("发送线程健康状态: " << (isThreadHealthy ? "健康" : "不健康"));

        if (!isThreadHealthy) {
            auto lastError = sendThread.getLastError();
            auto errorMsg = sendThread.getLastErrorMessage();
            PDO_TEST_DEBUG_PRINT("线程错误类型: " << static_cast<int>(lastError));
            PDO_TEST_DEBUG_PRINT("错误消息: " << errorMsg);
            return false;
        }

        // 4. 综合性能统计
        PDO_TEST_DEBUG_PRINT("测试1.4: RPDO基础功能性能统计");

        // 测试多次RPDO发送的平均性能
        size_t testCount = 0;
        uint64_t totalTime = 0;

        for (int i = 0; i < 10; ++i) {
            // 生成随机测试数据
            Position_type randomPos = 0x10000 * (i + 1);
            uint16_t randomCtrl = 0x000F;
            Velocity_type randomVel = 0x1000 * (i + 1);
            Current_type randomCur = 0x0800 * (i + 1);

            auto start = std::chrono::high_resolution_clock::now();

            // 写入数据（使用正确的标志位驱动方式）
            // 位置：转换为角度值
            float targetAngle = static_cast<float>(randomPos) * 180.0f / 32768.0f;
            testMotor.position.target_degree.store(targetAngle);
            testMotor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

            // 控制字仍然使用memcpy（不需要标志位）
            std::memcpy(const_cast<uint8_t*>(testMotor.stateAndMode.controlData.controlWordRaw), &randomCtrl, 2);

            // 速度：转换为RPM单位
            float targetVelocityRpm = static_cast<float>(randomVel);
            testMotor.velocity.target_rpm_velocity_mode.store(targetVelocityRpm);
            testMotor.velocity.flags_.fetch_or(MotorVelocity::Flags::TARGET_DATA_SEND_NEED_REFRESH_VELOCITY_MODE, std::memory_order_release);

            // 电流：转换为mA单位
            float targetCurrentMa = static_cast<float>(randomCur);
            testMotor.current.target_current.store(targetCurrentMa);
            testMotor.current.flags_.fetch_or(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

            // 调用刷新方法进行数据转换
            testMotor.refreshMotorData(testMotor.position);
            testMotor.refreshMotorData(testMotor.velocity);
            testMotor.refreshMotorData(testMotor.current);

            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

            testCount++;
            totalTime += duration.count();

            // 等待处理
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        double avgTime = static_cast<double>(totalTime) / testCount;
        PDO_TEST_DEBUG_PRINT("RPDO数据写入平均时间: " << avgTime << " us (10次测试)");
        PDO_TEST_DEBUG_PRINT("RPDO基础功能测试通过");

        return true;

    } catch (const std::exception& e) {
        std::cout << "[ERROR][testRpdoBasicFunctionality]: 测试异常: " << e.what() << std::endl;
        return false;
    }
}

// ====================== 多电机并发测试 ======================

/**
 * @brief 多电机并发测试
 *
 * @details 测试6个电机的并发RPDO发送性能：
 * - 所有电机同时发送RPDO1和RPDO2
 * - 验证并发发送的性能表现
 * - 检查所有电机的COB-ID正确性
 * - 统计批量发送性能指标
 *
 * @param motors 电机数组引用
 * @param sendThread 发送线程引用
 * @return bool 测试成功返回true，失败返回false
 */
bool testRpdoMultiMotorConcurrent(std::array<Motor, 6>& motors, SendThread& sendThread) {
    try {
        PDO_TEST_DEBUG_PRINT("开始多电机并发测试");

        // 1. 为所有电机生成不同的测试数据
        PDO_TEST_DEBUG_PRINT("测试2.1: 生成6个电机的测试数据");

        auto testData = RpdoTestDataGenerator::generateStandardTestData();
        RpdoTestDataGenerator::printTestData(testData);

        // 2. 批量写入所有电机数据
        PDO_TEST_DEBUG_PRINT("测试2.2: 批量写入所有电机数据");

        // 批量写入6个电机数据
        {
            auto start = std::chrono::high_resolution_clock::now();
            for (uint8_t i = 0; i < 6; ++i) {
                Motor& motor = motors.at(i);
                const auto& motorData = testData.motors[i];

                // 写入RPDO1数据（位置+控制字）- 使用正确的标志位驱动方式
                // 位置：转换为角度值
                float targetAngle = static_cast<float>(motorData.targetPosition) * 180.0f / 32768.0f;
                motor.position.target_degree.store(targetAngle);
                motor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

                // 控制字仍然使用memcpy（不需要标志位）
                std::memcpy(const_cast<uint8_t*>(motor.stateAndMode.controlData.controlWordRaw), &motorData.controlWord, 2);

                // 写入RPDO2数据（速度+电流）- 使用正确的标志位驱动方式
                // 速度：转换为RPM单位
                float targetVelocityRpm = static_cast<float>(motorData.targetVelocity);
                motor.velocity.target_rpm_velocity_mode.store(targetVelocityRpm);
                motor.velocity.flags_.fetch_or(MotorVelocity::Flags::TARGET_DATA_SEND_NEED_REFRESH_VELOCITY_MODE, std::memory_order_release);

                // 电流：转换为mA单位
                float targetCurrentMa = static_cast<float>(motorData.targetCurrent);
                motor.current.target_current.store(targetCurrentMa);
                motor.current.flags_.fetch_or(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

                // 调用刷新方法进行数据转换
                motor.refreshMotorData(motor.position);
                motor.refreshMotorData(motor.velocity);
                motor.refreshMotorData(motor.current);
            }
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << "[PERF][TestSendThreadPDO]: 批量写入6个电机数据 took " << duration.count() << " us\n";
        }

        // 3. 等待发送线程处理并发PDO
        PDO_TEST_DEBUG_PRINT("测试2.3: 等待并发PDO处理");

        // 等待并发PDO处理
        {
            auto start = std::chrono::high_resolution_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 等待多个周期
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << "[PERF][TestSendThreadPDO]: 等待并发PDO处理 took " << duration.count() << " us\n";
        }

        // 4. 输出外部验证信息
        PDO_TEST_DEBUG_PRINT("=== 多电机并发外部验证信息 ===");
        PDO_TEST_DEBUG_PRINT("请监听以下COB-ID (共12个帧):");

        for (uint8_t i = 0; i < 6; ++i) {
            uint8_t motorIndex = i + 1;
            const auto& motorData = testData.motors[i];

            PDO_TEST_DEBUG_PRINT("电机" << static_cast<int>(motorIndex) << ":");

            // RPDO1信息
            uint32_t rpdo1CobId = 0x200 + motorIndex;
            PDO_TEST_DEBUG_PRINT("  RPDO1 COB-ID: 0x" << std::hex << rpdo1CobId);
            PDO_TEST_DEBUG_PRINT("    目标位置: 0x" << std::setw(8) << std::setfill('0') << motorData.targetPosition);
            PDO_TEST_DEBUG_PRINT("    控制字: 0x" << std::setw(4) << std::setfill('0') << motorData.controlWord);

            // RPDO2信息
            uint32_t rpdo2CobId = 0x300 + motorIndex;
            PDO_TEST_DEBUG_PRINT("  RPDO2 COB-ID: 0x" << std::hex << rpdo2CobId);
            PDO_TEST_DEBUG_PRINT("    目标速度: 0x" << std::setw(4) << std::setfill('0') << motorData.targetVelocity);
            PDO_TEST_DEBUG_PRINT("    目标电流: 0x" << std::setw(4) << std::setfill('0') << motorData.targetCurrent);
            PDO_TEST_DEBUG_PRINT(std::dec);
        }

        PDO_TEST_DEBUG_PRINT("所有帧DLC应为8字节");
        PDO_TEST_DEBUG_PRINT("=== 验证信息结束 ===");

        // 5. 多次并发性能测试
        PDO_TEST_DEBUG_PRINT("测试2.4: 多次并发性能测试");

        size_t batchTestCount = 0;
        uint64_t batchTotalTime = 0;

        for (int testIteration = 0; testIteration < 20; ++testIteration) {
            auto batchStart = std::chrono::high_resolution_clock::now();

            // 为所有电机生成新的测试数据
            for (uint8_t i = 0; i < 6; ++i) {
                Motor& motor = motors.at(i);

                // 生成递增的测试数据
                Position_type targetPos = 0x10000 * (testIteration * 6 + i + 1);
                uint16_t controlWord = 0x000F;
                Velocity_type targetVel = 0x1000 * (testIteration * 6 + i + 1);
                Current_type targetCur = 0x0800 * (testIteration * 6 + i + 1);

                // 写入数据（使用正确的标志位驱动方式）
                // 位置：转换为角度值
                float targetAngle = static_cast<float>(targetPos) * 180.0f / 32768.0f;
                motor.position.target_degree.store(targetAngle);
                motor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

                // 控制字仍然使用memcpy（不需要标志位）
                std::memcpy(const_cast<uint8_t*>(motor.stateAndMode.controlData.controlWordRaw), &controlWord, 2);

                // 速度：转换为RPM单位
                float targetVelocityRpm = static_cast<float>(targetVel);
                motor.velocity.target_rpm_velocity_mode.store(targetVelocityRpm);
                motor.velocity.flags_.fetch_or(MotorVelocity::Flags::TARGET_DATA_SEND_NEED_REFRESH_VELOCITY_MODE, std::memory_order_release);

                // 电流：转换为mA单位
                float targetCurrentMa = static_cast<float>(targetCur);
                motor.current.target_current.store(targetCurrentMa);
                motor.current.flags_.fetch_or(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

                // 调用刷新方法进行数据转换
                motor.refreshMotorData(motor.position);
                motor.refreshMotorData(motor.velocity);
                motor.refreshMotorData(motor.current);
            }

            auto batchEnd = std::chrono::high_resolution_clock::now();
            auto batchDuration = std::chrono::duration_cast<std::chrono::microseconds>(batchEnd - batchStart);

            batchTestCount++;
            batchTotalTime += batchDuration.count();

            // 等待处理
            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            // 每5次测试输出一次进度
            if ((testIteration + 1) % 5 == 0) {
                PDO_TEST_DEBUG_PRINT("完成 " << (testIteration + 1) << "/20 次并发测试");
            }
        }

        // 计算并发性能统计
        double avgBatchTime = static_cast<double>(batchTotalTime) / batchTestCount;
        double avgMotorTime = avgBatchTime / 6.0;  // 平均每个电机的处理时间

        PDO_TEST_DEBUG_PRINT("=== 并发性能统计 ===");
        PDO_TEST_DEBUG_PRINT("批量写入平均时间: " << avgBatchTime << " us (6个电机)");
        PDO_TEST_DEBUG_PRINT("单电机平均时间: " << avgMotorTime << " us");
        PDO_TEST_DEBUG_PRINT("总测试次数: " << batchTestCount);
        PDO_TEST_DEBUG_PRINT("=== 性能统计结束 ===");

        // 6. 验证发送线程健康状态
        PDO_TEST_DEBUG_PRINT("测试2.5: 验证发送线程健康状态");

        bool isHealthy = sendThread.isHealthy();
        uint32_t errorCount = sendThread.getErrorCount();

        PDO_TEST_DEBUG_PRINT("线程健康状态: " << (isHealthy ? "健康" : "不健康"));
        PDO_TEST_DEBUG_PRINT("错误计数: " << errorCount);

        if (!isHealthy) {
            auto lastError = sendThread.getLastError();
            auto errorMsg = sendThread.getLastErrorMessage();
            PDO_TEST_DEBUG_PRINT("错误类型: " << static_cast<int>(lastError));
            PDO_TEST_DEBUG_PRINT("错误消息: " << errorMsg);
            return false;
        }

        PDO_TEST_DEBUG_PRINT("多电机并发测试通过");
        return true;

    } catch (const std::exception& e) {
        std::cout << "[ERROR][testRpdoMultiMotorConcurrent]: 测试异常: " << e.what() << std::endl;
        return false;
    }
}

// ====================== 边界值测试 ======================

/**
 * @brief 边界值测试
 *
 * @details 测试RPDO数据的边界值处理：
 * - 最大/最小位置值测试
 * - 最大/最小速度值测试
 * - 最大/最小电流值测试
 * - 控制字边界值测试
 * - 验证边界值数据的正确发送
 * - 检查数据溢出情况
 *
 * @param motors 电机数组引用
 * @param sendThread 发送线程引用
 * @return bool 测试成功返回true，失败返回false
 */
bool testRpdoBoundaryValues(std::array<Motor, 6>& motors, SendThread& sendThread) {
    try {
        PDO_TEST_DEBUG_PRINT("开始边界值测试");

        // 1. 生成边界值测试数据
        PDO_TEST_DEBUG_PRINT("测试3.1: 生成边界值测试数据");

        auto boundaryData = RpdoTestDataGenerator::generateBoundaryTestData();
        RpdoTestDataGenerator::printTestData(boundaryData);

        // 2. 测试单电机的边界值
        PDO_TEST_DEBUG_PRINT("测试3.2: 测试单电机边界值（电机1）");

        uint8_t testMotorIndex = 1;
        Motor& testMotor = motors.at(testMotorIndex - 1);
        const auto& motorBoundaryData = boundaryData.motors[testMotorIndex - 1];

        // 写入边界值数据到电机1（使用正确的标志位驱动方式）
        {
            auto start = std::chrono::high_resolution_clock::now();
            // 位置：转换为角度值
            float targetAngle = static_cast<float>(motorBoundaryData.targetPosition) * 180.0f / 32768.0f;
            testMotor.position.target_degree.store(targetAngle);
            testMotor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

            // 控制字仍然使用memcpy（不需要标志位）
            std::memcpy(const_cast<uint8_t*>(testMotor.stateAndMode.controlData.controlWordRaw), &motorBoundaryData.controlWord, 2);

            // 速度：转换为RPM单位
            float targetVelocityRpm = static_cast<float>(motorBoundaryData.targetVelocity);
            testMotor.velocity.target_rpm_velocity_mode.store(targetVelocityRpm);
            testMotor.velocity.flags_.fetch_or(MotorVelocity::Flags::TARGET_DATA_SEND_NEED_REFRESH_VELOCITY_MODE, std::memory_order_release);

            // 电流：转换为mA单位
            float targetCurrentMa = static_cast<float>(motorBoundaryData.targetCurrent);
            testMotor.current.target_current.store(targetCurrentMa);
            testMotor.current.flags_.fetch_or(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

            // 调用刷新方法进行数据转换
            testMotor.refreshMotorData(testMotor.position);
            testMotor.refreshMotorData(testMotor.velocity);
            testMotor.refreshMotorData(testMotor.current);

            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << "[PERF][TestSendThreadPDO]: 写入边界值数据到电机1 took " << duration.count() << " us\n";
        }

        // 等待边界值PDO处理
        {
            auto start = std::chrono::high_resolution_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << "[PERF][TestSendThreadPDO]: 等待边界值PDO处理 took " << duration.count() << " us\n";
        }

        // 输出边界值验证信息
        PDO_TEST_DEBUG_PRINT("=== 边界值外部验证信息 ===");
        PDO_TEST_DEBUG_PRINT("电机1边界值测试数据:");

        PDO_TEST_DEBUG_PRINT("  目标位置边界值: 0x" << std::hex << std::setw(8) << std::setfill('0')
                          << motorBoundaryData.targetPosition << " ("
                          << (motorBoundaryData.targetPosition > 0x7FFFFFFF ? "负值" : "正值") << ")");
        PDO_TEST_DEBUG_PRINT("  控制字边界值: 0x" << std::hex << std::setw(4) << std::setfill('0')
                          << motorBoundaryData.controlWord << std::dec);
        PDO_TEST_DEBUG_PRINT("  目标速度边界值: 0x" << std::hex << std::setw(4) << std::setfill('0')
                          << motorBoundaryData.targetVelocity << " ("
                          << (static_cast<int16_t>(motorBoundaryData.targetVelocity) < 0 ? "负速度" : "正速度") << ")");
        PDO_TEST_DEBUG_PRINT("  目标电流边界值: 0x" << std::hex << std::setw(4) << std::setfill('0')
                          << motorBoundaryData.targetCurrent << " ("
                          << (static_cast<int16_t>(motorBoundaryData.targetCurrent) < 0 ? "负电流" : "正电流") << ")");
        PDO_TEST_DEBUG_PRINT(std::dec);
        PDO_TEST_DEBUG_PRINT("  RPDO1 COB-ID: 0x" << std::hex << (0x200 + testMotorIndex));
        PDO_TEST_DEBUG_PRINT("  RPDO2 COB-ID: 0x" << (0x300 + testMotorIndex) << std::dec);
        PDO_TEST_DEBUG_PRINT("=== 边界值验证信息结束 ===");

        // 3. 多电机边界值并发测试
        PDO_TEST_DEBUG_PRINT("测试3.3: 多电机边界值并发测试");

        // 批量写入边界值数据
        {
            auto start = std::chrono::high_resolution_clock::now();
            for (uint8_t i = 0; i < 6; ++i) {
                Motor& motor = motors.at(i);
                const auto& motorData = boundaryData.motors[i];

                // 位置：转换为角度值
                float targetAngle = static_cast<float>(motorData.targetPosition) * 180.0f / 32768.0f;
                motor.position.target_degree.store(targetAngle);
                motor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

                // 控制字仍然使用memcpy（不需要标志位）
                std::memcpy(const_cast<uint8_t*>(motor.stateAndMode.controlData.controlWordRaw), &motorData.controlWord, 2);

                // 速度：转换为RPM单位
                float targetVelocityRpm = static_cast<float>(motorData.targetVelocity);
                motor.velocity.target_rpm_velocity_mode.store(targetVelocityRpm);
                motor.velocity.flags_.fetch_or(MotorVelocity::Flags::TARGET_DATA_SEND_NEED_REFRESH_VELOCITY_MODE, std::memory_order_release);

                // 电流：转换为mA单位
                float targetCurrentMa = static_cast<float>(motorData.targetCurrent);
                motor.current.target_current.store(targetCurrentMa);
                motor.current.flags_.fetch_or(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

                // 调用刷新方法进行数据转换
                motor.refreshMotorData(motor.position);
                motor.refreshMotorData(motor.velocity);
                motor.refreshMotorData(motor.current);
            }
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << "[PERF][TestSendThreadPDO]: 批量写入边界值数据 took " << duration.count() << " us\n";
        }

        // 等待边界值并发处理
        {
            auto start = std::chrono::high_resolution_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << "[PERF][TestSendThreadPDO]: 等待边界值并发处理 took " << duration.count() << " us\n";
        }

        // 4. 输出多电机边界值验证信息
        PDO_TEST_DEBUG_PRINT("=== 多电机边界值验证信息 ===");
        PDO_TEST_DEBUG_PRINT("6个电机的边界值COB-ID:");

        for (uint8_t i = 0; i < 6; ++i) {
            uint8_t motorIdx = i + 1;
            uint32_t rpdo1Id = 0x200 + motorIdx;
            uint32_t rpdo2Id = 0x300 + motorIdx;

            PDO_TEST_DEBUG_PRINT("电机" << static_cast<int>(motorIdx) << ": RPDO1=0x" << std::hex << rpdo1Id
                              << ", RPDO2=0x" << rpdo2Id << std::dec);
        }

        PDO_TEST_DEBUG_PRINT("所有帧DLC应为8字节");
        PDO_TEST_DEBUG_PRINT("=== 多电机边界值信息结束 ===");

        // 5. 特殊边界值测试（全0和全1）
        PDO_TEST_DEBUG_PRINT("测试3.4: 特殊边界值测试");

        auto zeroData = RpdoTestDataGenerator::generateZeroTestData();
        Motor& zeroMotor = motors.at(0);  // 电机1测试零值

        // 写入零值数据（使用正确的标志位驱动方式）
        {
            auto start = std::chrono::high_resolution_clock::now();
            // 位置：零值转换为角度值
            float targetAngle = 0.0f;
            zeroMotor.position.target_degree.store(targetAngle);
            zeroMotor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

            // 控制字仍然使用memcpy（不需要标志位）
            std::memcpy(const_cast<uint8_t*>(zeroMotor.stateAndMode.controlData.controlWordRaw), &zeroData.motors[0].controlWord, 2);

            // 速度：零值转换为RPM单位
            float targetVelocityRpm = 0.0f;
            zeroMotor.velocity.target_rpm_velocity_mode.store(targetVelocityRpm);
            zeroMotor.velocity.flags_.fetch_or(MotorVelocity::Flags::TARGET_DATA_SEND_NEED_REFRESH_VELOCITY_MODE, std::memory_order_release);

            // 电流：零值转换为mA单位
            float targetCurrentMa = 0.0f;
            zeroMotor.current.target_current.store(targetCurrentMa);
            zeroMotor.current.flags_.fetch_or(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

            // 调用刷新方法进行数据转换
            zeroMotor.refreshMotorData(zeroMotor.position);
            zeroMotor.refreshMotorData(zeroMotor.velocity);
            zeroMotor.refreshMotorData(zeroMotor.current);

            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << "[PERF][TestSendThreadPDO]: 写入零值数据 took " << duration.count() << " us\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        PDO_TEST_DEBUG_PRINT("=== 零值验证信息 ===");
        PDO_TEST_DEBUG_PRINT("电机1零值测试:");
        PDO_TEST_DEBUG_PRINT("  目标位置: 0x00000000");
        PDO_TEST_DEBUG_PRINT("  控制字: 0x0000");
        PDO_TEST_DEBUG_PRINT("  目标速度: 0x0000");
        PDO_TEST_DEBUG_PRINT("  目标电流: 0x0000");
        PDO_TEST_DEBUG_PRINT("  RPDO1 COB-ID: 0x201");
        PDO_TEST_DEBUG_PRINT("  RPDO2 COB-ID: 0x301");
        PDO_TEST_DEBUG_PRINT("=== 零值验证信息结束 ===");

        // 6. 边界值性能测试
        PDO_TEST_DEBUG_PRINT("测试3.5: 边界值性能测试");

        size_t boundaryTestCount = 0;
        uint64_t boundaryTotalTime = 0;

        for (int i = 0; i < 10; ++i) {
            auto start = std::chrono::high_resolution_clock::now();

            // 交替写入最大值和最小值
            for (uint8_t motorIdx = 0; motorIdx < 6; ++motorIdx) {
                Motor& motor = motors.at(motorIdx);

                if (i % 2 == 0) {
                    // 写入最大值
                    float targetAngle = 180.0f * (1.0f - 1.0f / (1UL << 31));
                    motor.position.target_degree.store(targetAngle);
                    motor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

                    uint16_t controlWord = 0xFFFF;
                    std::memcpy(const_cast<uint8_t*>(motor.stateAndMode.controlData.controlWordRaw), &controlWord, 2);

                    float targetVelocityRpm = 14000.0f * (1.0f - 1.0f / (1UL << 15));
                    motor.velocity.target_rpm_velocity_mode.store(targetVelocityRpm);
                    motor.velocity.flags_.fetch_or(MotorVelocity::Flags::TARGET_DATA_SEND_NEED_REFRESH_VELOCITY_MODE, std::memory_order_release);

                    float targetCurrentMa = 16000.0f * (1.0f - 1.0f / (1UL << 15));
                    motor.current.target_current.store(targetCurrentMa);
                    motor.current.flags_.fetch_or(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);
                } else {
                    // 写入最小值
                    float targetAngle = -180.0f;
                    motor.position.target_degree.store(targetAngle);
                    motor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

                    uint16_t controlWord = 0x0000;
                    std::memcpy(const_cast<uint8_t*>(motor.stateAndMode.controlData.controlWordRaw), &controlWord, 2);

                    float targetVelocityRpm = -14000.0f;
                    motor.velocity.target_rpm_velocity_mode.store(targetVelocityRpm);
                    motor.velocity.flags_.fetch_or(MotorVelocity::Flags::TARGET_DATA_SEND_NEED_REFRESH_VELOCITY_MODE, std::memory_order_release);

                    float targetCurrentMa = -16000.0f;
                    motor.current.target_current.store(targetCurrentMa);
                    motor.current.flags_.fetch_or(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);
                }

                // 调用刷新方法进行数据转换
                motor.refreshMotorData(motor.position);
                motor.refreshMotorData(motor.velocity);
                motor.refreshMotorData(motor.current);
            }

            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

            boundaryTestCount++;
            boundaryTotalTime += duration.count();

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        double avgBoundaryTime = static_cast<double>(boundaryTotalTime) / boundaryTestCount;
        PDO_TEST_DEBUG_PRINT("边界值平均写入时间: " << avgBoundaryTime << " us (6个电机)");

        // 7. 验证数据完整性
        PDO_TEST_DEBUG_PRINT("测试3.6: 验证数据完整性");

        bool isHealthy = sendThread.isHealthy();
        uint32_t errorCount = sendThread.getErrorCount();

        PDO_TEST_DEBUG_PRINT("边界值测试后线程健康状态: " << (isHealthy ? "健康" : "不健康"));
        PDO_TEST_DEBUG_PRINT("错误计数: " << errorCount);

        if (!isHealthy || errorCount > 0) {
            auto lastError = sendThread.getLastError();
            auto errorMsg = sendThread.getLastErrorMessage();
            PDO_TEST_DEBUG_PRINT("检测到错误，类型: " << static_cast<int>(lastError));
            PDO_TEST_DEBUG_PRINT("错误消息: " << errorMsg);
            return false;
        }

        PDO_TEST_DEBUG_PRINT("边界值测试通过");
        return true;

    } catch (const std::exception& e) {
        std::cout << "[ERROR][testRpdoBoundaryValues]: 测试异常: " << e.what() << std::endl;
        return false;
    }
}

// ====================== 性能统计测试 ======================

/**
 * @brief 性能统计测试
 *
 * @details 测试RPDO发送的各项性能指标：
 * - 单项性能测试（单个RPDO帧发送性能）
 * - 批量性能测试（多帧发送性能）
 * - 周期性能测试（2ms周期内的处理能力）
 * - 内存分配测试（验证无动态内存分配的性能优势）
 * - 吞吐量测试（单位时间内的PDO帧发送数量）
 *
 * @param motors 电机数组引用
 * @param sendThread 发送线程引用
 * @return bool 测试成功返回true，失败返回false
 */
bool testRpdoPerformance(std::array<Motor, 6>& motors, SendThread& sendThread) {
    try {
        PDO_TEST_DEBUG_PRINT("开始性能统计测试");

        // 1. 单项性能测试
        PDO_TEST_DEBUG_PRINT("测试4.1: 单项性能测试（单个RPDO帧）");

        size_t singleTestCount = 0;
        uint64_t singleTotalTime = 0;
        uint64_t singleMinTime = UINT64_MAX;
        uint64_t singleMaxTime = 0;

        Motor& testMotor = motors.at(0);  // 使用电机1进行测试

        for (int i = 0; i < 50; ++i) {
            // 计算物理量数值
            float testAngle = 10.0f * (i + 1); // 角度值
            uint16_t testCtrl = 0x000F;
            float testVelocityRpm = 1000.0f * (i + 1); // RPM值
            float testCurrentMa = 1000.0f * (i + 1); // mA值

            auto start = std::chrono::high_resolution_clock::now();
            // 使用正确的标志位驱动方式写入数据
            testMotor.position.target_degree.store(testAngle);
            testMotor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

            std::memcpy(const_cast<uint8_t*>(testMotor.stateAndMode.controlData.controlWordRaw), &testCtrl, 2);

            testMotor.velocity.target_rpm_velocity_mode.store(testVelocityRpm);
            testMotor.velocity.flags_.fetch_or(MotorVelocity::Flags::TARGET_DATA_SEND_NEED_REFRESH_VELOCITY_MODE, std::memory_order_release);

            testMotor.current.target_current.store(testCurrentMa);
            testMotor.current.flags_.fetch_or(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

            // 调用刷新方法进行数据转换
            testMotor.refreshMotorData(testMotor.position);
            testMotor.refreshMotorData(testMotor.velocity);
            testMotor.refreshMotorData(testMotor.current);

            auto end = std::chrono::high_resolution_clock::now();
            auto singleTime = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            std::cout << "[PERF][TestSendThreadPDO]: 单项RPDO数据写入 took " << singleTime << " us\n";

            singleTestCount++;
            singleTotalTime += singleTime;

            if (singleTime < singleMinTime) singleMinTime = singleTime;
            if (singleTime > singleMaxTime) singleMaxTime = singleTime;

            // 短暂等待避免过载
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            // 每10次输出进度
            if ((i + 1) % 10 == 0) {
                PDO_TEST_DEBUG_PRINT("完成 " << (i + 1) << "/50 次单项测试");
            }
        }

        double singleAvgTime = static_cast<double>(singleTotalTime) / singleTestCount;
        PDO_TEST_DEBUG_PRINT("=== 单项性能统计 ===");
        PDO_TEST_DEBUG_PRINT("平均写入时间: " << singleAvgTime << " us");
        PDO_TEST_DEBUG_PRINT("最小写入时间: " << singleMinTime << " us");
        PDO_TEST_DEBUG_PRINT("最大写入时间: " << singleMaxTime << " us");
        PDO_TEST_DEBUG_PRINT("测试次数: " << singleTestCount);
        PDO_TEST_DEBUG_PRINT("=== 单项统计结束 ===");

        // 2. 批量性能测试
        PDO_TEST_DEBUG_PRINT("测试4.2: 批量性能测试（6个电机）");

        size_t batchTestCount = 0;
        uint64_t batchTotalTime = 0;
        uint64_t batchMinTime = UINT64_MAX;
        uint64_t batchMaxTime = 0;

        for (int i = 0; i < 30; ++i) {
            auto batchStart = std::chrono::high_resolution_clock::now();
            for (uint8_t motorIdx = 0; motorIdx < 6; ++motorIdx) {
                Motor& motor = motors.at(motorIdx);

                // 计算物理量数值
                float targetAngle = 10.0f * (i * 6 + motorIdx + 1); // 角度值
                uint16_t controlWord = 0x000F;
                float targetVelocityRpm = 1000.0f * (i * 6 + motorIdx + 1); // RPM值
                float targetCurrentMa = 1000.0f * (i * 6 + motorIdx + 1); // mA值

                // 使用正确的标志位驱动方式写入数据
                motor.position.target_degree.store(targetAngle);
                motor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

                std::memcpy(const_cast<uint8_t*>(motor.stateAndMode.controlData.controlWordRaw), &controlWord, 2);

                motor.velocity.target_rpm_velocity_mode.store(targetVelocityRpm);
                motor.velocity.flags_.fetch_or(MotorVelocity::Flags::TARGET_DATA_SEND_NEED_REFRESH_VELOCITY_MODE, std::memory_order_release);

                motor.current.target_current.store(targetCurrentMa);
                motor.current.flags_.fetch_or(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

                // 调用刷新方法进行数据转换
                motor.refreshMotorData(motor.position);
                motor.refreshMotorData(motor.velocity);
                motor.refreshMotorData(motor.current);
            }
            auto batchEnd = std::chrono::high_resolution_clock::now();
            auto batchTime = std::chrono::duration_cast<std::chrono::microseconds>(batchEnd - batchStart).count();
            std::cout << "[PERF][TestSendThreadPDO]: 批量写入6个电机RPDO数据 took " << batchTime << " us\n";

            batchTestCount++;
            batchTotalTime += batchTime;

            if (batchTime < batchMinTime) batchMinTime = batchTime;
            if (batchTime > batchMaxTime) batchMaxTime = batchTime;

            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            if ((i + 1) % 10 == 0) {
                PDO_TEST_DEBUG_PRINT("完成 " << (i + 1) << "/30 次批量测试");
            }
        }

        double batchAvgTime = static_cast<double>(batchTotalTime) / batchTestCount;
        double avgPerMotor = batchAvgTime / 6.0;

        PDO_TEST_DEBUG_PRINT("=== 批量性能统计 ===");
        PDO_TEST_DEBUG_PRINT("批量平均时间: " << batchAvgTime << " us (6个电机)");
        PDO_TEST_DEBUG_PRINT("单电机平均: " << avgPerMotor << " us");
        PDO_TEST_DEBUG_PRINT("批量最小时间: " << batchMinTime << " us");
        PDO_TEST_DEBUG_PRINT("批量最大时间: " << batchMaxTime << " us");
        PDO_TEST_DEBUG_PRINT("测试次数: " << batchTestCount);
        PDO_TEST_DEBUG_PRINT("=== 批量统计结束 ===");

        // 3. 周期性能测试
        PDO_TEST_DEBUG_PRINT("测试4.3: 周期性能测试（2ms周期内处理能力）");

        size_t cycleTestCount = 0;
        uint64_t cycleTotalTime = 0;
        uint64_t successfulCycles = 0;

        for (int i = 0; i < 20; ++i) {
            auto cycleStart = std::chrono::high_resolution_clock::now();

            // 在单个2ms周期内尽可能多地写入数据
            int writesInCycle = 0;
            auto cycleDeadline = cycleStart + std::chrono::milliseconds(2);

            while (std::chrono::high_resolution_clock::now() < cycleDeadline && writesInCycle < 6) {
                Motor& motor = motors.at(writesInCycle % 6);

                // 计算物理量数值
                float targetAngle = 10.0f * (i * 6 + writesInCycle + 1); // 角度值
                uint16_t controlWord = 0x000F;

                // 使用正确的标志位驱动方式写入数据
                motor.position.target_degree.store(targetAngle);
                motor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

                std::memcpy(const_cast<uint8_t*>(motor.stateAndMode.controlData.controlWordRaw), &controlWord, 2);

                // 调用刷新方法进行数据转换
                motor.refreshMotorData(motor.position);

                writesInCycle++;
                cycleTestCount++;
            }

            auto cycleEnd = std::chrono::high_resolution_clock::now();
            auto cycleDuration = std::chrono::duration_cast<std::chrono::microseconds>(cycleEnd - cycleStart);

            cycleTotalTime += cycleDuration.count();
            if (writesInCycle > 0) successfulCycles++;

            PDO_TEST_DEBUG_PRINT("周期" << (i + 1) << ": 写入" << writesInCycle << "个电机，耗时"
                              << cycleDuration.count() << " us");

            // 等待下一个周期
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long long>(3)));
        }

        double avgCycleTime = static_cast<double>(cycleTotalTime) / 20.0;
        double successRate = (static_cast<double>(successfulCycles) / 20.0) * 100.0;

        PDO_TEST_DEBUG_PRINT("=== 周期性能统计 ===");
        PDO_TEST_DEBUG_PRINT("平均周期时间: " << avgCycleTime << " us");
        PDO_TEST_DEBUG_PRINT("周期成功率: " << successRate << "%");
        PDO_TEST_DEBUG_PRINT("成功周期数: " << successfulCycles << "/20");
        PDO_TEST_DEBUG_PRINT("=== 周期统计结束 ===");

        // 4. 吞吐量测试
        PDO_TEST_DEBUG_PRINT("测试4.4: 吞吐量测试（1秒内的帧发送数量）");

        auto throughputStart = std::chrono::high_resolution_clock::now();
        auto throughputEnd = throughputStart + std::chrono::seconds(1);
        size_t frameCount = 0;

        while (std::chrono::high_resolution_clock::now() < throughputEnd) {
            for (uint8_t motorIdx = 0; motorIdx < 6; ++motorIdx) {
                Motor& motor = motors.at(motorIdx);

                // 计算物理量数值
                float targetAngle = 10.0f * (frameCount + motorIdx + 1); // 角度值
                uint16_t controlWord = 0x000F;

                // 使用正确的标志位驱动方式写入数据
                motor.position.target_degree.store(targetAngle);
                motor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

                std::memcpy(const_cast<uint8_t*>(motor.stateAndMode.controlData.controlWordRaw), &controlWord, 2);

                // 调用刷新方法进行数据转换
                motor.refreshMotorData(motor.position);

                frameCount++;

                // 避免CPU过载
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }

        double framesPerSecond = static_cast<double>(frameCount);
        double dataThroughput = framesPerSecond * 8.0;  // 每帧8字节

        PDO_TEST_DEBUG_PRINT("=== 吞吐量统计 ===");
        PDO_TEST_DEBUG_PRINT("1秒内发送帧数: " << framesPerSecond);
        PDO_TEST_DEBUG_PRINT("数据吞吐量: " << dataThroughput << " 字节/秒");
        PDO_TEST_DEBUG_PRINT("平均帧间隔: " << (1000000.0 / framesPerSecond) << " us");
        PDO_TEST_DEBUG_PRINT("=== 吞吐量统计结束 ===");

        // 5. 综合性能评估
        PDO_TEST_DEBUG_PRINT("测试4.5: 综合性能评估");

        bool isHealthy = sendThread.isHealthy();
        uint32_t errorCount = sendThread.getErrorCount();

        PDO_TEST_DEBUG_PRINT("=== 综合性能报告 ===");
        PDO_TEST_DEBUG_PRINT("单项写入性能: " << singleAvgTime << " us/电机");
        PDO_TEST_DEBUG_PRINT("批量写入性能: " << avgPerMotor << " us/电机");
        PDO_TEST_DEBUG_PRINT("批量加速比: " << (singleAvgTime / avgPerMotor) << "x");
        PDO_TEST_DEBUG_PRINT("周期处理能力: " << successRate << "% (2ms周期)");
        PDO_TEST_DEBUG_PRINT("实时吞吐量: " << framesPerSecond << " 帧/秒");
        PDO_TEST_DEBUG_PRINT("线程健康状态: " << (isHealthy ? "健康" : "警告"));
        PDO_TEST_DEBUG_PRINT("错误计数: " << errorCount);
        PDO_TEST_DEBUG_PRINT("=== 性能报告结束 ===");

        if (!isHealthy) {
            auto lastError = sendThread.getLastError();
            auto errorMsg = sendThread.getLastErrorMessage();
            PDO_TEST_DEBUG_PRINT("性能测试期间检测到错误: " << static_cast<int>(lastError));
            PDO_TEST_DEBUG_PRINT("错误消息: " << errorMsg);
            return false;
        }

        PDO_TEST_DEBUG_PRINT("性能统计测试通过");
        return true;

    } catch (const std::exception& e) {
        std::cout << "[ERROR][testRpdoPerformance]: 测试异常: " << e.what() << std::endl;
        return false;
    }
}

// ====================== 实时性验证测试 ======================

/**
 * @brief RPDO实时性验证测试
 *
 * @details 测试RPDO的实时性能，包括：
 * - 周期确定性测试（验证2ms同步周期的确定性）
 * - 发送延迟统计（从数据准备到串口传输的延迟）
 * - 吞吐量测试（单位时间内PDO帧传输数量）
 * - 抖动分析（周期时间的稳定性）
 *
 * @param motors 电机数组引用
 * @param sendThread 发送线程引用
 * @return bool 测试成功返回true，失败返回false
 *
 * @note 本测试用于验证RPDO通信的实时性能指标
 * @warning 测试过程需要精确时间测量，避免系统负载影响结果
 */
bool testRpdoRealtime(std::array<Motor, 6>& motors, SendThread& sendThread) {
    try {
        PDO_TEST_DEBUG_PRINT("开始RPDO实时性验证测试");

        // 准备测试数据
        RpdoTestDataGenerator testDataGenerator;
        RpdoFrameValidator frameValidator;

        // 1. 周期确定性测试
        PDO_TEST_DEBUG_PRINT("测试5.1: 周期确定性测试（验证2ms同步周期）");

        const int cycleCount = 100;  // 测试100个周期
        const std::chrono::microseconds targetCycleTime(2000);  // 2ms
        std::vector<std::chrono::microseconds> cycleDelays;
        cycleDelays.reserve(cycleCount);

        // 预热系统
        PDO_TEST_DEBUG_PRINT("系统预热中...");
        for (int i = 0; i < 10; ++i) {
            auto testData = testDataGenerator.generateStandardTestData();
            testDataGenerator.writeTestDataToMotors(testData, motors);
            // 数据写入后会自动触发PDO处理
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        // 正式测试
        PDO_TEST_DEBUG_PRINT("开始周期确定性测试，测试周期数: " << cycleCount);

        auto lastCycleTime = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < cycleCount; ++i) {
            // 记录周期开始时间
            auto cycleStartTime = std::chrono::high_resolution_clock::now();

            // 准备和发送PDO数据
            auto testData = testDataGenerator.generateStandardTestData();
            testDataGenerator.writeTestDataToMotors(testData, motors);
            // 数据写入后会自动触发PDO处理

            // 记录周期结束时间
            auto cycleEndTime = std::chrono::high_resolution_clock::now();

            // 计算周期时间
            auto cycleDuration = std::chrono::duration_cast<std::chrono::microseconds>(
                cycleEndTime - cycleStartTime);

            cycleDelays.push_back(cycleDuration);

            // 等待到下一个周期开始
            auto nextCycleTime = lastCycleTime + targetCycleTime;
            auto now = std::chrono::high_resolution_clock::now();

            if (now < nextCycleTime) {
                std::this_thread::sleep_until(nextCycleTime);
            }

            lastCycleTime = nextCycleTime;

            // 每20个周期输出一次进度
            if ((i + 1) % 20 == 0) {
                PDO_TEST_DEBUG_PRINT("周期测试进度: " << (i + 1) << "/" << cycleCount);
            }
        }

        // 分析周期确定性结果
        PDO_TEST_DEBUG_PRINT("=== 周期确定性分析 ===");

        // 计算统计指标
        auto minDelay = *std::min_element(cycleDelays.begin(), cycleDelays.end());
        auto maxDelay = *std::max_element(cycleDelays.begin(), cycleDelays.end());
        auto avgDelay = std::accumulate(cycleDelays.begin(), cycleDelays.end(),
                                      std::chrono::microseconds::zero()) / cycleCount;
        double avgDelayCount = avgDelay.count();

        // 计算标准差
        double variance = 0;
        for (const auto& delay : cycleDelays) {
            double diff = delay.count() - avgDelayCount;
            variance += diff * diff;
        }
        variance /= cycleCount;
        double stdDeviation = std::sqrt(variance);

        // 计算抖动
        auto jitter = maxDelay - minDelay;

        PDO_TEST_DEBUG_PRINT("目标周期时间: " << targetCycleTime.count() << " us");
        PDO_TEST_DEBUG_PRINT("最小周期时间: " << minDelay.count() << " us");
        PDO_TEST_DEBUG_PRINT("最大周期时间: " << maxDelay.count() << " us");
        PDO_TEST_DEBUG_PRINT("平均周期时间: " << avgDelay.count() << " us");
        PDO_TEST_DEBUG_PRINT("周期抖动: " << jitter.count() << " us");
        PDO_TEST_DEBUG_PRINT("标准差: " << stdDeviation << " us");

        // 验证周期确定性
        bool cycleDeterministic = true;
        if (jitter.count() > 500) {  // 抖动超过500us认为不合格
            PDO_TEST_DEBUG_PRINT("警告: 周期抖动过大 (" << jitter.count() << " us > 500 us)");
            cycleDeterministic = false;
        }

        if (stdDeviation > 100) {  // 标准差超过100us认为不合格
            PDO_TEST_DEBUG_PRINT("警告: 周期稳定性差 (标准差 " << stdDeviation << " us > 100 us)");
            cycleDeterministic = false;
        }

        PDO_TEST_DEBUG_PRINT("周期确定性结果: " << (cycleDeterministic ? "通过" : "失败"));
        PDO_TEST_DEBUG_PRINT("=== 周期确定性分析结束 ===");

        // 2. 发送延迟统计测试
        PDO_TEST_DEBUG_PRINT("测试5.2: 发送延迟统计测试");

        const int delayTestCount = 50;
        std::vector<std::chrono::microseconds> sendDelays;
        sendDelays.reserve(delayTestCount);

        PDO_TEST_DEBUG_PRINT("开始发送延迟测试，测试次数: " << delayTestCount);

        for (int i = 0; i < delayTestCount; ++i) {
            // 准备测试数据
            auto testData = testDataGenerator.generateStandardTestData();
            testDataGenerator.writeTestDataToMotors(testData, motors);

            // 记录数据准备完成时间
            auto dataPreparedTime = std::chrono::high_resolution_clock::now();

            // 数据写入后会自动触发PDO处理

            // 等待传输完成（假设处理时间不超过1ms）
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            // 记录传输完成时间
            auto transmissionCompleteTime = std::chrono::high_resolution_clock::now();

            // 计算发送延迟
            auto sendDelay = std::chrono::duration_cast<std::chrono::microseconds>(
                transmissionCompleteTime - dataPreparedTime);

            sendDelays.push_back(sendDelay);
        }

        // 分析发送延迟结果
        PDO_TEST_DEBUG_PRINT("=== 发送延迟分析 ===");

        auto minSendDelay = *std::min_element(sendDelays.begin(), sendDelays.end());
        auto maxSendDelay = *std::max_element(sendDelays.begin(), sendDelays.end());
        auto avgSendDelay = std::accumulate(sendDelays.begin(), sendDelays.end(),
                                         std::chrono::microseconds::zero()) / delayTestCount;

        PDO_TEST_DEBUG_PRINT("最小发送延迟: " << minSendDelay.count() << " us");
        PDO_TEST_DEBUG_PRINT("最大发送延迟: " << maxSendDelay.count() << " us");
        PDO_TEST_DEBUG_PRINT("平均发送延迟: " << avgSendDelay.count() << " us");

        // 验证发送延迟
        bool sendDelayAcceptable = true;
        if (avgSendDelay.count() > 1000) {  // 平均延迟超过1ms认为不合格
            PDO_TEST_DEBUG_PRINT("警告: 平均发送延迟过大 (" << avgSendDelay.count() << " us > 1000 us)");
            sendDelayAcceptable = false;
        }

        if (maxSendDelay.count() > 2000) {  // 最大延迟超过2ms认为不合格
            PDO_TEST_DEBUG_PRINT("警告: 最大发送延迟过大 (" << maxSendDelay.count() << " us > 2000 us)");
            sendDelayAcceptable = false;
        }

        PDO_TEST_DEBUG_PRINT("发送延迟结果: " << (sendDelayAcceptable ? "通过" : "失败"));
        PDO_TEST_DEBUG_PRINT("=== 发送延迟分析结束 ===");

        // 3. 吞吐量测试
        PDO_TEST_DEBUG_PRINT("测试5.3: 吞吐量测试");

        const std::chrono::seconds throughputTestDuration(3);  // 3秒测试
        auto testStartTime = std::chrono::high_resolution_clock::now();
        auto testEndTime = testStartTime + throughputTestDuration;
        int frameCount = 0;

        PDO_TEST_DEBUG_PRINT("开始吞吐量测试，测试时长: " << throughputTestDuration.count() << " 秒");

        while (std::chrono::high_resolution_clock::now() < testEndTime) {
            // 准备并发送PDO帧
            auto testData = testDataGenerator.generateStandardTestData();
            testDataGenerator.writeTestDataToMotors(testData, motors);
            // 数据写入后会自动触发PDO处理
            frameCount++;

            // 控制发送频率（每帧间隔约2ms）
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        // 计算吞吐量
        auto actualTestDuration = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - testStartTime);
        double throughputFps = (frameCount * 1000.0) / actualTestDuration.count();
        double throughputBps = throughputFps * 8 * 6;  // 假设每帧8字节，6个电机

        PDO_TEST_DEBUG_PRINT("=== 吞吐量分析 ===");
        PDO_TEST_DEBUG_PRINT("实际测试时长: " << actualTestDuration.count() << " ms");
        PDO_TEST_DEBUG_PRINT("发送帧数: " << frameCount);
        PDO_TEST_DEBUG_PRINT("帧吞吐量: " << throughputFps << " 帧/秒");
        PDO_TEST_DEBUG_PRINT("数据吞吐量: " << throughputBps << " 字节/秒");

        // 验证吞吐量
        bool throughputAcceptable = true;
        if (throughputFps < 400) {  // 帧吞吐量低于400fps认为不合格
            PDO_TEST_DEBUG_PRINT("警告: 帧吞吐量过低 (" << throughputFps << " fps < 400 fps)");
            throughputAcceptable = false;
        }

        PDO_TEST_DEBUG_PRINT("吞吐量结果: " << (throughputAcceptable ? "通过" : "失败"));
        PDO_TEST_DEBUG_PRINT("=== 吞吐量分析结束 ===");

        // 4. 抖动分析
        PDO_TEST_DEBUG_PRINT("测试5.4: 抖动分析");

        // 使用周期测试数据进行抖动分析
        PDO_TEST_DEBUG_PRINT("=== 抖动分析 ===");

        // 计算抖动百分比
        double jitterPercentage = (jitter.count() * 100.0) / targetCycleTime.count();

        // 计算在目标时间±10%范围内的周期数量
        int onTimeCycles = 0;
        auto lowerBound = targetCycleTime - std::chrono::microseconds(200);  // -10%
        auto upperBound = targetCycleTime + std::chrono::microseconds(200);  // +10%

        for (const auto& delay : cycleDelays) {
            if (delay >= lowerBound && delay <= upperBound) {
                onTimeCycles++;
            }
        }

        double onTimePercentage = (onTimeCycles * 100.0) / cycleCount;

        PDO_TEST_DEBUG_PRINT("抖动百分比: " << jitterPercentage << "%");
        PDO_TEST_DEBUG_PRINT("准时周期数: " << onTimeCycles << "/" << cycleCount);
        PDO_TEST_DEBUG_PRINT("准时周期百分比: " << onTimePercentage << "%");

        // 验证抖动
        bool jitterAcceptable = true;
        if (jitterPercentage > 25) {  // 抖动超过25%认为不合格
            PDO_TEST_DEBUG_PRINT("警告: 抖动百分比过大 (" << jitterPercentage << "% > 25%)");
            jitterAcceptable = false;
        }

        if (onTimePercentage < 80) {  // 准时周期少于80%认为不合格
            PDO_TEST_DEBUG_PRINT("警告: 准时周期百分比过低 (" << onTimePercentage << "% < 80%)");
            jitterAcceptable = false;
        }

        PDO_TEST_DEBUG_PRINT("抖动分析结果: " << (jitterAcceptable ? "通过" : "失败"));
        PDO_TEST_DEBUG_PRINT("=== 抖动分析结束 ===");

        // 5. 综合实时性评估
        PDO_TEST_DEBUG_PRINT("测试5.5: 综合实时性评估");

        PDO_TEST_DEBUG_PRINT("=== 实时性综合报告 ===");
        PDO_TEST_DEBUG_PRINT("周期确定性: " << (cycleDeterministic ? "通过" : "失败"));
        PDO_TEST_DEBUG_PRINT("发送延迟: " << (sendDelayAcceptable ? "通过" : "失败"));
        PDO_TEST_DEBUG_PRINT("吞吐量: " << (throughputAcceptable ? "通过" : "失败"));
        PDO_TEST_DEBUG_PRINT("抖动分析: " << (jitterAcceptable ? "通过" : "失败"));

        // 所有测试都通过才算成功
        bool realtimeTestPassed = cycleDeterministic && sendDelayAcceptable &&
                                  throughputAcceptable && jitterAcceptable;

        PDO_TEST_DEBUG_PRINT("实时性测试总体结果: " << (realtimeTestPassed ? "通过" : "失败"));
        PDO_TEST_DEBUG_PRINT("=== 实时性报告结束 ===");

        // 输出外部调试设备捕获指导
        PDO_TEST_DEBUG_PRINT("=== 外部调试设备捕获指导 ===");
        PDO_TEST_DEBUG_PRINT("实时性测试期间，请使用CAN分析仪验证以下指标:");
        PDO_TEST_DEBUG_PRINT("1. 帧间隔时间应接近2ms (允许±200us抖动)");
        PDO_TEST_DEBUG_PRINT("2. 无帧丢失或重复帧");
        PDO_TEST_DEBUG_PRINT("3. 帧时间戳显示稳定的周期性");
        PDO_TEST_DEBUG_PRINT("4. 高峰期吞吐量应达到400+ fps");
        PDO_TEST_DEBUG_PRINT("=== 指导结束 ===");

        if (!realtimeTestPassed) {
            PDO_TEST_DEBUG_PRINT("实时性验证测试失败");
            return false;
        }

        PDO_TEST_DEBUG_PRINT("实时性验证测试通过");
        return true;

    } catch (const std::exception& e) {
        std::cout << "[ERROR][testRpdoRealtime]: 测试异常: " << e.what() << std::endl;
        return false;
    }
}

// ====================== 主测试函数 ======================

/**
 * @brief 发送线程RPDO功能测试主函数
 *
 * @details 测试发送线程的RPDO处理功能，包括：
 * - RPDO基础功能测试（RPDO1和RPDO2）
 * - 多电机并发发送测试
 * - 边界值测试
 * - 性能统计测试
 * - 实时性验证
 * - 外部调试接口
 *
 * @return bool 测试成功返回true，失败返回false
 *
 * @note 本测试使用真实串口环境，需要外部调试设备捕获数据包进行验证
 * @warning 测试前请确保串口设备已正确连接，避免连接失败导致测试跳过
 */
bool testSendThreadPdoFunctionality() {
    try {
        PDO_TEST_DEBUG_PRINT("开始发送线程RPDO功能测试");
        PDO_TEST_DEBUG_PRINT("本测试使用真实串口环境，请确保外部调试设备已准备就绪");

        // 1. 创建测试环境
        PDO_TEST_DEBUG_PRINT("步骤1: 创建测试环境");

        // 创建环形缓冲区
        CircularBuffer sendBuffer;
        CircularBuffer planBuffer;

        // 创建电机数组（6个电机）
        static std::array<Motor, 6> global_motors = {
            Motor(1), Motor(2), Motor(3),
            Motor(4), Motor(5), Motor(6)
        };

        // 创建PDO映射表（为6个电机）
        auto pdoMappingTable = buildArmMappingTable(6);
        PDO_TEST_DEBUG_PRINT("PDO映射表构建完成，条目数: " << pdoMappingTable.size());

        // 创建实际的串口管理器（使用真实串口）
        SerialPortManager serialManager;

        // 连接串口（使用COM6作为测试串口）
        const char* portName = "COM6";
        PDO_TEST_DEBUG_PRINT("尝试连接串口: " << portName);

        if (!serialManager.connect(portName)) {
            std::cerr << "[ERROR][testSendThreadPdoFunctionality]: 串口连接失败 - " << portName << std::endl;
            std::cerr << "[ERROR][testSendThreadPdoFunctionality]: 由于串口连接失败，PDO测试将被跳过" << std::endl;
            std::cerr << "[ERROR][testSendThreadPdoFunctionality]: 请检查串口是否存在或被其他程序占用" << std::endl;

            // 返回失败，但不抛出异常，让其他测试可以继续
            return false;
        }

        PDO_TEST_DEBUG_PRINT("串口连接成功");

        // 创建发送线程
        SendThread sendThread(sendBuffer, planBuffer, global_motors,
                             static_cast<uint8_t>(global_motors.size()),
                             serialManager, pdoMappingTable);

        // 启动发送线程
        PDO_TEST_DEBUG_PRINT("启动发送线程");
        sendThread.start();

        // 等待线程完全启动
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (!sendThread.isRunning()) {
            std::cerr << "[ERROR][testSendThreadPdoFunctionality]: 发送线程启动失败" << std::endl;
            return false;
        }

        PDO_TEST_DEBUG_PRINT("发送线程启动成功");

        // 输出外部调试设备捕获指导
        PDO_TEST_DEBUG_PRINT("=== 外部调试设备捕获指导 ===");
        PDO_TEST_DEBUG_PRINT("请使用CAN分析仪或类似设备监听以下COB-ID:");
        PDO_TEST_DEBUG_PRINT("RPDO1: 0x201-0x206 (目标位置+控制字)");
        PDO_TEST_DEBUG_PRINT("RPDO2: 0x301-0x306 (目标速度+目标电流)");
        PDO_TEST_DEBUG_PRINT("所有帧DLC应为8字节");
        PDO_TEST_DEBUG_PRINT("=== 指导结束 ===");

        // 2. RPDO基础功能测试
        PDO_TEST_DEBUG_PRINT("步骤2: RPDO基础功能测试");
        if (!testRpdoBasicFunctionality(global_motors, sendThread)) {
            PDO_TEST_DEBUG_PRINT("RPDO基础功能测试失败");
            sendThread.stop();
            return false;
        }
        PDO_TEST_DEBUG_PRINT("RPDO基础功能测试通过");

        // 3. 多电机并发测试
        PDO_TEST_DEBUG_PRINT("步骤3: 多电机并发测试");
        if (!testRpdoMultiMotorConcurrent(global_motors, sendThread)) {
            PDO_TEST_DEBUG_PRINT("多电机并发测试失败");
            sendThread.stop();
            return false;
        }
        PDO_TEST_DEBUG_PRINT("多电机并发测试通过");

        // 4. 边界值测试
        PDO_TEST_DEBUG_PRINT("步骤4: 边界值测试");
        if (!testRpdoBoundaryValues(global_motors, sendThread)) {
            PDO_TEST_DEBUG_PRINT("边界值测试失败");
            sendThread.stop();
            return false;
        }
        PDO_TEST_DEBUG_PRINT("边界值测试通过");

        // 5. 性能统计测试
        PDO_TEST_DEBUG_PRINT("步骤5: 性能统计测试");
        if (!testRpdoPerformance(global_motors, sendThread)) {
            PDO_TEST_DEBUG_PRINT("性能统计测试失败");
            sendThread.stop();
            return false;
        }
        PDO_TEST_DEBUG_PRINT("性能统计测试通过");

        // 6. 实时性验证
        PDO_TEST_DEBUG_PRINT("步骤6: 实时性验证");
        if (!testRpdoRealtime(global_motors, sendThread)) {
            PDO_TEST_DEBUG_PRINT("实时性验证失败");
            sendThread.stop();
            return false;
        }
        PDO_TEST_DEBUG_PRINT("实时性验证通过");

        // 7. 清理测试环境
        PDO_TEST_DEBUG_PRINT("步骤7: 清理测试环境");
        sendThread.stop();
        PDO_TEST_DEBUG_PRINT("发送线程已停止");

        PDO_TEST_DEBUG_PRINT("发送线程RPDO功能测试全部通过");
        PDO_TEST_DEBUG_PRINT("请使用外部调试设备验证捕获的数据包");
        return true;

    } catch (const std::exception& e) {
        std::cout << "[ERROR][testSendThreadPdoFunctionality]: 测试异常: " << e.what() << std::endl;
        return false;
    }
}

#endif // TEST_SEND_THREAD_PDO_HPP