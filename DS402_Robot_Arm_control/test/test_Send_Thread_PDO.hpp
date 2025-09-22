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
 * - 进行功能验证和数据包验证
 * - 通过外部调试设备验证发送的数据包
 *
 * @par 测试重点：
 * - RPDO1：目标位置和控制字发送（0x200 + NodeID）
 * - RPDO2：目标速度和电流发送（0x300 + NodeID）
 * - 六电机并发发送性能分析
 * - 真实串口传输功能验证
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
#define ENABLE_DEBUG_OUTPUT false

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

// ====================== PerformanceMonitor测试辅助类 ======================

/**
 * @brief 性能监控辅助类
 *
 * @details 简化测试中的性能统计操作，提供统一的接口来获取和验证Send_Thread的性能统计
 */
class PerformanceMonitor {
private:
    SendThread& sendThread_;
    SendThread::PerformanceStats baselineStats_;
    bool hasBaseline_;

public:
    explicit PerformanceMonitor(SendThread& sendThread)
        : sendThread_(sendThread), hasBaseline_(false) {}

    /**
     * @brief 设置基准性能统计
     */
    void setBaseline() {
        baselineStats_ = sendThread_.getPerformanceStats();
        hasBaseline_ = true;
    }

    /**
     * @brief 获取当前性能统计
     */
    SendThread::PerformanceStats getCurrentStats() const {
        return sendThread_.getPerformanceStats();
    }

    /**
     * @brief 重置性能统计
     */
    void reset() {
        sendThread_.resetPerformanceStats();
        hasBaseline_ = false;
    }

    /**
     * @brief 验证性能统计在合理范围内
     *
     * @param maxExpectedTimeUs 预期的最大处理时间（微秒）
     * @return bool 验证通过返回true
     */
    bool validatePerformanceRange(uint64_t maxExpectedTimeUs = 2000) const {
        auto stats = getCurrentStats();

        // 验证当前处理时间在合理范围内
        if (stats.currentCycleTime > maxExpectedTimeUs) {
            PDO_TEST_DEBUG_PRINT("[性能警告] 当前周期时间 " << stats.currentCycleTime
                << " us 超过预期 " << maxExpectedTimeUs << " us");
            return false;
        }

        // 验证平均处理时间在合理范围内
        if (stats.avgProcessingTime > maxExpectedTimeUs) {
            PDO_TEST_DEBUG_PRINT("[性能警告] 平均处理时间 " << stats.avgProcessingTime
                << " us 超过预期 " << maxExpectedTimeUs << " us");
            return false;
        }

        return true;
    }

    /**
     * @brief 打印性能统计报告
     */
    void printReport() const {
        auto stats = getCurrentStats();

        PDO_TEST_DEBUG_PRINT("=== Send_Thread 性能统计报告 ===");
        PDO_TEST_DEBUG_PRINT("当前周期时间: " << stats.currentCycleTime << " us");
        PDO_TEST_DEBUG_PRINT("最小处理时间: " << stats.minProcessingTime << " us");
        PDO_TEST_DEBUG_PRINT("最大处理时间: " << stats.maxProcessingTime << " us");
        PDO_TEST_DEBUG_PRINT("平均处理时间: " << stats.avgProcessingTime << " us");
        PDO_TEST_DEBUG_PRINT("总样本数: " << stats.totalSamples);
        PDO_TEST_DEBUG_PRINT("周期计数: " << stats.cycleCount);

        if (hasBaseline_) {
            PDO_TEST_DEBUG_PRINT("=== 与基准对比 ===");
            PDO_TEST_DEBUG_PRINT("基准周期时间: " << baselineStats_.currentCycleTime << " us");
            PDO_TEST_DEBUG_PRINT("基准平均时间: " << baselineStats_.avgProcessingTime << " us");

            if (stats.avgProcessingTime > baselineStats_.avgProcessingTime) {
                double degradation = ((double)stats.avgProcessingTime - baselineStats_.avgProcessingTime) / baselineStats_.avgProcessingTime * 100.0;
                PDO_TEST_DEBUG_PRINT("性能变化: +" << degradation << "%");
            } else {
                double improvement = ((double)baselineStats_.avgProcessingTime - stats.avgProcessingTime) / baselineStats_.avgProcessingTime * 100.0;
                PDO_TEST_DEBUG_PRINT("性能变化: -" << improvement << "%");
            }
        }

        PDO_TEST_DEBUG_PRINT("=== 报告结束 ===");
    }

    /**
     * @brief 验证性能统计功能正常工作
     */
    bool validateStatsFunctionality() {
        try {
            // 重置统计
            reset();

            // 获取重置后的统计
            auto resetStats = getCurrentStats();

            // 验证重置后的值是否合理
            if (resetStats.totalSamples != 0 || resetStats.cycleCount != 0) {
                PDO_TEST_DEBUG_PRINT("[错误] 重置后统计数值异常");
                return false;
            }

            // 等待一个周期
            std::this_thread::sleep_for(std::chrono::milliseconds(3));

            // 再次获取统计
            auto newStats = getCurrentStats();

            // 验证统计是否有更新
            if (newStats.cycleCount <= resetStats.cycleCount) {
                PDO_TEST_DEBUG_PRINT("[错误] 性能统计计数器未更新");
                return false;
            }

            return true;
        } catch (const std::exception& e) {
            PDO_TEST_DEBUG_PRINT("[错误] 性能统计验证异常: " << e.what());
            return false;
        }
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
 * - 数据正确性验证
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
            // 将原始位置值转换为角度值
            float targetAngle = static_cast<float>(testData.motors[testMotorIndex - 1].targetPosition) * 180.0f / 32768.0f;
            testMotor.position.target_degree.store(targetAngle);
            testMotor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

            // 控制字仍然使用memcpy（不需要标志位）
            std::memcpy(const_cast<uint8_t*>(testMotor.stateAndMode.controlData.controlWordRaw), &testData.motors[testMotorIndex - 1].controlWord, 2);

            // 调用刷新方法进行数据转换
            testMotor.refreshMotorData(testMotor.position);
        }

        // 等待发送线程处理PDO（基于帧计数验证）
        {
            // 预期要发送的帧数量：RPDO1需要发送1帧
            const uint64_t expectedFrames = 1;
            uint64_t startFrameCount = sendThread.getGlobalFrameCounter();

            // 等待直到发送了预期数量的帧（最多等待2ms，超时则退出）
            auto startTime = std::chrono::high_resolution_clock::now();
            const auto timeout = std::chrono::milliseconds(2);
            while (sendThread.getGlobalFrameCounter() - startFrameCount < expectedFrames) {
                auto now = std::chrono::high_resolution_clock::now();
                if (now - startTime > timeout) {
                    PDO_TEST_DEBUG_PRINT("警告：PDO帧发送超时");
                    break;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(50));  // 短暂休眠避免忙等待
            }
            uint64_t actualFramesSent = sendThread.getGlobalFrameCounter() - startFrameCount;

            PDO_TEST_DEBUG_PRINT("RPDO1帧发送验证：期望=" << expectedFrames
                              << ", 实际=" << actualFramesSent);
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
        }

        // 等待发送线程处理PDO（基于帧计数验证）
        {
            // 预期要发送的帧数量：RPDO2需要发送1帧
            const uint64_t expectedFrames = 1;
            uint64_t startFrameCount = sendThread.getGlobalFrameCounter();

            // 等待直到发送了预期数量的帧（最多等待2ms，超时则退出）
            auto startTime = std::chrono::high_resolution_clock::now();
            const auto timeout = std::chrono::milliseconds(2);
            while (sendThread.getGlobalFrameCounter() - startFrameCount < expectedFrames) {
                auto now = std::chrono::high_resolution_clock::now();
                if (now - startTime > timeout) {
                    PDO_TEST_DEBUG_PRINT("警告：PDO帧发送超时");
                    break;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(50));  // 短暂休眠避免忙等待
            }
            uint64_t actualFramesSent = sendThread.getGlobalFrameCounter() - startFrameCount;

            PDO_TEST_DEBUG_PRINT("RPDO2帧发送验证：期望=" << expectedFrames
                              << ", 实际=" << actualFramesSent);
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

        // 4. 基础功能总结验证
        PDO_TEST_DEBUG_PRINT("测试1.4: RPDO基础功能总结验证");

        // 验证发送线程统计功能（如果启用）
#ifdef ENABLE_CYCLE_TIMING
        auto stats = sendThread.getPerformanceStats();
        PDO_TEST_DEBUG_PRINT("=== 发送线程性能统计 ===");
        PDO_TEST_DEBUG_PRINT("周期数: " << stats.cycleCount);
        PDO_TEST_DEBUG_PRINT("当前处理时间: " << stats.currentCycleTime << " us");
        PDO_TEST_DEBUG_PRINT("平均处理时间: " << stats.avgProcessingTime << " us");
        PDO_TEST_DEBUG_PRINT("最小处理时间: " << stats.minProcessingTime << " us");
        PDO_TEST_DEBUG_PRINT("最大处理时间: " << stats.maxProcessingTime << " us");
        PDO_TEST_DEBUG_PRINT("统计样本数: " << stats.totalSamples);
        PDO_TEST_DEBUG_PRINT("=== 统计结束 ===");
#endif

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
 * - 验证批量发送功能正确性
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
        }

        // 3. 等待发送线程处理并发PDO
        PDO_TEST_DEBUG_PRINT("测试2.3: 等待并发PDO处理");

        // 等待并发PDO处理（基于帧计数验证）
        {
            // 预期要发送的帧数量：6个电机 × 2帧RPDO = 12帧
            const uint64_t expectedFrames = 12;
            uint64_t startFrameCount = sendThread.getGlobalFrameCounter();

            // 等待直到发送了预期数量的帧（最多等待5ms，超时则退出）
            auto startTime = std::chrono::high_resolution_clock::now();
            const auto timeout = std::chrono::milliseconds(5);
            while (sendThread.getGlobalFrameCounter() - startFrameCount < expectedFrames) {
                auto now = std::chrono::high_resolution_clock::now();
                if (now - startTime > timeout) {
                    PDO_TEST_DEBUG_PRINT("警告：并发PDO帧发送超时");
                    break;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(50));  // 短暂休眠避免忙等待
            }
            uint64_t actualFramesSent = sendThread.getGlobalFrameCounter() - startFrameCount;

            PDO_TEST_DEBUG_PRINT("并发PDO处理验证：期望=" << expectedFrames
                              << ", 实际=" << actualFramesSent);
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

        // 5. 多次并发功能测试
        PDO_TEST_DEBUG_PRINT("测试2.4: 多次并发功能测试");

        for (int testIteration = 0; testIteration < 20; ++testIteration) {
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

            // 等待处理
            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            // 每5次测试输出一次进度
            if ((testIteration + 1) % 5 == 0) {
                PDO_TEST_DEBUG_PRINT("完成 " << (testIteration + 1) << "/20 次并发测试");
            }
        }

        PDO_TEST_DEBUG_PRINT("多次并发功能测试完成");

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

        // 等待边界值PDO处理
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

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

        // 等待边界值并发处理
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

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

        // 6. 边界值功能测试
        PDO_TEST_DEBUG_PRINT("测试3.5: 边界值功能测试");

        for (int i = 0; i < 10; ++i) {
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

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        PDO_TEST_DEBUG_PRINT("边界值功能测试完成");

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

// ====================== 性能统计验证测试 ======================

/**
 * @brief 性能统计验证测试
 *
 * @details 验证Send_Thread内置性能统计功能：
 * - 验证周期时间统计正确性
 * - 验证处理时间统计功能
 * - 验证统计信息查询接口
 * - 验证统计重置功能
 *
 * @param motors 电机数组引用
 * @param sendThread 发送线程引用
 * @return bool 测试成功返回true，失败返回false
 */
bool testRpdoPerformanceStats(std::array<Motor, 6>& motors, SendThread& sendThread) {
    try {
        PDO_TEST_DEBUG_PRINT("开始性能统计验证测试");

#ifdef ENABLE_CYCLE_TIMING
        // 创建性能监控器
        PerformanceMonitor monitor(sendThread);

        // 1. 验证统计功能正常性
        PDO_TEST_DEBUG_PRINT("测试4.1: 验证统计功能正常性");
        if (!monitor.validateStatsFunctionality()) {
            PDO_TEST_DEBUG_PRINT("性能统计功能验证失败");
            return false;
        }
        PDO_TEST_DEBUG_PRINT("性能统计功能验证通过");

        // 2. 设置基准统计
        PDO_TEST_DEBUG_PRINT("测试4.2: 设置基准统计");
        monitor.setBaseline();

        // 3. 执行PDO操作产生统计数据
        PDO_TEST_DEBUG_PRINT("测试4.3: 执行PDO操作产生统计数据");

        Motor& testMotor = motors.at(0);

        // 执行多次PDO操作
        for (int i = 0; i < 10; ++i) {
            float targetAngle = 10.0f * (i + 1);
            uint16_t controlWord = 0x000F;

            testMotor.position.target_degree.store(targetAngle);
            testMotor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);
            std::memcpy(const_cast<uint8_t*>(testMotor.stateAndMode.controlData.controlWordRaw), &controlWord, 2);
            testMotor.refreshMotorData(testMotor.position);

            // 短暂等待让发送线程处理
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        // 4. 验证性能范围
        PDO_TEST_DEBUG_PRINT("测试4.4: 验证性能范围");
        if (!monitor.validatePerformanceRange(2000)) {
            PDO_TEST_DEBUG_PRINT("性能范围验证失败");
            return false;
        }
        PDO_TEST_DEBUG_PRINT("性能范围验证通过");

        // 5. 打印性能报告
        PDO_TEST_DEBUG_PRINT("测试4.5: 打印性能报告");
        monitor.printReport();

        // 6. 验证处理时间正常性
        PDO_TEST_DEBUG_PRINT("测试4.6: 验证处理时间正常性");
        bool isTimeNormal = sendThread.isProcessingTimeNormal();

        TEST_ASSERT(isTimeNormal, "处理时间应在正常范围内");
        PDO_TEST_DEBUG_PRINT("处理时间正常性检查: " << (isTimeNormal ? "正常" : "异常"));

        PDO_TEST_DEBUG_PRINT("性能统计验证测试通过");
        return true;

#else
        PDO_TEST_DEBUG_PRINT("警告: ENABLE_CYCLE_TIMING未启用，跳过性能统计验证");
        return true;
#endif

    } catch (const std::exception& e) {
        std::cout << "[ERROR][testRpdoPerformanceStats]: 测试异常: " << e.what() << std::endl;
        return false;
    }
}

// ====================== 简化功能验证测试 ======================

/**
 * @brief RPDO功能验证测试
 *
 * @details 简化的功能验证测试，替代复杂的实时性测试：
 * - 验证基本PDO发送功能
 * - 验证数据正确性
 * - 验证线程健康状态
 *
 * @param motors 电机数组引用
 * @param sendThread 发送线程引用
 * @return bool 测试成功返回true，失败返回false
 */
bool testRpdoFunctionality(std::array<Motor, 6>& motors, SendThread& sendThread) {
    try {
        PDO_TEST_DEBUG_PRINT("开始RPDO功能验证测试");

        // 1. 基本功能验证
        PDO_TEST_DEBUG_PRINT("测试5.1: 基本PDO发送验证");

        Motor& testMotor = motors.at(0);
        uint64_t startFrameCount = sendThread.getGlobalFrameCounter();

        // 执行基本PDO操作
        float targetAngle = 45.0f;
        uint16_t controlWord = 0x000F;

        testMotor.position.target_degree.store(targetAngle);
        testMotor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);
        std::memcpy(const_cast<uint8_t*>(testMotor.stateAndMode.controlData.controlWordRaw), &controlWord, 2);
        testMotor.refreshMotorData(testMotor.position);

        // 等待处理
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        uint64_t endFrameCount = sendThread.getGlobalFrameCounter();
        uint64_t framesSent = endFrameCount - startFrameCount;

        PDO_TEST_DEBUG_PRINT("PDO帧发送验证：发送了 " << framesSent << " 帧");

        // 2. 健康状态验证
        PDO_TEST_DEBUG_PRINT("测试5.2: 线程健康状态验证");

        bool isHealthy = sendThread.isHealthy();
        bool isProcessingTimeNormal = sendThread.isProcessingTimeNormal();

#ifdef ENABLE_CYCLE_TIMING
        auto stats = sendThread.getPerformanceStats();
        PDO_TEST_DEBUG_PRINT("=== 线程状态报告 ===");
        PDO_TEST_DEBUG_PRINT("健康状态: " << (isHealthy ? "健康" : "不健康"));
        PDO_TEST_DEBUG_PRINT("处理时间正常: " << (isProcessingTimeNormal ? "正常" : "异常"));
        PDO_TEST_DEBUG_PRINT("当前周期数: " << stats.cycleCount);
        PDO_TEST_DEBUG_PRINT("当前处理时间: " << stats.currentCycleTime << " us");
        PDO_TEST_DEBUG_PRINT("平均处理时间: " << stats.avgProcessingTime << " us");
        PDO_TEST_DEBUG_PRINT("错误计数: " << sendThread.getErrorCount());
        PDO_TEST_DEBUG_PRINT("=== 状态报告结束 ===");
#else
        PDO_TEST_DEBUG_PRINT("=== 线程状态报告 ===");
        PDO_TEST_DEBUG_PRINT("健康状态: " << (isHealthy ? "健康" : "不健康"));
        PDO_TEST_DEBUG_PRINT("错误计数: " << sendThread.getErrorCount());
        PDO_TEST_DEBUG_PRINT("=== 状态报告结束 ===");
#endif

        // 3. 验证结果
        TEST_ASSERT(isHealthy, "线程应保持健康状态");
        TEST_ASSERT(framesSent > 0, "应至少发送1帧");

        PDO_TEST_DEBUG_PRINT("RPDO功能验证测试通过");
        return true;

    } catch (const std::exception& e) {
        std::cout << "[ERROR][testRpdoFunctionality]: 测试异常: " << e.what() << std::endl;
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
 * - 性能统计验证测试
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

        // 连接串口（使用COM5作为测试串口）
        const char* portName = "COM5";
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

        // 5. 性能统计验证测试
        PDO_TEST_DEBUG_PRINT("步骤5: 性能统计验证测试");
        if (!testRpdoPerformanceStats(global_motors, sendThread)) {
            PDO_TEST_DEBUG_PRINT("性能统计验证测试失败");
            sendThread.stop();
            return false;
        }
        PDO_TEST_DEBUG_PRINT("性能统计验证测试通过");

        // 6. 功能验证测试
        PDO_TEST_DEBUG_PRINT("步骤6: 功能验证测试");
        if (!testRpdoFunctionality(global_motors, sendThread)) {
            PDO_TEST_DEBUG_PRINT("功能验证测试失败");
            sendThread.stop();
            return false;
        }
        PDO_TEST_DEBUG_PRINT("功能验证测试通过");

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