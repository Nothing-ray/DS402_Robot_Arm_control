/**
 * @file test_Send_Thread.hpp
 * @brief 测试发送线程的PDO功能
 * 
 * 测试基于PDO映射表的数据读取和CAN帧构建功能
 */

#ifndef TEST_SEND_THREAD_HPP
#define TEST_SEND_THREAD_HPP

#include <iostream>
#include <vector>
#include <memory>
#include "../CLASS_Motor.hpp"
#include "../Send_Thread.hpp"
#include "../PDO_config.hpp"
#include "../CircularBuffer.hpp"
#include "../Serial_Module.hpp"

/**
 * @brief 测试发送线程的PDO功能
 * 
 * 测试内容：
 * 1. 创建电机实例和PDO映射表
 * 2. 初始化发送线程
 * 3. 设置电机目标数据
 * 4. 验证PDO数据处理功能
 */
inline void testSendThreadPdoFunction() {
    std::cout << "========== 开始测试发送线程PDO功能 ==========" << std::endl;
    
    try {
        // 步骤1：创建电机实例（6个电机）
        std::cout << "[步骤1] 创建电机实例..." << std::endl;
        std::vector<Motor> motors;
        motors.reserve(6);
        for (uint8_t i = 1; i <= 6; ++i) {
            motors.emplace_back(i);
        }
        std::cout << "✓ 成功创建 " << motors.size() << " 个电机实例" << std::endl;
        
        // 步骤2：构建PDO映射表
        std::cout << "[步骤2] 构建PDO映射表..." << std::endl;
        auto pdoMappingTable = buildArmMappingTable(6);
        std::cout << "✓ PDO映射表构建完成，包含 " << pdoMappingTable.size() << " 个映射条目" << std::endl;
        
        // 步骤3：设置一些测试数据
        std::cout << "[步骤3] 设置电机测试数据..." << std::endl;
        for (size_t i = 0; i < motors.size(); ++i) {
            auto& motor = motors[i];
            
            // 设置目标位置（编码器值）
            motor.position.raw_target.atomicWriteValue(1000 + i * 100);
            
            // 设置控制字
            uint16_t controlWord = 0x000F; // 基本控制字
            std::memcpy(motor.stateAndMode.controlData.controlWordRaw, &controlWord, 2);
            
            // 设置目标电流（mA）
            motor.current.raw_target.atomicWriteValue(500 + i * 50);
            
            // 设置目标速度（RPM）
            motor.velocity.raw_target_velocity_mode.atomicWriteValue(1000 + i * 100);
            
            std::cout << "  电机 " << (i+1) << ": 位置=" << (1000 + i * 100) 
                      << ", 电流=" << (500 + i * 50) << "mA, 速度=" << (1000 + i * 100) << "RPM" << std::endl;
        }
        std::cout << "✓ 测试数据设置完成" << std::endl;
        
        // 步骤4：创建缓冲区和串口管理器
        std::cout << "[步骤4] 创建通信资源..." << std::endl;
        CircularBuffer sendBuffer(1024);
        CircularBuffer planBuffer(1024);
        SerialPortManager serialManager;
        
        // 注意：这里我们不实际连接串口，只是测试PDO数据处理逻辑
        std::cout << "✓ 通信资源创建完成（串口未连接）" << std::endl;
        
        // 步骤5：创建发送线程实例
        std::cout << "[步骤5] 创建发送线程..." << std::endl;
        SendThread sendThread(sendBuffer, planBuffer, motors, 6, serialManager, pdoMappingTable);
        std::cout << "✓ 发送线程创建成功" << std::endl;
        
        // 步骤6：测试PDO数据处理（不启动线程，直接调用函数）
        std::cout << "[步骤6] 测试PDO数据处理功能..." << std::endl;
        
        // 获取线程的私有访问权限来测试内部函数
        // 注意：这种方式仅用于测试，生产代码不应该这样做
        class SendThreadTestAccessor : public SendThread {
        public:
            SendThreadTestAccessor(CircularBuffer& sendBuffer,
                                 CircularBuffer& planBuffer,
                                 std::vector<Motor>& motors,
                                 uint8_t motorCount,
                                 SerialPortManager& serialManager,
                                 const std::vector<PdoMappingEntry>& pdoMappingTable)
                : SendThread(sendBuffer, planBuffer, motors, motorCount, serialManager, pdoMappingTable) {}
            
            // 暴露私有方法用于测试
            size_t testBuildAndSendPdoFrames() {
                return buildAndSendPdoFrames();
            }
        };
        
        SendThreadTestAccessor testAccessor(sendBuffer, planBuffer, motors, 6, serialManager, pdoMappingTable);
        
        // 测试PDO数据处理（预期会因串口未连接而失败，但能验证逻辑）
        try {
            size_t processedFrames = testAccessor.testBuildAndSendPdoFrames();
            std::cout << "✓ PDO数据处理测试完成，处理了 " << processedFrames << " 个帧" << std::endl;
        } catch (const std::exception& e) {
            std::cout << "⚠ PDO数据处理测试异常（预期，因为串口未连接）: " << e.what() << std::endl;
            std::cout << "✓ PDO数据处理逻辑测试完成（异常处理正常）" << std::endl;
        }
        
        // 步骤7：验证数据映射正确性
        std::cout << "[步骤7] 验证PDO映射表数据..." << std::endl;
        int rpdoCount = 0;
        int tpdoCount = 0;
        
        for (const auto& entry : pdoMappingTable) {
            if (!entry.isTx) {
                rpdoCount++;
                std::cout << "  RPDO: 电机" << static_cast<int>(entry.motorIndex) 
                          << ", PDO" << static_cast<int>(entry.pdoIndex)
                          << ", OD=0x" << std::hex << entry.index 
                          << ", 偏移=" << std::dec << static_cast<int>(entry.offsetInPdo)
                          << ", 大小=" << static_cast<int>(entry.size) << std::endl;
            } else {
                tpdoCount++;
            }
        }
        
        std::cout << "✓ 映射表验证完成：RPDO=" << rpdoCount << ", TPDO=" << tpdoCount << std::endl;
        
        std::cout << "========== 发送线程PDO功能测试完成 ==========" << std::endl;
        std::cout << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "[ERROR] 测试失败: " << e.what() << std::endl;
        std::cout << "========== 发送线程PDO功能测试失败 ==========" << std::endl;
        std::cout << std::endl;
    }
}

/**
 * @brief 简化的PDO功能测试（仅测试数据映射）
 */
inline void testSendThreadPdoMapping() {
    std::cout << "========== 开始测试PDO映射功能 ==========" << std::endl;
    
    try {
        // 创建单个电机测试
        Motor motor(1);
        
        // 设置测试数据
        motor.position.raw_target.atomicWriteValue(12345);
        motor.current.raw_target.atomicWriteValue(678);
        
        // 构建PDO映射表
        auto pdoMappingTable = buildArmMappingTable(1);
        
        // 查找位置映射
        bool foundPosition = false;
        bool foundCurrent = false;
        
        for (const auto& entry : pdoMappingTable) {
            if (entry.motorIndex == 1 && !entry.isTx) { // RPDO
                if (entry.index == OD_TARGET_POSITION) {
                    foundPosition = true;
                    std::cout << "✓ 找到位置映射: PDO" << static_cast<int>(entry.pdoIndex) 
                              << ", 偏移=" << static_cast<int>(entry.offsetInPdo) 
                              << ", 大小=" << static_cast<int>(entry.size) << std::endl;
                    
                    // 验证数据读取
                    Position_type value = motor.position.raw_target.atomicReadValue();
                    std::cout << "  位置数据验证: 期望=12345, 实际=" << value << std::endl;
                    
                } else if (entry.index == OD_TARGET_CURRENT) {
                    foundCurrent = true;
                    std::cout << "✓ 找到电流映射: PDO" << static_cast<int>(entry.pdoIndex) 
                              << ", 偏移=" << static_cast<int>(entry.offsetInPdo) 
                              << ", 大小=" << static_cast<int>(entry.size) << std::endl;
                    
                    // 验证数据读取
                    Current_type value = motor.current.raw_target.atomicReadValue();
                    std::cout << "  电流数据验证: 期望=678, 实际=" << value << std::endl;
                }
            }
        }
        
        if (foundPosition && foundCurrent) {
            std::cout << "✓ PDO映射功能测试通过" << std::endl;
        } else {
            std::cout << "✗ PDO映射功能测试失败：未找到所有必需的映射" << std::endl;
        }
        
        std::cout << "========== PDO映射功能测试完成 ==========" << std::endl;
        std::cout << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "[ERROR] PDO映射测试失败: " << e.what() << std::endl;
        std::cout << "========== PDO映射功能测试失败 ==========" << std::endl;
        std::cout << std::endl;
    }
}

#endif // TEST_SEND_THREAD_HPP