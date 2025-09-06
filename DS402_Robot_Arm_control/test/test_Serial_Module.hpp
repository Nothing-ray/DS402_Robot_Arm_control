/**
 * @file test_Serial_Module.hpp
 * @brief Serial_Module模块基础发送测试函数
 * 
 * @details 提供简单的串口发送功能测试，按回车键逐个发送CAN帧数组中的帧
 */

#ifndef TEST_SERIAL_MODULE_HPP
#define TEST_SERIAL_MODULE_HPP

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <boost/asio.hpp>
#include <initializer_list>

#include "../Serial_Module.hpp"
#include "../CAN_frame.hpp"

/**
 * @brief 辅助函数：从初始化列表创建CanFrame
 * @param id 帧ID
 * @param data 数据初始化列表
 * @param dlc 数据长度
 * @return 构造的CanFrame对象
 */
inline CanFrame createCanFrame(uint32_t id, std::initializer_list<uint8_t> data, uint8_t dlc) {
    return CanFrame(id, data.begin(), dlc);
}

/**
 * @brief 基础串口发送测试函数
 * 
 * @param portName 串口设备名称，默认"COM1"
 */
inline void testSerialBasicSend(const std::string& portName = "COM1") {
    SerialPortManager serial;
    
    // 连接串口
    if (!serial.connect(portName)) {
        std::cerr << "[ERROR][testSerialBasicSend]: 串口连接失败 - " << portName << std::endl;
        return;
    }
    
    std::cout << "[INFO][testSerialBasicSend]: 串口连接成功" << std::endl;
    
    // 测试CAN帧数组 - 可根据需要手动添加修改
    std::vector<CanFrame> testFrames = {
        // 简单数据帧示例
        createCanFrame(0x100, {0x01, 0x02, 0x03, 0x04}, 4),
        createCanFrame(0x200, {0xAA, 0xBB, 0xCC, 0xDD}, 4),
        createCanFrame(0x300, {0x11, 0x22}, 2),
        
        // SDO帧示例
        createCanFrame(0x600 + 1, {0x2F, 0x60, 0x60, 0x00, 0x01}, 5),
        createCanFrame(0x600 + 1, {0x40, 0x60, 0x60, 0x00}, 4),
        
        // PDO帧示例
        createCanFrame(0x180 + 1, {0x00, 0x00, 0x00, 0x00}, 4),
        createCanFrame(0x280 + 1, {0x00, 0x00, 0x00, 0x00}, 4),
        
        // NMT帧
        createCanFrame(0x000, {0x01}, 1)
    };
    
    std::cout << "加载了 " << testFrames.size() << " 个测试帧" << std::endl;
    std::cout << "按回车发送下一个帧，输入'q'退出..." << std::endl;
    
    size_t currentIndex = 0;
    while (currentIndex < testFrames.size()) {
        std::string input;
        std::getline(std::cin, input);
        
        if (input == "q" || input == "Q") {
            break;
        }
        
        const auto& frame = testFrames[currentIndex];
        
        if (serial.sendFrame(frame)) {
            std::cout << "发送成功: 帧" << currentIndex 
                      << " - ID=0x" << std::hex << frame.frameID 
                      << ", DLC=" << std::dec << static_cast<int>(frame.dlc) << std::endl;
        } else {
            std::cerr << "发送失败: 帧" << currentIndex << std::endl;
        }
        
        currentIndex++;
        
        if (currentIndex >= testFrames.size()) {
            std::cout << "所有测试帧已发送完毕" << std::endl;
            break;
        }
    }
    
    serial.disconnect();
    std::cout << "测试结束" << std::endl;
}

/**
 * @brief 批量串口发送测试函数
 * 
 * @details 自动生成6个电机的所有PDO帧，按照电机1-6的顺序批量发送，
 *          最后发送同步帧作为结束标志
 * 
 * @param portName 串口设备名称，默认"COM1"
 */
inline void testSerialBatchSend(const std::string& portName = "COM1") {
    SerialPortManager serial;
    
    // 连接串口
    if (!serial.connect(portName)) {
        std::cerr << "[ERROR][testSerialBatchSend]: 串口连接失败 - " << portName << std::endl;
        return;
    }
    
    std::cout << "[INFO][testSerialBatchSend]: 串口连接成功" << std::endl;
    
    // 生成所有电机的PDO测试帧
    std::vector<CanFrame> testFrames;
    
    // 为6个电机生成PDO帧
    for (uint8_t motorID = 1; motorID <= 6; ++motorID) {
        // 为每个电机生成测试数据（递增的值）
        uint32_t positionValue = 1000 * motorID;
        uint16_t velocityValue = 500 * motorID;
        uint16_t currentValue = 100 * motorID;
        uint16_t controlWord = 0x0006; // 标准控制字
        uint16_t statusWord = 0x0237;  // 标准状态字
        
        // 电机1的PDO帧
        // RPDO1: 目标位置 + 控制字
        testFrames.push_back(createCanFrame(0x200 + motorID, {
            static_cast<uint8_t>((positionValue >> 24) & 0xFF),
            static_cast<uint8_t>((positionValue >> 16) & 0xFF),
            static_cast<uint8_t>((positionValue >> 8) & 0xFF),
            static_cast<uint8_t>(positionValue & 0xFF),
            static_cast<uint8_t>((controlWord >> 8) & 0xFF),
            static_cast<uint8_t>(controlWord & 0xFF)
        }, 6));
        
        // RPDO2: 目标速度 + 目标电流
        testFrames.push_back(createCanFrame(0x300 + motorID, {
            static_cast<uint8_t>((velocityValue >> 8) & 0xFF),
            static_cast<uint8_t>(velocityValue & 0xFF),
            static_cast<uint8_t>((currentValue >> 8) & 0xFF),
            static_cast<uint8_t>(currentValue & 0xFF)
        }, 4));
        
        // TPDO1: 实际位置 + 状态字
        testFrames.push_back(createCanFrame(0x180 + motorID, {
            static_cast<uint8_t>((positionValue >> 24) & 0xFF),
            static_cast<uint8_t>((positionValue >> 16) & 0xFF),
            static_cast<uint8_t>((positionValue >> 8) & 0xFF),
            static_cast<uint8_t>(positionValue & 0xFF),
            static_cast<uint8_t>((statusWord >> 8) & 0xFF),
            static_cast<uint8_t>(statusWord & 0xFF)
        }, 6));
        
        // TPDO2: 实际速度 + 实际电流
        testFrames.push_back(createCanFrame(0x280 + motorID, {
            static_cast<uint8_t>((velocityValue >> 8) & 0xFF),
            static_cast<uint8_t>(velocityValue & 0xFF),
            static_cast<uint8_t>((currentValue >> 8) & 0xFF),
            static_cast<uint8_t>(currentValue & 0xFF)
        }, 4));
    }
    
    // 添加同步帧作为结束标志
    testFrames.push_back(createCanFrame(0x080, {0x00}, 1));
    
    std::cout << "生成了 " << testFrames.size() << " 个测试帧" << std::endl;
    std::cout << "开始批量发送所有帧..." << std::endl;
    
    // 批量发送所有帧
    size_t successCount = 0;
    size_t failCount = 0;
    
    // 开始计时（纳秒精度）
    auto startTime = std::chrono::high_resolution_clock::now();
    
    for (size_t i = 0; i < testFrames.size(); ++i) {
        const auto& frame = testFrames[i];
        
        if (serial.sendFrame(frame)) {
            successCount++;
        } else {
            failCount++;
        }
        
        // 添加小延迟避免发送过快
        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // 结束计时并计算耗时（纳秒转换为微秒）
    auto endTime = std::chrono::high_resolution_clock::now();
    auto durationNs = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime);
    double elapsedTimeUs = durationNs.count() / 1000.0; // 纳秒转换为微秒
    
    std::cout << "批量发送完成 - 成功: " << successCount 
              << ", 失败: " << failCount << std::endl;
    std::cout << "批量发送总耗时: " << std::fixed << std::setprecision(2) 
              << elapsedTimeUs << " us" << std::endl;
    std::cout << "平均每帧耗时: " << std::fixed << std::setprecision(2) 
              << (successCount > 0 ? elapsedTimeUs / successCount : 0) << " us/帧" << std::endl;
    
    serial.disconnect();
    std::cout << "批量发送测试结束" << std::endl;
}

/**
 * @brief 真正的批量发送测试函数（使用sendFramesBatch方法）
 * 
 * @details 使用SerialPortManager的sendFramesBatch方法进行真正的批量发送测试，
 *          而不是逐帧发送。这样可以测试批量发送的性能优势。
 * 
 * @param portName 串口设备名称，默认"COM1"
 */
inline void testSerialRealBatchSend(const std::string& portName = "COM1") {
    SerialPortManager serial;
    
    // 连接串口
    if (!serial.connect(portName)) {
        std::cerr << "[ERROR][testSerialRealBatchSend]: 串口连接失败 - " << portName << std::endl;
        return;
    }
    
    std::cout << "[INFO][testSerialRealBatchSend]: 串口连接成功" << std::endl;
    
    // 生成所有电机的PDO测试帧
    std::vector<CanFrame> testFrames;
    
    // 为6个电机生成PDO帧
    for (uint8_t motorID = 1; motorID <= 6; ++motorID) {
        // 为每个电机生成测试数据（递增的值）
        uint32_t positionValue = 1000 * motorID;
        uint16_t velocityValue = 500 * motorID;
        uint16_t currentValue = 100 * motorID;
        uint16_t controlWord = 0x0006; // 标准控制字
        uint16_t statusWord = 0x0237;  // 标准状态字
        
        // 电机1的PDO帧
        // RPDO1: 目标位置 + 控制字
        testFrames.push_back(createCanFrame(0x200 + motorID, {
            static_cast<uint8_t>((positionValue >> 24) & 0xFF),
            static_cast<uint8_t>((positionValue >> 16) & 0xFF),
            static_cast<uint8_t>((positionValue >> 8) & 0xFF),
            static_cast<uint8_t>(positionValue & 0xFF),
            static_cast<uint8_t>((controlWord >> 8) & 0xFF),
            static_cast<uint8_t>(controlWord & 0xFF)
        }, 6));
        
        // RPDO2: 目标速度 + 目标电流
        testFrames.push_back(createCanFrame(0x300 + motorID, {
            static_cast<uint8_t>((velocityValue >> 8) & 0xFF),
            static_cast<uint8_t>(velocityValue & 0xFF),
            static_cast<uint8_t>((currentValue >> 8) & 0xFF),
            static_cast<uint8_t>(currentValue & 0xFF)
        }, 4));
        
        // TPDO1: 实际位置 + 状态字
        testFrames.push_back(createCanFrame(0x180 + motorID, {
            static_cast<uint8_t>((positionValue >> 24) & 0xFF),
            static_cast<uint8_t>((positionValue >> 16) & 0xFF),
            static_cast<uint8_t>((positionValue >> 8) & 0xFF),
            static_cast<uint8_t>(positionValue & 0xFF),
            static_cast<uint8_t>((statusWord >> 8) & 0xFF),
            static_cast<uint8_t>(statusWord & 0xFF)
        }, 6));
        
        // TPDO2: 实际速度 + 实际电流
        testFrames.push_back(createCanFrame(0x280 + motorID, {
            static_cast<uint8_t>((velocityValue >> 8) & 0xFF),
            static_cast<uint8_t>(velocityValue & 0xFF),
            static_cast<uint8_t>((currentValue >> 8) & 0xFF),
            static_cast<uint8_t>(currentValue & 0xFF)
        }, 4));
    }
    
    // 添加同步帧作为结束标志
    testFrames.push_back(createCanFrame(0x080, {0x00}, 1));
    
    std::cout << "生成了 " << testFrames.size() << " 个测试帧" << std::endl;
    std::cout << "开始真正的批量发送测试..." << std::endl;
    
    // 开始计时（纳秒精度）
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // 使用真正的批量发送方法
    size_t successCount = serial.sendFramesBatch(testFrames);
    
    // 结束计时并计算耗时（纳秒转换为微秒）
    auto endTime = std::chrono::high_resolution_clock::now();
    auto durationNs = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime);
    double elapsedTimeUs = durationNs.count() / 1000.0; // 纳秒转换为微秒
    
    size_t failCount = testFrames.size() - successCount;
    
    std::cout << "真正的批量发送完成 - 成功: " << successCount 
              << ", 失败: " << failCount << std::endl;
    std::cout << "批量发送总耗时: " << std::fixed << std::setprecision(2) 
              << elapsedTimeUs << " us" << std::endl;
    std::cout << "平均每帧耗时: " << std::fixed << std::setprecision(2) 
              << (successCount > 0 ? elapsedTimeUs / successCount : 0) << " us/帧" << std::endl;
    std::cout << "批量发送吞吐量: " << std::fixed << std::setprecision(2) 
              << (successCount > 0 ? (successCount * 1000000.0) / elapsedTimeUs : 0) << " 帧/秒" << std::endl;
    
    serial.disconnect();
    std::cout << "真正的批量发送测试结束" << std::endl;
}

#endif // TEST_SERIAL_MODULE_HPP