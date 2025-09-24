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
 * @brief sendBufferSync方法测试函数
 * 
 * @details 测试SerialPortManager的sendBufferSync方法，使用环形缓冲区进行批量发送。
 *          自动生成6个电机的所有PDO帧，填充到环形缓冲区，然后使用sendBufferSync
 *          方法进行同步批量发送。
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
    
    // 创建环形缓冲区
    CircularBuffer buffer;
    
    // 生成所有电机的PDO测试帧并填充到缓冲区
    size_t totalFrames = 0;
    
    // 为6个电机生成PDO帧
    for (uint8_t motorID = 1; motorID <= 6; ++motorID) {
        // 为每个电机生成测试数据（递增的值）
        uint32_t positionValue = 1000 * motorID;
        uint16_t velocityValue = 500 * motorID;
        uint16_t currentValue = 100 * motorID;
        uint16_t controlWord = 0x0006; // 标准控制字
        uint16_t statusWord = 0x0237;  // 标准状态字
        
        // 创建CAN帧并添加到缓冲区
        CanFrame rpdo1 = createCanFrame(0x200 + motorID, {
            static_cast<uint8_t>((positionValue >> 24) & 0xFF),
            static_cast<uint8_t>((positionValue >> 16) & 0xFF),
            static_cast<uint8_t>((positionValue >> 8) & 0xFF),
            static_cast<uint8_t>(positionValue & 0xFF),
            static_cast<uint8_t>((controlWord >> 8) & 0xFF),
            static_cast<uint8_t>(controlWord & 0xFF)
        }, 6);
        
        CanFrame rpdo2 = createCanFrame(0x300 + motorID, {
            static_cast<uint8_t>((velocityValue >> 8) & 0xFF),
            static_cast<uint8_t>(velocityValue & 0xFF),
            static_cast<uint8_t>((currentValue >> 8) & 0xFF),
            static_cast<uint8_t>(currentValue & 0xFF)
        }, 4);
        
        CanFrame tpdo1 = createCanFrame(0x180 + motorID, {
            static_cast<uint8_t>((positionValue >> 24) & 0xFF),
            static_cast<uint8_t>((positionValue >> 16) & 0xFF),
            static_cast<uint8_t>((positionValue >> 8) & 0xFF),
            static_cast<uint8_t>(positionValue & 0xFF),
            static_cast<uint8_t>((statusWord >> 8) & 0xFF),
            static_cast<uint8_t>(statusWord & 0xFF)
        }, 6);
        
        CanFrame tpdo2 = createCanFrame(0x280 + motorID, {
            static_cast<uint8_t>((velocityValue >> 8) & 0xFF),
            static_cast<uint8_t>(velocityValue & 0xFF),
            static_cast<uint8_t>((currentValue >> 8) & 0xFF),
            static_cast<uint8_t>(currentValue & 0xFF)
        }, 4);
        
        // 添加到缓冲区
        if (buffer.pushFrame(rpdo1)) totalFrames++;
        if (buffer.pushFrame(rpdo2)) totalFrames++;
        if (buffer.pushFrame(tpdo1)) totalFrames++;
        if (buffer.pushFrame(tpdo2)) totalFrames++;
    }
    
    // 添加同步帧作为结束标志
    CanFrame syncFrame = createCanFrame(0x080, {0x00}, 1);
    if (buffer.pushFrame(syncFrame)) totalFrames++;
    
    std::cout << "缓冲区填充完成 - 总帧数: " << totalFrames << " 帧" << std::endl;
    std::cout << "缓冲区使用空间: " << buffer.getUsedSpace() << " 字节" << std::endl;
    std::cout << "可用完整帧数: " << buffer.getAvailableFrames() << " 帧" << std::endl;
    std::cout << "开始使用sendBufferSync进行批量发送..." << std::endl;
    
    // 开始计时（纳秒精度）
    auto startTime = std::chrono::high_resolution_clock::now();

    size_t bytesSent = 0;
    size_t allsendbytes = 0;


    
    // 使用sendBufferSync方法进行批量发送
    for (size_t i = 0; i < 25; i++)
    {
        bytesSent = serial.sendBufferSync(buffer, 13, false); // -1表示发送所有帧，true表示发送后清空缓冲区
        allsendbytes += bytesSent;
    }
    

    //allsendbytes = serial.sendBufferSyncAuto(buffer);
    
    size_t framesSent = allsendbytes / CAN_FRAME_SIZE ;
    
    // 结束计时并计算耗时（纳秒转换为微秒）
    auto endTime = std::chrono::high_resolution_clock::now();
    auto durationNs = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime);
    double elapsedTimeUs = durationNs.count() / 1000.0; // 纳秒转换为微秒
    
    std::cout << "sendBufferSync批量发送完成" << std::endl;
    std::cout << "发送字节数: " << bytesSent << " 字节" << std::endl;
    std::cout << "发送帧数: " << framesSent << " 帧" << std::endl;
    std::cout << "批量发送总耗时: " << std::fixed << std::setprecision(2) 
              << elapsedTimeUs << " us" << std::endl;
    std::cout << "平均每帧耗时: " << std::fixed << std::setprecision(2) 
              << (framesSent > 0 ? elapsedTimeUs / framesSent : 0) << " us/帧" << std::endl;
    std::cout << "吞吐量: " << std::fixed << std::setprecision(2) 
              << (framesSent > 0 ? (framesSent * 1000000.0) / elapsedTimeUs : 0) << " 帧/秒" << std::endl;
    
    // 验证完整性
    if (bytesSent % CAN_FRAME_SIZE != 0) {
        std::cerr << "[WARN][testSerialBatchSend]: 发送字节数不是13的倍数，可能存在帧完整性问题" << std::endl;
    }
    
    if (framesSent != totalFrames) {
        std::cerr << "[WARN][testSerialBatchSend]: 发送帧数(" << framesSent 
                  << ")与预期帧数(" << totalFrames << ")不一致" << std::endl;
    }
    
    serial.disconnect();
    std::cout << "sendBufferSync批量发送测试结束" << std::endl;
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

/**
 * @brief 串口接收测试函数
 *
 * @details 测试SerialPortManager的receiveToBuffer方法，创建一个简易的接收功能，
 *          每隔10秒显示接收到的数据统计，直到按回车退出。
 *
 * @param portName 串口设备名称，默认"COM1"
 */
inline void testSerialReceive(const std::string& portName = "COM1") {
    SerialPortManager serial;

    // 连接串口
    if (!serial.connect(portName)) {
        std::cerr << "[ERROR][testSerialReceive]: 串口连接失败 - " << portName << std::endl;
        return;
    }

    std::cout << "[INFO][testSerialReceive]: 串口连接成功" << std::endl;

    // 创建环形缓冲区用于接收数据
    CircularBuffer receiveBuffer;

    // 统计信息
    std::atomic<uint64_t> totalBytesReceived{0};
    std::atomic<uint64_t> totalFramesReceived{0};
    std::atomic<bool> running{true};

    // 接收线程
    std::thread receiveThread([&]() {
        std::cout << "[INFO][testSerialReceive]: 接收线程启动" << std::endl;

        while (running.load()) {
            // 使用receiveToBuffer接收数据
            size_t bytesRead = serial.receiveToBuffer(receiveBuffer);

            if (bytesRead > 0) {
                totalBytesReceived += bytesRead;

                // 计算接收到的完整帧数
                size_t framesReceived = bytesRead / CAN_FRAME_SIZE;
                totalFramesReceived += framesReceived;

                // 简要显示接收信息
                std::cout << "[DEBUG][testSerialReceive]: 接收 " << bytesRead
                          << " 字节 (" << framesReceived << " 帧)" << std::endl;
            }

            // 短暂休眠避免CPU占用过高
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        std::cout << "[INFO][testSerialReceive]: 接收线程结束" << std::endl;
    });

    // 统计显示线程
    std::thread statsThread([&]() {
        std::cout << "[INFO][testSerialReceive]: 统计线程启动" << std::endl;

        while (running.load()) {
            // 等待10秒
            for (int i = 0; i < 100 && running.load(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (!running.load()) break;

            // 显示统计信息
            uint64_t bytes = totalBytesReceived.load();
            uint64_t frames = totalFramesReceived.load();
            uint32_t bufferSize = receiveBuffer.getUsedSpace();
            uint32_t availableFrames = receiveBuffer.getAvailableFrames();

            std::cout << "\n[STATS][testSerialReceive]: ========================" << std::endl;
            std::cout << "[STATS][testSerialReceive]: 累计接收统计:" << std::endl;
            std::cout << "[STATS][testSerialReceive]:   总字节数: " << bytes << std::endl;
            std::cout << "[STATS][testSerialReceive]:   总帧数: " << frames << std::endl;
            std::cout << "[STATS][testSerialReceive]:   缓冲区使用: " << bufferSize << " 字节" << std::endl;
            std::cout << "[STATS][testSerialReceive]:   可用完整帧: " << availableFrames << " 帧" << std::endl;
            std::cout << "[STATS][testSerialReceive]: =========================" << std::endl;

            // 显示缓冲区内容预览（如果有数据）
            if (availableFrames > 0) {
                std::cout << "[STATS][testSerialReceive]: 缓冲区内容预览:" << std::endl;

                // 创建临时缓冲区读取前几帧
                std::array<uint8_t, 13 * 3> previewBuffer; // 最多显示3帧
                size_t previewSize = std::min(static_cast<size_t>(availableFrames * CAN_FRAME_SIZE),
                                            previewBuffer.size());

                if (receiveBuffer.popBytes(previewBuffer.data(), previewSize) > 0) {
                    for (size_t i = 0; i < previewSize; i += CAN_FRAME_SIZE) {
                        if (i + CAN_FRAME_SIZE <= previewSize) {
                            uint8_t dlc = previewBuffer[i];
                            uint32_t frameID = (previewBuffer[i+1] << 24) |
                                              (previewBuffer[i+2] << 16) |
                                              (previewBuffer[i+3] << 8) |
                                              previewBuffer[i+4];

                            std::cout << "[STATS][testSerialReceive]:   帧 " << (i/CAN_FRAME_SIZE + 1)
                                      << ": ID=0x" << std::hex << frameID
                                      << ", DLC=" << std::dec << static_cast<int>(dlc) << std::endl;
                        }
                    }
                }
            }

            std::cout << std::endl;
        }

        std::cout << "[INFO][testSerialReceive]: 统计线程结束" << std::endl;
    });

    std::cout << "[INFO][testSerialReceive]: 接收测试已启动" << std::endl;
    std::cout << "[INFO][testSerialReceive]: 每10秒显示接收统计信息" << std::endl;
    std::cout << "[INFO][testSerialReceive]: 按回车键退出测试..." << std::endl;

    // 等待用户输入回车键退出
    std::cin.get();

    // 停止所有线程
    running.store(false);

    // 等待线程结束
    receiveThread.join();
    statsThread.join();

    // 显示最终统计
    std::cout << "\n[FINAL][testSerialReceive]: 最终统计:" << std::endl;
    std::cout << "[FINAL][testSerialReceive]:   总接收字节数: " << totalBytesReceived.load() << std::endl;
    std::cout << "[FINAL][testSerialReceive]:   总接收帧数: " << totalFramesReceived.load() << std::endl;
    std::cout << "[FINAL][testSerialReceive]:   缓冲区剩余: " << receiveBuffer.getUsedSpace() << " 字节" << std::endl;

    serial.disconnect();
    std::cout << "[INFO][testSerialReceive]: 接收测试结束" << std::endl;
}

#endif // TEST_SERIAL_MODULE_HPP