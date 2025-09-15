#ifndef TEST_SERIAL_MANAGER_BASIC_HPP
#define TEST_SERIAL_MANAGER_BASIC_HPP

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <memory>
#include <vector>

#include "CAN_frame.hpp"
#include "Serial_Module.hpp"
#include "CircularBuffer.hpp"

// 启用串口调试输出
#define ENABLE_SERIAL_DEBUG

/**
 * @brief 串口管理器基础功能测试
 *
 * @details 在Send_Thread使用串口管理器之前，独立测试串口管理器的各项功能
 * 用于确定问题出现在串口管理器本身还是后续的集成环节
 *
 * 测试内容包括：
 * 1. 串口连接测试
 * 2. 基本CAN帧构造测试
 * 3. 单帧发送测试
 * 4. 批量发送测试
 * 5. 环形缓冲区集成测试
 *
 * @return bool 测试结果：true表示测试通过，false表示测试失败
 */
bool testSerialManagerBasicFunctionality() {
    std::cout << "========================================" << std::endl;
    std::cout << "   串口管理器基础功能测试" << std::endl;
    std::cout << "========================================" << std::endl;

    bool allTestsPassed = true;

    // 测试1：串口连接测试
    std::cout << "\n[TEST 1]: 串口连接测试..." << std::endl;
    SerialPortManager serialManager;

    if (!serialManager.connect("COM3")) {
        std::cerr << "[ERROR][TEST 1]: 串口连接失败 - COM3" << std::endl;
        std::cerr << "[INFO][TEST 1]: 这可能是由于没有硬件设备导致的，继续测试其他功能..." << std::endl;
        allTestsPassed = false;
    } else {
        std::cout << "[SUCCESS][TEST 1]: 串口连接成功" << std::endl;
    }

    // 测试2：基本CAN帧构造测试
    std::cout << "\n[TEST 2]: 基本CAN帧构造测试..." << std::endl;

    try {
        // 构造一个标准的SDO请求帧（读操作）
        uint8_t sdoData[8] = {0x40, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00}; // 读索引0x6016子索引0x00
        CanFrame sdoFrame(0x601, sdoData, 8); // Node ID = 1

        const auto& binaryFrame = sdoFrame.getBinaryFrame();

        std::cout << "[DEBUG][TEST 2]: CAN帧构造成功" << std::endl;
        std::cout << "[DEBUG][TEST 2]: 帧ID: 0x" << std::hex << sdoFrame.frameID << std::dec << std::endl;
        std::cout << "[DEBUG][TEST 2]: 帧长度: " << static_cast<int>(sdoFrame.dlc) << " 字节" << std::endl;
        std::cout << "[DEBUG][TEST 2]: 二进制数据: ";
        for (uint8_t byte : binaryFrame) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(byte) << " ";
        }
        std::cout << std::dec << std::endl;

        // 验证帧格式
        if (binaryFrame.size() != 13) {
            std::cerr << "[ERROR][TEST 2]: CAN帧长度错误，预期13字节，实际" << binaryFrame.size() << "字节" << std::endl;
            allTestsPassed = false;
        } else {
            std::cout << "[SUCCESS][TEST 2]: CAN帧构造正确" << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "[ERROR][TEST 2]: CAN帧构造异常: " << e.what() << std::endl;
        allTestsPassed = false;
    }

    // 测试3：单帧发送测试（仅在有连接时进行）
    std::cout << "\n[TEST 3]: 单帧发送测试..." << std::endl;

    if (serialManager.isConnected()) {
        try {
            // 构造一个测试帧
            uint8_t testData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
            CanFrame testFrame(0x123, testData, 8);

            std::cout << "[DEBUG][TEST 3]: 尝试发送测试帧..." << std::endl;
            bool sendResult = serialManager.sendFrame(testFrame);

            if (sendResult) {
                std::cout << "[SUCCESS][TEST 3]: 单帧发送成功" << std::endl;
            } else {
                std::cerr << "[ERROR][TEST 3]: 单帧发送失败" << std::endl;
                allTestsPassed = false;
            }

        } catch (const std::exception& e) {
            std::cerr << "[ERROR][TEST 3]: 单帧发送异常: " << e.what() << std::endl;
            allTestsPassed = false;
        }
    } else {
        std::cout << "[INFO][TEST 3]: 串口未连接，跳过单帧发送测试" << std::endl;
    }

    // 测试4：批量发送测试（仅在有连接时进行）
    std::cout << "\n[TEST 4]: 批量发送测试..." << std::endl;

    if (serialManager.isConnected()) {
        try {
            // 构造多个测试帧
            std::vector<CanFrame> testFrames;
            for (int i = 0; i < 3; ++i) {
                uint8_t frameData[8] = {
                    static_cast<uint8_t>(0x10 + i),
                    static_cast<uint8_t>(0x20 + i),
                    static_cast<uint8_t>(0x30 + i),
                    static_cast<uint8_t>(0x40 + i),
                    static_cast<uint8_t>(0x50 + i),
                    static_cast<uint8_t>(0x60 + i),
                    static_cast<uint8_t>(0x70 + i),
                    static_cast<uint8_t>(0x80 + i)
                };
                testFrames.emplace_back(0x200 + i, frameData, 8);
            }

            std::cout << "[DEBUG][TEST 4]: 尝试批量发送" << testFrames.size() << "帧..." << std::endl;
            size_t successCount = serialManager.sendFramesBatch(testFrames);

            if (successCount == testFrames.size()) {
                std::cout << "[SUCCESS][TEST 4]: 批量发送成功，成功" << successCount << "/" << testFrames.size() << "帧" << std::endl;
            } else {
                std::cerr << "[ERROR][TEST 4]: 批量发送失败，成功" << successCount << "/" << testFrames.size() << "帧" << std::endl;
                allTestsPassed = false;
            }

        } catch (const std::exception& e) {
            std::cerr << "[ERROR][TEST 4]: 批量发送异常: " << e.what() << std::endl;
            allTestsPassed = false;
        }
    } else {
        std::cout << "[INFO][TEST 4]: 串口未连接，跳过批量发送测试" << std::endl;
    }

    // 测试5：环形缓冲区集成测试
    std::cout << "\n[TEST 5]: 环形缓冲区集成测试..." << std::endl;

    try {
        CircularBuffer testBuffer;

        // 构造测试帧并放入缓冲区
        for (int i = 0; i < 3; ++i) {
            uint8_t frameData[8] = {
                static_cast<uint8_t>(0xA0 + i),
                static_cast<uint8_t>(0xB0 + i),
                static_cast<uint8_t>(0xC0 + i),
                static_cast<uint8_t>(0xD0 + i),
                static_cast<uint8_t>(0xE0 + i),
                static_cast<uint8_t>(0xF0 + i),
                static_cast<uint8_t>(0x01 + i),
                static_cast<uint8_t>(0x02 + i)
            };
            CanFrame frame(0x300 + i, frameData, 8);

            if (!testBuffer.pushFrame(frame)) {
                std::cerr << "[ERROR][TEST 5]: 缓冲区写入失败" << std::endl;
                allTestsPassed = false;
                break;
            }
        }

        std::cout << "[DEBUG][TEST 5]: 缓冲区状态 - 已用: " << testBuffer.getUsedSpace()
                  << ", 可用帧数: " << testBuffer.getAvailableFrames() << std::endl;

        // 测试sendBufferSync功能
        if (serialManager.isConnected() && !testBuffer.isEmpty()) {
            std::cout << "[DEBUG][TEST 5]: 测试sendBufferSync功能..." << std::endl;

            size_t bytesSent = serialManager.sendBufferSync(testBuffer, 13, false); // 发送1帧

            if (bytesSent == 13) {
                std::cout << "[SUCCESS][TEST 5]: sendBufferSync发送成功，发送" << bytesSent << "字节" << std::endl;
            } else {
                std::cerr << "[ERROR][TEST 5]: sendBufferSync发送失败，预期13字节，实际" << bytesSent << "字节" << std::endl;
                allTestsPassed = false;
            }
        } else if (!serialManager.isConnected()) {
            std::cout << "[INFO][TEST 5]: 串口未连接，跳过sendBufferSync测试" << std::endl;
        }

        // 测试sendBufferSyncAuto功能
        if (serialManager.isConnected() && !testBuffer.isEmpty()) {
            std::cout << "[DEBUG][TEST 5]: 测试sendBufferSyncAuto功能..." << std::endl;

            size_t remainingFrames = testBuffer.getAvailableFrames();
            size_t bytesSent = serialManager.sendBufferSyncAuto(testBuffer, true); // 发送剩余所有帧

            size_t expectedBytes = remainingFrames * 13;
            if (bytesSent == expectedBytes) {
                std::cout << "[SUCCESS][TEST 5]: sendBufferSyncAuto发送成功，发送" << bytesSent << "字节" << std::endl;
            } else {
                std::cerr << "[ERROR][TEST 5]: sendBufferSyncAuto发送失败，预期" << expectedBytes
                          << "字节，实际" << bytesSent << "字节" << std::endl;
                allTestsPassed = false;
            }
        } else if (!serialManager.isConnected()) {
            std::cout << "[INFO][TEST 5]: 串口未连接，跳过sendBufferSyncAuto测试" << std::endl;
        }

        std::cout << "[SUCCESS][TEST 5]: 环形缓冲区集成测试完成" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[ERROR][TEST 5]: 环形缓冲区集成测试异常: " << e.what() << std::endl;
        allTestsPassed = false;
    }

    // 测试总结
    std::cout << "\n========================================" << std::endl;
    std::cout << "   串口管理器基础功能测试总结" << std::endl;
    std::cout << "========================================" << std::endl;

    if (allTestsPassed) {
        std::cout << "[SUCCESS]: 所有测试通过，串口管理器设计正常" << std::endl;
    } else {
        std::cout << "[ERROR]: 部分测试失败，串口管理器可能存在问题" << std::endl;

        if (!serialManager.isConnected()) {
            std::cout << "[INFO]: 注意：串口未连接可能导致部分测试失败" << std::endl;
            std::cout << "[INFO]: 建议检查硬件连接或使用虚拟串口进行测试" << std::endl;
        }
    }

    std::cout << "========================================" << std::endl;

    return allTestsPassed;
}

#endif // TEST_SERIAL_MANAGER_BASIC_HPP