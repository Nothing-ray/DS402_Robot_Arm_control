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

#endif // TEST_SERIAL_MODULE_HPP