#ifndef SEND_FRAME_HPP
#define SEND_FRAME_HPP

#include <boost/asio.hpp>
#include <iostream>
#include <iomanip>
#include <string>
#include "CAN_frame.hpp" // 引入上面的 CAN_frame.hpp，才能用到 CanFrame

// 如果需要在外部统一控制调试开关，可以把这里的宏注释掉，
// 并在编译命令或其他头文件中自行定义/取消定义 DEBUG_MODE。
//#ifndef DEBUG_MODE
#define DEBUG_MODE 0
//#endif

/*******************************************************************
 *
 *  send_frame.hpp
 *  负责操作串口、初始化串口、以及发送帧的功能
 *
 ******************************************************************/

 /**
  * @brief 初始化并返回一个已配置的串口对象
  * @param ioService Boost.Asio I/O 服务对象引用
  * @param portName  串口设备名称 (如 "COM1" 或 "/dev/ttyUSB0")
  * @param baudRate  串口的波特率，默认 3000000
  * @return          已配置的 Boost.Asio serial_port 对象
  */
boost::asio::serial_port initializeSerialPort(boost::asio::io_service& ioService,
    const std::string& portName,
    unsigned int baudRate = 3000000);

/**
 * @brief 将 CAN 帧的二进制数据发送到指定串口
 * @param serialPort 已初始化的串口对象
 * @param frame      待发送的 CAN 帧数据
 */
void sendDataToSerial(boost::asio::serial_port& serialPort, const CanFrame& frame);

/**
 * @brief 通过串口发送 CAN 帧数据
 * @param portName 串口设备名称 (如 "COM1" 或 "/dev/ttyUSB0")
 * @param frame    待发送的 CAN 帧数据
 * @param baudRate 串口波特率，默认 3000000
 */
void sendBinaryFrame(const std::string& portName, const CanFrame& frame, unsigned int baudRate = 3000000);

/*******************************************************************
 *                    以下为实现部分 (同头文件内联)
 ******************************************************************/

inline boost::asio::serial_port initializeSerialPort(boost::asio::io_service& ioService,
    const std::string& portName,
    unsigned int baudRate) {
    // 创建串口对象
    boost::asio::serial_port serialPort(ioService, portName);
    // 设置波特率
    serialPort.set_option(boost::asio::serial_port_base::baud_rate(baudRate));

#if DEBUG_MODE
    std::cout << "[DEBUG] Serial port initialized: "
        << portName
        << " with baud rate: "
        << baudRate
        << std::endl;
#endif 

    return serialPort;
}

inline void sendDataToSerial(boost::asio::serial_port& serialPort, const CanFrame& frame) {
    // 获取 CAN 帧的二进制数据
    const std::vector<uint8_t>& binaryData = frame.getBinaryFrame();
    // 发送数据
    boost::asio::write(serialPort, boost::asio::buffer(binaryData.data(), binaryData.size()));

#if DEBUG_MODE
    // 输出调试信息：显示发送的字节数据
    std::cout << "[DEBUG] Frame sent successfully. Bytes: ";
    for (uint8_t byte : binaryData) {
        std::cout << std::hex << std::setw(2)
            << std::setfill('0')
            << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;
#endif
}

inline void sendBinaryFrame(const std::string& portName,
    const CanFrame& frame,
    unsigned int baudRate) {
    try {
        // 1. 初始化 Boost.Asio I/O 服务
        boost::asio::io_service ioService;
        // 2. 初始化串口
        boost::asio::serial_port serialPort = initializeSerialPort(ioService, portName, baudRate);
        // 3. 发送数据到串口
        sendDataToSerial(serialPort, frame);
    }
    catch (const std::exception& e) {
#if DEBUG_MODE
        std::cerr << "[DEBUG] Error: " << e.what() << std::endl;
#endif
    }
}

#endif // SEND_FRAME_HPP

#undef DEBUG_MODE