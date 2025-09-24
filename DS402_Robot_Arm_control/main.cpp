#include <cstdint>
#include <array>
#include <string>
#include <cmath>
#include <mutex>
#include <atomic>
#include <boost/asio.hpp>
#include <thread>
#include <chrono>
#include <stdint.h>
#include <type_traits>
#include <locale>
#include <codecvt>
#include <iostream>

// 串口接收测试和无锁发送测试
#include "test/test_Serial_Module.hpp"
#include "Serial_Module.hpp"

/**
 * @brief 主函数
 *
 * @details 程序入口点，直接启动串口接收测试
 * - 设置系统locale以支持中文输出
 * - 指定串口标识符并启动接收测试
 * - 每10秒显示接收统计信息，按回车键退出
 *
 * @return int 程序退出码（0=成功，非0=失败）
 */
int main() {
    // 设置locale以支持中文输出（符合UTF8 with BOM要求）
    std::locale::global(std::locale(""));  // 使用系统默认locale
    std::wcout.imbue(std::locale());

    std::cout << "CANopen DS402机械臂驱动程序 - 串口接收测试" << std::endl;
    std::cout << "基于CANopen协议的DS402子协议" << std::endl;
    std::cout << "文件编码: UTF-8 with BOM" << std::endl;
    std::cout << std::endl;

    // 指定要连接的串口标识符
    const std::string portName = "COM1";  // 可以根据需要修改为 "COM2", "COM3" 等

    std::cout << "========================================" << std::endl;
    std::cout << "   串口接收功能测试" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "目标串口: " << portName << std::endl;
    std::cout << std::endl;

    try {
        std::cout << "[INFO][main]: 开始串口接收功能测试..." << std::endl;
        std::cout << "[INFO][main]: 此测试将启动接收线程，每10秒显示统计信息" << std::endl;
        std::cout << "[INFO][main]: 按回车键可退出接收测试" << std::endl;
        std::cout << std::endl;

        // 调用接收测试函数
        testSerialReceive(portName);

        std::cout << "[INFO][main]: 串口接收功能测试完成" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cout << "[ERROR][main]: 程序执行过程中发生异常: " << e.what() << std::endl;
        return 1;
    }
}