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

// 符合CLAUDE.md的测试标准：将整个测试流程包装到单一函数内
#include "test/test_Send_Thread_PDO.hpp"
#include "Serial_Module.hpp"

/**
 * @brief 主测试函数 - 符合CLAUDE.md测试标准
 *
 * @details 根据CLAUDE.md要求实现的统一测试函数，包含以下特点：
 * - 将整个测试流程包装到单一函数内
 * - 确保测试过程只需要调用一个函数就能完成测试
 * - 提供完善的DEBUG输出，特别是在性能敏感的部分
 * - 测试涵盖各种分支条件和异常状况
 * - 专注于PDO功能测试，不包含SDO测试
 * - 使用UTF8 with BOM编码和中文输出
 *
 * @return int 测试结果：0表示成功，非0表示失败
 */
int runAllTests() {
    std::cout << "========================================" << std::endl;
    std::cout << "   CANopen DS402机械臂驱动程序测试" << std::endl;
    std::cout << "========================================" << std::endl;

    int testResult = 0;
    auto startTime = std::chrono::steady_clock::now();

    try {
        // 测试1: 发送线程PDO功能测试
        std::cout << "\n[TEST]: 开始发送线程PDO功能测试..." << std::endl;

        bool pdoTestResult = testSendThreadPdoFunctionality();

        if (pdoTestResult) {
            std::cout << "[SUCCESS]: 发送线程PDO功能测试通过" << std::endl;
        } else {
            std::cout << "[ERROR]: 发送线程PDO功能测试失败" << std::endl;
            testResult = 1;  // 设置失败标志
        }

    } catch (const std::exception& e) {
        std::cout << "[ERROR][main]: 测试过程中发生异常: " << e.what() << std::endl;
        testResult = 1;
    }

    // 输出测试统计信息
    auto endTime = std::chrono::steady_clock::now();
    auto totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "\n========================================" << std::endl;
    std::cout << "            测试总结" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "测试结果: " << (testResult == 0 ? "成功" : "失败") << std::endl;
    std::cout << "总耗时: " << totalDuration.count() << " ms" << std::endl;
    std::cout << "========================================" << std::endl;

    return testResult;
}

/**
 * @brief 主函数
 *
 * @details 程序入口点，调用完整的测试流程
 * - 设置系统locale以支持中文输出
 * - 调用runAllTests()执行所有测试
 * - 返回测试结果给操作系统
 *
 * @return int 程序退出码（0=成功，非0=失败）
 */
int main() {
    // 设置locale以支持中文输出（符合UTF8 with BOM要求）
    std::locale::global(std::locale(""));  // 使用系统默认locale
    std::wcout.imbue(std::locale());

    std::cout << "CANopen DS402机械臂驱动程序 - PDO测试环境" << std::endl;
    std::cout << "基于CANopen协议的DS402子协议" << std::endl;
    std::cout << "专注于PDO功能测试，不包含SDO测试" << std::endl;
    std::cout << "文件编码: UTF-8 with BOM" << std::endl;
    std::cout << std::endl;

    // 调用完整的测试流程（包括串口管理器基础功能测试）
    std::cout << "========================================" << std::endl;
    std::cout << "   完整测试流程" << std::endl;
    std::cout << "========================================" << std::endl;

    try {
        int testResult = runAllTests();

        std::cout << "\n[INFO][main]: 程序执行完成，退出码: " << testResult << std::endl;
        return testResult;
    } catch (const std::exception& e) {
        std::cout << "[ERROR][main]: 程序执行过程中发生异常: " << e.what() << std::endl;
        return 1;
    }
}