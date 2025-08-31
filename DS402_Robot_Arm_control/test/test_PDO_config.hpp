/**
 * @file test_PDO_config.hpp
 * @brief PDO配置和映射测试
 * 
 * 包含PDO映射表构建、COB-ID转换、边界值处理、TPDO数据处理等测试
 */

#ifndef TEST_PDO_CONFIG_HPP
#define TEST_PDO_CONFIG_HPP

#include <array>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <algorithm>

#include <random>       // 用于 std::mt19937, std::random_device, std::uniform_int_distribution
#include <atomic>       // 用于 std::atomic
#include <vector>       // 用于 std::vector
#include <numeric>      // 用于 std::accumulate


#include "../CLASS_Motor.hpp"
#include "../PDO_config.hpp"
#include "../Data_processing.hpp"
#include "../CAN_frame.hpp"
#include "../CAN_processing.hpp"
#include "../SDO_State_Machine.hpp"
using namespace std::chrono_literals;


// 计时工具宏
#define TIME_IT(operation, description) \
    do { \
        auto start = std::chrono::high_resolution_clock::now(); \
        operation; \
        auto end = std::chrono::high_resolution_clock::now(); \
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start); \
        std::cout << description << " took " << duration.count() << " us\n"; \
    } while(0)







/******************** PDO配置测试 ********************/



/**
 * @brief 测试PDO配置系统的完整功能
 *
 * 本测试函数执行以下验证：
 * 1. 测试映射表初始化功能，包括构建时间和内容完整性
 * 2. 测试COB-ID转换工具的正确性和异常处理
 * 3. 使用高精度计时器测量关键操作性能
 *
 * 测试包含以下场景：
 * - 正常电机ID(1-12)的转换
 * - 边界值测试(最小/最大有效ID)
 * - 无效输入测试(0/13等非法ID)
 *
 * @note 测试结果将输出到标准输出，错误信息输出到标准错误
 * @warning 测试中将故意触发异常，相关错误信息属于预期行为
 */
void testPDOConfiguration() {
    std::cout << "=== PDO配置测试开始 ===\n\n";

    /* 第一阶段：映射表初始化测试 */
    std::vector<PdoMappingEntry> mappingTable;

    // 使用计时宏测量6电机映射表构建时间
    TIME_IT(
        mappingTable = buildArmMappingTable(6),
        "构建6电机映射表"
    );

    /* 打印映射表详细内容 */
    std::cout << "\n=== 映射表内容 ===\n";
    for (const auto& entry : mappingTable) {
        // 格式化输出每个映射条目信息：
        // 1. 电机编号 | 2. PDO类型及通道 | 3. 对象字典地址
        // 4. 数据帧偏移 | 5. 数据大小 | 6. Motor类成员偏移
        std::cout << "电机" << (int)entry.motorIndex
            << " | " << (entry.isTx ? "TPDO" : "RPDO") << (int)entry.pdoIndex
            << " | 对象字典: 0x" << std::hex << entry.index << std::dec << "/" << (int)entry.subIndex
            << " | 帧偏移: " << (int)entry.offsetInPdo
            << " | 大小: " << (int)entry.size << " 字节"
            << " | 电机偏移: " << entry.motorFieldOffset
            << "\n";
    }
    std::cout << "映射条目总数: " << mappingTable.size() << "\n\n";

    /* 第二阶段：COB-ID转换工具测试 */
    std::cout << "=== COB-ID转换测试 ===\n";

    // 定义COB-ID测试lambda函数，封装重复测试逻辑
    auto testCobIdConversion = [](uint8_t motorId) {
        std::cout << "电机" << (int)motorId << ":\n";

        try {
            // 测试SDO ID转换
            std::cout << "  SDO ID: 0x" << std::hex << toSdoMotorId(motorId) << std::dec << "\n";

            // 测试所有PDO通道(1-4)的RPDO/TPDO转换
            for (uint8_t pdo = 1; pdo <= 4; ++pdo) {
                std::cout << "  RPDO" << (int)pdo << ": 0x" << std::hex
                    << toRpdoCobId(motorId, pdo) << std::dec << "\n";
                std::cout << "  TPDO" << (int)pdo << ": 0x" << std::hex
                    << toTpdoCobId(motorId, pdo) << std::dec << "\n";
            }
        }
        catch (const std::exception& e) {
            // 捕获并报告转换异常
            std::cerr << "  错误: " << e.what() << "\n";
        }
        };

    /* 测试用例1：正常值测试（电机1） */
    TIME_IT(
        testCobIdConversion(1),
        "电机1的COB-ID转换"
    );

    /* 测试用例2：边界值测试（电机12，最大值） */
    std::cout << "\n边界值测试:\n";
    TIME_IT(
        testCobIdConversion(12),
        "电机12的COB-ID转换(最大值)"
    );

    /* 测试用例3：异常值测试 */
    std::cout << "\n无效值测试:\n";

    // 子用例3.1：电机ID为0（非法值）
    try {
        TIME_IT(
            toSdoMotorId(0),
            "无效电机ID 0"
        );
    }
    catch (const std::exception& e) {
        std::cerr << "预期错误: " << e.what() << "\n";
    }

    // 子用例3.2：电机ID为13（超出上限）
    try {
        TIME_IT(
            toRpdoCobId(13, 1),
            "无效电机ID 13"
        );
    }
    catch (const std::exception& e) {
        std::cerr << "预期错误: " << e.what() << "\n";
    }

    // 子用例3.3：PDO索引为5（非法值）
    try {
        TIME_IT(
            toTpdoCobId(1, 5),
            "无效PDO索引 5"
        );
    }
    catch (const std::exception& e) {
        std::cerr << "预期错误: " << e.what() << "\n";
    }

    std::cout << "\n=== PDO配置测试完成 ===\n";
}

#undef TIME_IT

#endif // TEST_PDO_CONFIG_HPP
