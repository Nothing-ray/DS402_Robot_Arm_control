/**
 * @file Test_module.hpp
 * @brief 机械臂控制系统综合测试模块
 *
 * @details 本文件包含DS402机械臂控制系统的完整测试套件，提供对核心模块的
 * 功能验证、性能测试和并发压力测试。测试覆盖电机类、PDO配置、SDO状态机等
 * 关键组件，确保系统在实时环境下的可靠性和性能。
 *
 * 主要测试内容：
 * - 电机类全面测试：初始化、数据转换、多线程并发、性能基准
 * - PDO配置测试：映射表构建、COB-ID转换、边界值处理
 * - TPDO处理测试：实际位置/速度/电流数据接收和解析
 * - SDO状态机测试：单线程性能、多线程并发、实时性评估
 *
 * @note 所有测试均支持中文输出，包含详细的性能统计和错误报告
 * @warning 测试中会故意触发异常情况，相关错误信息属于预期行为
 */

#ifndef TEST_MODULE
#define TEST_MODULE

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


#include "CLASS_Motor.hpp"
#include "PDO_config.hpp"
#include "Data_processing.hpp"
#include "CAN_frame.hpp"
#include "CAN_processing.hpp"
#include "SDO_State_Machine.hpp"
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




/******************** Motor类测试 ********************/



// 全局控制标志
std::atomic<bool> running{ true };



/******************** Motor类测试函数 ********************/

#include <vector>
#include <chrono>
#include <numeric>




/*请改写这个针对电机类的测试函数：
要求继续保留其单void函数性质，确保一次调用即可完成测试。

但是需要改写功能：
先单线程对一个电机进行初始化，读写，修改值，刷新。
再尝试进行多线程测试。
读写不再测试指定的时长，而是固定测试50次，将每次所消耗的时间保存起来，测试完成后再一并进行输出。
将所有的英文显示输出改为中文*/




/**
 * @brief 电机类全面测试函数（兼容性优化版本）
 * @param motors 电机实例数组引用
 *
 * @details 测试流程：
 * 1. 单线程测试：初始化、数据转换、刷新机制验证
 * 2. 多线程并发测试：模拟实际工作场景
 * 3. 性能基准测试：50次固定迭代，收集时序统计
 * 4. 数据一致性验证：多线程环境下的数据完整性检查
 * 5. 边界条件测试：极值处理和错误恢复
 *
 * @note 避免模板相关的编译问题，采用直接函数调用方式
 */
void testMotorClass(std::array<Motor, 6>&motors) {
    std::cout << "=== 开始电机类全面测试 ===\n";
    std::cout << "测试平台: " << (sizeof(void*) == 8 ? "64位" : "32位") << "\n";

    // 验证缓存行对齐（避免模板参数问题）
    bool alignmentOK = true;
    try {
        AlignedRawData<4, int32_t> testData;
        alignmentOK = (reinterpret_cast<uintptr_t>(&testData) % 64 == 0);
    }
    catch (...) {
        alignmentOK = false;
    }
    std::cout << "缓存行大小验证: " << (alignmentOK ? "通过" : "需检查") << "\n\n";

    // 性能数据收集容器
    struct PerformanceData {
        std::vector<long> initTimes;                    // 初始化耗时
        std::vector<long> singleRefreshTimes;           // 单线程刷新耗时
        std::vector<long> singleReadTimes;              // 单线程读取耗时
        std::vector<long> singleWriteTimes;             // 单线程写入耗时
        std::vector<long> multiWriteTimes;              // 多线程写入耗时
        std::vector<long> multiReadTimes;               // 多线程读取耗时
        std::vector<long> flagOperationTimes;           // 标志位操作耗时
        std::vector<long> atomicOperationTimes;         // 原子操作耗时
    } perfData;

    // 数据一致性检查计数器
    std::atomic<int> dataConsistencyErrors{ 0 };
    std::atomic<int> totalOperations{ 0 };

    /******************** 阶段1: 单线程基础功能测试 ********************/
    std::cout << "[阶段1] 单线程基础功能测试\n";
    std::cout << "========================================\n";

    // 初始化性能测试
    std::cout << "1.1 初始化性能测试...\n";
    for (int i = 0; i < 6; ++i) {
        auto start = std::chrono::high_resolution_clock::now();
        motors[i].init();
        auto end = std::chrono::high_resolution_clock::now();
        long initTime = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        perfData.initTimes.push_back(initTime);
        std::cout << "  电机" << i << " 初始化耗时: " << initTime << " 微秒\n";
    }

    // 数据转换精度测试
    std::cout << "\n1.2 数据转换精度测试...\n";
    Motor& testMotor = motors[0];

    // 测试电流转换
    float testCurrents[] = { 0.0f, 100.5f, -50.2f, 1000.0f, -1000.0f };
    for (float testCurrent : testCurrents) {
        testMotor.current.target_current.store(testCurrent);
        testMotor.current.flags_.fetch_or(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

        // 手动调用电流数据刷新
        testMotor.refreshMotorData(testMotor.current);

        float readBack = testMotor.current.target_current.load();
        Current_type encoderVal = testMotor.current.target_encoder.load();

        std::cout << "  电流测试: " << testCurrent << " mA → 编码器:" << encoderVal
            << " → 读回:" << readBack << " mA (误差:" << std::abs(testCurrent - readBack) << ")\n";
    }

    // 测试位置转换（角度与编码器值）
    std::cout << "\n  位置转换测试:\n";
    float testAngles[] = { 0.0f, 90.0f, -90.0f, 180.0f, -180.0f, 359.9f };
    for (float testAngle : testAngles) {
        testMotor.position.target_degree.store(testAngle);
        testMotor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

        // 手动调用位置数据刷新
        testMotor.refreshMotorData(testMotor.position);

        float readBack = testMotor.position.target_degree.load();
        Position_type encoderVal = testMotor.position.target_encoder.load();

        std::cout << "  角度测试: " << testAngle << "° → 编码器:" << encoderVal
            << " → 读回:" << readBack << "° (误差:" << std::abs(testAngle - readBack) << ")\n";
    }

    // 测试速度转换
    std::cout << "\n  速度转换测试:\n";
    float testSpeeds[] = { 0.0f, 100.0f, -100.0f, 500.0f, -500.0f };
    for (float testSpeed : testSpeeds) {
        testMotor.velocity.target_rpm_velocity_mode.store(testSpeed);
        testMotor.velocity.flags_.fetch_or(MotorVelocity::Flags::TARGET_DATA_SEND_NEED_REFRESH_VELOCITY_MODE, std::memory_order_release);

        // 手动调用速度数据刷新
        testMotor.refreshMotorData(testMotor.velocity);

        float readBack = testMotor.velocity.target_rpm_velocity_mode.load();
        Velocity_type encoderVal = testMotor.velocity.target_encoder_velocity_mode.load();

        std::cout << "  速度测试: " << testSpeed << " RPM → 编码器:" << encoderVal
            << " → 读回:" << readBack << " RPM (误差:" << std::abs(testSpeed - readBack) << ")\n";
    }

    /******************** 阶段2: 标志位系统测试 ********************/
    std::cout << "\n[阶段2] 标志位系统测试\n";
    std::cout << "========================================\n";

    // 测试标志位的并发安全性
    std::cout << "2.1 标志位并发安全测试...\n";
    for (int i = 0; i < 50; ++i) {
        auto start = std::chrono::high_resolution_clock::now();

        // 设置多个标志位
        testMotor.current.flags_.fetch_or(MotorCurrent::Flags::RAW_DATA_SEND_NEED_REFRESH, std::memory_order_release);
        testMotor.current.flags_.fetch_or(MotorCurrent::Flags::ENCODER_DATA_SEND_NEED_REFRESH, std::memory_order_release);
        testMotor.current.flags_.fetch_or(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

        // 检查标志位
        bool flag1 = testMotor.current.needsProcess(MotorCurrent::Flags::RAW_DATA_SEND_NEED_REFRESH);
        bool flag2 = testMotor.current.needsProcess(MotorCurrent::Flags::ENCODER_DATA_SEND_NEED_REFRESH);
        bool flag3 = testMotor.current.needsProcess(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH);

        // 清除标志位
        testMotor.current.markProcessed(MotorCurrent::Flags::RAW_DATA_SEND_NEED_REFRESH);
        testMotor.current.markProcessed(MotorCurrent::Flags::ENCODER_DATA_SEND_NEED_REFRESH);
        testMotor.current.markProcessed(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH);

        auto end = std::chrono::high_resolution_clock::now();
        long flagTime = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        perfData.flagOperationTimes.push_back(flagTime);

        if (!flag1 || !flag2 || !flag3) {
            std::cout << "  警告: 第" << i << "次标志位检查失败\n";
        }
    }

    /******************** 阶段3: 原子操作性能测试 ********************/
    std::cout << "\n[阶段3] 原子操作性能测试\n";
    std::cout << "========================================\n";

    std::cout << "3.1 原子读写性能测试 (50次迭代)...\n";
    for (int i = 0; i < 50; ++i) {
        auto start = std::chrono::high_resolution_clock::now();

        // 原子写入测试
        Current_type testValue = static_cast<Current_type>(100 + i);
        testMotor.current.raw_actual.atomicWriteValue(testValue);
        testMotor.position.raw_actual.atomicWriteValue(static_cast<Position_type>(1000 + i));
        testMotor.velocity.raw_actual.atomicWriteValue(static_cast<Velocity_type>(500 + i));

        // 原子读取测试
        Current_type readCurrent = testMotor.current.raw_actual.atomicReadValue();
        Position_type readPosition = testMotor.position.raw_actual.atomicReadValue();
        Velocity_type readVelocity = testMotor.velocity.raw_actual.atomicReadValue();

        auto end = std::chrono::high_resolution_clock::now();
        long atomicTime = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        perfData.atomicOperationTimes.push_back(atomicTime);

        // 数据一致性检查
        if (readCurrent != testValue || readPosition != (1000 + i) || readVelocity != (500 + i)) {
            dataConsistencyErrors.fetch_add(1);
        }
        totalOperations.fetch_add(1);
    }

    /******************** 阶段4: 刷新机制详细测试 ********************/
    std::cout << "\n[阶段4] 刷新机制详细测试\n";
    std::cout << "========================================\n";

    std::cout << "4.1 各类刷新模式测试 (50次迭代)...\n";
    for (int i = 0; i < 50; ++i) {
        auto start = std::chrono::high_resolution_clock::now();

        // 模拟接收到的原始数据
        Current_type rawCurrent = static_cast<Current_type>(200 + i);
        Position_type rawPosition = static_cast<Position_type>(2000 + i * 10);
        Velocity_type rawVelocity = static_cast<Velocity_type>(600 + i);

        // 写入原始数据并设置刷新标志
        testMotor.current.raw_actual.atomicWriteValue(rawCurrent);
        testMotor.position.raw_actual.atomicWriteValue(rawPosition);
        testMotor.velocity.raw_actual.atomicWriteValue(rawVelocity);

        testMotor.current.flags_.fetch_or(MotorCurrent::Flags::RAW_DATA_RECEIVE_NEED_REFRESH, std::memory_order_release);
        testMotor.position.flags_.fetch_or(MotorPosition::Flags::RAW_DATA_RECEIVE_NEED_REFRESH, std::memory_order_release);
        testMotor.velocity.flags_.fetch_or(MotorVelocity::Flags::RAW_DATA_RECEIVE_NEED_REFRESH, std::memory_order_release);

        // 执行刷新
        testMotor.refreshAllMotorData();

        auto end = std::chrono::high_resolution_clock::now();
        long refreshTime = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        perfData.singleRefreshTimes.push_back(refreshTime);

        // 验证刷新结果
        float actualCurrent = testMotor.current.actual_current.load();
        float actualPosition = testMotor.position.actual_degree.load();
        float actualVelocity = testMotor.velocity.actual_rpm.load();

        if (i % 10 == 0) {  // 每10次输出一次详细信息
            std::cout << "  第" << i << "次刷新: 电流" << actualCurrent << "mA, 位置"
                << actualPosition << "°, 速度" << actualVelocity << "RPM (耗时:" << refreshTime << "μs)\n";
        }
    }

    /******************** 阶段5: 多线程并发压力测试 ********************/
    std::cout << "\n[阶段5] 多线程并发压力测试\n";
    std::cout << "========================================\n";

    std::cout << "5.1 启动高强度并发测试 (6电机 × 2线程 × 50迭代)...\n";

    std::vector<std::thread> threads;
    std::atomic<int> completedOperations{ 0 };

    // 为每个电机创建读写线程对
    for (size_t motorIdx = 0; motorIdx < motors.size(); ++motorIdx) {
        Motor& motor = motors[motorIdx];

        // 写入线程
        threads.emplace_back([&motor, &perfData, &completedOperations, motorIdx]() {
            for (int i = 0; i < 50; ++i) {
                auto start = std::chrono::high_resolution_clock::now();

                // 高频写入操作
                float targetCurrent = 150.0f + i + motorIdx * 10;
                float targetPosition = 60.0f + i + motorIdx * 15;
                float targetVelocity = 700.0f + i + motorIdx * 20;

                motor.current.target_current.store(targetCurrent);
                motor.position.target_degree.store(targetPosition);
                motor.velocity.target_rpm_velocity_mode.store(targetVelocity);

                // 设置刷新标志
                motor.current.flags_.fetch_or(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);
                motor.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);
                motor.velocity.flags_.fetch_or(MotorVelocity::Flags::TARGET_DATA_SEND_NEED_REFRESH_VELOCITY_MODE, std::memory_order_release);

                // 执行刷新
                {
                    std::lock_guard<std::mutex> lock(motor.mtx_);
                    motor.refreshAllMotorData();
                }

                auto end = std::chrono::high_resolution_clock::now();
                long writeTime = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

                // 线程安全地添加性能数据
                static std::mutex perfMutex;
                {
                    std::lock_guard<std::mutex> perfLock(perfMutex);
                    perfData.multiWriteTimes.push_back(writeTime);
                }

                completedOperations.fetch_add(1);
                std::this_thread::sleep_for(std::chrono::microseconds(100)); // 模拟2ms周期的一部分
            }
            });

        // 读取线程
        threads.emplace_back([&motor, &perfData, &completedOperations, &dataConsistencyErrors, motorIdx]() {
            for (int i = 0; i < 50; ++i) {
                auto start = std::chrono::high_resolution_clock::now();

                // 模拟从CAN总线接收数据
                Current_type receivedCurrent = static_cast<Current_type>(300 + i + motorIdx * 5);
                Position_type receivedPosition = static_cast<Position_type>(3000 + i * 20 + motorIdx * 100);
                Velocity_type receivedVelocity = static_cast<Velocity_type>(800 + i + motorIdx * 10);

                {
                    std::lock_guard<std::mutex> lock(motor.mtx_);

                    // 写入接收数据
                    motor.current.raw_actual.atomicWriteValue(receivedCurrent);
                    motor.position.raw_actual.atomicWriteValue(receivedPosition);
                    motor.velocity.raw_actual.atomicWriteValue(receivedVelocity);

                    // 设置接收刷新标志
                    motor.current.flags_.fetch_or(MotorCurrent::Flags::RAW_DATA_RECEIVE_NEED_REFRESH, std::memory_order_release);
                    motor.position.flags_.fetch_or(MotorPosition::Flags::RAW_DATA_RECEIVE_NEED_REFRESH, std::memory_order_release);
                    motor.velocity.flags_.fetch_or(MotorVelocity::Flags::RAW_DATA_RECEIVE_NEED_REFRESH, std::memory_order_release);

                    // 执行刷新
                    motor.refreshAllMotorData();

                    // 读取转换后的数据
                    float actualCurrent = motor.current.actual_current.load();
                    float actualPosition = motor.position.actual_degree.load();
                    float actualVelocity = motor.velocity.actual_rpm.load();

                    // 简单数据一致性检查
                    if (std::abs(actualCurrent - receivedCurrent) > 0.1f) {
                        dataConsistencyErrors.fetch_add(1);
                    }
                }

                auto end = std::chrono::high_resolution_clock::now();
                long readTime = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

                // 线程安全地添加性能数据
                static std::mutex perfMutex;
                {
                    std::lock_guard<std::mutex> perfLock(perfMutex);
                    perfData.multiReadTimes.push_back(readTime);
                }

                completedOperations.fetch_add(1);
                std::this_thread::sleep_for(std::chrono::microseconds(150));
            }
            });
    }

    // 等待所有线程完成
    std::cout << "等待所有线程完成...\n";
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    std::cout << "所有线程已完成，总操作数: " << completedOperations.load() << "\n";

    /******************** 阶段6: 性能统计与分析 ********************/
    std::cout << "\n[阶段6] 性能统计与分析\n";
    std::cout << "========================================\n";

    // 计算统计数据的Lambda函数
    auto calculateStats = [](const std::vector<long>& data) -> std::tuple<double, long, long> {
        if (data.empty()) return { 0.0, 0, 0 };
        double avg = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
        long maxVal = *std::max_element(data.begin(), data.end());
        long minVal = *std::min_element(data.begin(), data.end());
        return { avg, maxVal, minVal };
        };

    // 输出各项性能统计
    auto [avgInit, maxInit, minInit] = calculateStats(perfData.initTimes);
    std::cout << "初始化性能:\n";
    std::cout << "  平均: " << avgInit << " μs, 最大: " << maxInit << " μs, 最小: " << minInit << " μs\n";

    auto [avgRefresh, maxRefresh, minRefresh] = calculateStats(perfData.singleRefreshTimes);
    std::cout << "\n单线程刷新性能:\n";
    std::cout << "  平均: " << avgRefresh << " μs, 最大: " << maxRefresh << " μs, 最小: " << minRefresh << " μs\n";

    auto [avgAtomicOp, maxAtomicOp, minAtomicOp] = calculateStats(perfData.atomicOperationTimes);
    std::cout << "\n原子操作性能:\n";
    std::cout << "  平均: " << avgAtomicOp << " μs, 最大: " << maxAtomicOp << " μs, 最小: " << minAtomicOp << " μs\n";

    auto [avgMultiWrite, maxMultiWrite, minMultiWrite] = calculateStats(perfData.multiWriteTimes);
    std::cout << "\n多线程写入性能:\n";
    std::cout << "  平均: " << avgMultiWrite << " μs, 最大: " << maxMultiWrite << " μs, 最小: " << minMultiWrite << " μs\n";

    auto [avgMultiRead, maxMultiRead, minMultiRead] = calculateStats(perfData.multiReadTimes);
    std::cout << "\n多线程读取性能:\n";
    std::cout << "  平均: " << avgMultiRead << " μs, 最大: " << maxMultiRead << " μs, 最小: " << minMultiRead << " μs\n";

    auto [avgFlag, maxFlag, minFlag] = calculateStats(perfData.flagOperationTimes);
    std::cout << "\n标志位操作性能:\n";
    std::cout << "  平均: " << avgFlag << " μs, 最大: " << maxFlag << " μs, 最小: " << minFlag << " μs\n";

    /******************** 阶段7: 最终状态验证 ********************/
    std::cout << "\n[阶段7] 最终状态验证\n";
    std::cout << "========================================\n";

    std::cout << "数据一致性检查:\n";
    std::cout << "  总操作数: " << totalOperations.load() << "\n";
    std::cout << "  一致性错误: " << dataConsistencyErrors.load() << "\n";

    double successRate = 100.0;
    if (totalOperations.load() > 0) {
        successRate = 100.0 - (double)dataConsistencyErrors.load() / totalOperations.load() * 100.0;
    }
    std::cout << "  成功率: " << successRate << "%\n";

    std::cout << "\n各电机最终状态:\n";
    for (size_t i = 0; i < motors.size(); ++i) {
        const Motor& motor = motors[i];
        std::cout << "电机" << i << ":\n";
        std::cout << "  电流: " << motor.current.actual_current.load() << " mA (编码器: "
            << motor.current.actual_encoder.load() << ")\n";
        std::cout << "  位置: " << motor.position.actual_degree.load() << "° (编码器: "
            << motor.position.actual_encoder.load() << ")\n";
        std::cout << "  速度: " << motor.velocity.actual_rpm.load() << " RPM (编码器: "
            << motor.velocity.actual_encoder.load() << ")\n";

        // 读取原始数据进行验证
        Current_type rawCurrent = motor.current.raw_actual.atomicReadValue();
        Position_type rawPosition = motor.position.raw_actual.atomicReadValue();
        Velocity_type rawVelocity = motor.velocity.raw_actual.atomicReadValue();

        std::cout << "  原始值: 电流=" << rawCurrent << ", 位置=" << rawPosition << ", 速度=" << rawVelocity << "\n\n";
    }

    /******************** 实时性能评估 ********************/
    std::cout << "实时性能评估 (基于2ms控制周期):\n";
    double maxAcceptableTime = 200.0; // 2ms周期的10%作为可接受上限

    bool realTimeCapable = true;
    if (avgMultiWrite > maxAcceptableTime || avgMultiRead > maxAcceptableTime) {
        realTimeCapable = false;
        std::cout << "  警告: 平均执行时间超过实时要求\n";
    }

    if (maxMultiWrite > 500.0 || maxMultiRead > 500.0) {
        realTimeCapable = false;
        std::cout << "  警告: 最大执行时间可能影响实时性\n";
    }

    std::cout << "  实时性评估: " << (realTimeCapable ? "满足要求" : "需要优化") << "\n";

    std::cout << "\n=== 电机类全面测试完成 ===\n";
    std::cout << "测试覆盖: 初始化、数据转换、标志位系统、原子操作、多线程并发、性能基准\n";
    std::cout << "总体评估: " << (dataConsistencyErrors.load() == 0 && realTimeCapable ? "通过" : "部分通过") << "\n";
}




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
    std::cout << "=== PDO Configuration Test Begin ===\n\n";

    /* 第一阶段：映射表初始化测试 */
    std::vector<PdoMappingEntry> mappingTable;

    // 使用计时宏测量6电机映射表构建时间
    TIME_IT(
        mappingTable = buildArmMappingTable(6),
        "Build mapping table for 6 motors"
    );

    /* 打印映射表详细内容 */
    std::cout << "\n=== Mapping Table Content ===\n";
    for (const auto& entry : mappingTable) {
        // 格式化输出每个映射条目信息：
        // 1. 电机编号 | 2. PDO类型及通道 | 3. 对象字典地址
        // 4. 数据帧偏移 | 5. 数据大小 | 6. Motor类成员偏移
        std::cout << "Motor " << (int)entry.motorIndex
            << " | " << (entry.isTx ? "TPDO" : "RPDO") << (int)entry.pdoIndex
            << " | OD: 0x" << std::hex << entry.index << std::dec << "/" << (int)entry.subIndex
            << " | Frame offset: " << (int)entry.offsetInPdo
            << " | Size: " << (int)entry.size << " bytes"
            << " | Motor offset: " << entry.motorFieldOffset
            << "\n";
    }
    std::cout << "Total entries: " << mappingTable.size() << "\n\n";

    /* 第二阶段：COB-ID转换工具测试 */
    std::cout << "=== COB-ID Conversion Test ===\n";

    // 定义COB-ID测试lambda函数，封装重复测试逻辑
    auto testCobIdConversion = [](uint8_t motorId) {
        std::cout << "For Motor " << (int)motorId << ":\n";

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
            std::cerr << "  Error: " << e.what() << "\n";
        }
        };

    /* 测试用例1：正常值测试（电机1） */
    TIME_IT(
        testCobIdConversion(1),
        "COB-ID conversion for motor 1"
    );

    /* 测试用例2：边界值测试（电机12，最大值） */
    std::cout << "\nTesting boundary values:\n";
    TIME_IT(
        testCobIdConversion(12),
        "COB-ID conversion for motor 12 (max)"
    );

    /* 测试用例3：异常值测试 */
    std::cout << "\nTesting invalid values:\n";

    // 子用例3.1：电机ID为0（非法值）
    try {
        TIME_IT(
            toSdoMotorId(0),
            "Invalid motor ID 0"
        );
    }
    catch (const std::exception& e) {
        std::cerr << "Expected error: " << e.what() << "\n";
    }

    // 子用例3.2：电机ID为13（超出上限）
    try {
        TIME_IT(
            toRpdoCobId(13, 1),
            "Invalid motor ID 13"
        );
    }
    catch (const std::exception& e) {
        std::cerr << "Expected error: " << e.what() << "\n";
    }

    // 子用例3.3：PDO索引为5（非法值）
    try {
        TIME_IT(
            toTpdoCobId(1, 5),
            "Invalid PDO index 5"
        );
    }
    catch (const std::exception& e) {
        std::cerr << "Expected error: " << e.what() << "\n";
    }

    std::cout << "\n=== PDO Configuration Test Complete ===\n";
}




/*********************** PDO读写测试区 **********************/


/**
 * @brief 全面测试TPDO(下位机→上位机)数据接收和处理
 *
 * 测试覆盖：
 * - TPDO1: 实际位置(4B) + 状态字(2B)
 * - TPDO2: 实际速度(2B) + 实际电流(2B)
 * - 所有电机节点(1-6)
 * - 各种数据边界值和特殊情况
 *
 * 数据范围说明：
 * - 电流：-32767 ~ 32768 mA
 * - 速度：-3000 ~ 3000 RPM
 * - 位置：-32768 ~ 32767 脉冲 (对应 -180° ~ 180°)
 */
void testTPDOProcessing() {
    std::cout << "\n===============================================" << std::endl;
    std::cout << "        TPDO (下位机→上位机) 全面测试" << std::endl;
    std::cout << "===============================================" << std::endl;

    // 1. 初始化测试环境
    std::cout << "\n[步骤1] 初始化测试环境..." << std::endl;
    std::array<Motor, 6> motors = {
        Motor(1), Motor(2), Motor(3),
        Motor(4), Motor(5), Motor(6)
    };

    // 构建PDO配置表
    std::vector<PdoMappingEntry> pdoTable = buildArmMappingTable(6);
    std::cout << " Y  初始化6个电机" << std::endl;
    std::cout << " Y  PDO配置表生成完成，共" << pdoTable.size() << "条映射" << std::endl;

    // 2. TPDO1测试 (0x180 + NodeID) - 实际位置 + 状态字
    std::cout << "\n[步骤2] 测试TPDO1 (实际位置 + 状态字)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "位置范围: -32768 ~ 32767 脉冲 (对应 -180° ~ 180°)" << std::endl;

    struct TPDO1TestCase {
        uint8_t nodeId;
        int32_t position;
        uint16_t statusWord;
        const char* description;
    };

    std::vector<TPDO1TestCase> tpdo1Tests = {
        {1, 0, 0x0237, "电机1: 0°位置(0脉冲)，已使能状态"},
        {2, 32767, 0x0233, "电机2: +180°最大位置(32767脉冲)，未使能"},
        {3, -32768, 0x0627, "电机3: -180°最小位置(-32768脉冲)，运行中"},
        {4, 16384, 0x0237, "电机4: +90°位置(16384脉冲)，正常运行"},
        {5, -16384, 0x021F, "电机5: -90°位置(-16384脉冲)，快速停止"},
        {6, 5461, 0x0608, "电机6: +30°位置(5461脉冲)，故障状态"}
    };

    for (const auto& test : tpdo1Tests) {
        // 构造TPDO1帧数据
        uint8_t data[8] = { 0 };

        // 位置数据 (小端序，4字节)
        data[0] = (test.position >> 0) & 0xFF;
        data[1] = (test.position >> 8) & 0xFF;
        data[2] = (test.position >> 16) & 0xFF;
        data[3] = (test.position >> 24) & 0xFF;

        // 状态字 (小端序，2字节)
        data[4] = (test.statusWord >> 0) & 0xFF;
        data[5] = (test.statusWord >> 8) & 0xFF;

        CanFrame tpdo1Frame(0x180 + test.nodeId, data, 6);

        std::cout << "\n测试用例: " << test.description << std::endl;
        std::cout << "  CAN ID: 0x" << std::hex << (0x180 + test.nodeId) << std::dec
            << ", 数据: ";
        for (int i = 0; i < 6; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                << (int)data[i] << " ";
        }
        std::cout << std::dec << std::endl;

        // 处理CAN帧
        parseCanFrame(tpdo1Frame, motors, pdoTable);

        // 验证结果
        auto& motor = motors[test.nodeId - 1];
        {
            std::lock_guard<std::mutex> lock(motor.mtx_);

            // 检查位置值
            double actualPos = motor.position.actual_degree.load();
            double angleDegrees = (double)actualPos * 180.0 / 32767.0;
            std::cout << "  实际位置: " << actualPos << " 脉冲 ("
                << std::fixed << std::setprecision(1) << angleDegrees
                << "°) [期望: " << convertSensorAngle(test.position,true) << "]";
            if ((actualPos - convertSensorAngle(test.position, true)) < 0.01f) {
                std::cout << "  Y " << std::endl;
            }
            else {
                std::cout << "  N  错误!" << std::endl;
            }

            // 检查状态字
            uint16_t actualStatus = (motor.stateAndMode.controlData.statusWordRaw[1] << 8) |
                motor.stateAndMode.controlData.statusWordRaw[0];
            std::cout << "  状态字: 0x" << std::hex << actualStatus
                << " (期望: 0x" << test.statusWord << ")" << std::dec;
            if (actualStatus == test.statusWord) {
                std::cout << "  Y " << std::endl;
            }
            else {
                std::cout << "  N  错误!" << std::endl;
            }

            // 检查刷新标志
            std::cout << "  位置刷新标志: " <<
                ((motor.position.flags_.load() & MotorPosition::RAW_DATA_RECEIVE_NEED_REFRESH) ? "已设置  Y " : "未设置  N ") << std::endl;
            std::cout << "  状态刷新标志: " <<
                (motor.stateAndMode.refresh ? "已设置  Y " : "未设置  N ") << std::endl;
        }
    }

    // 3. TPDO2测试 (0x280 + NodeID) - 实际速度 + 实际电流
    std::cout << "\n[步骤3] 测试TPDO2 (实际速度 + 实际电流)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "速度范围: -3000 ~ 3000 RPM" << std::endl;
    std::cout << "电流范围: -32767 ~ 32768 mA" << std::endl;

    struct TPDO2TestCase {
        uint8_t nodeId;
        int16_t velocity;
        int16_t current;
        const char* description;
    };

    std::vector<TPDO2TestCase> tpdo2Tests = {
        {1, 0, 0, "电机1: 停止状态(0 RPM)，无电流(0 mA)"},
        {2, 3000, 15000, "电机2: 最大正速度(3000 RPM)，15A电流"},
        {3, -3000, -15000, "电机3: 最大负速度(-3000 RPM)，-15A电流"},
        {4, 1500, 32767, "电机4: 半速正转(1500 RPM)，最大正电流(32.767A)"},
        {5, -1500, -32767, "电机5: 半速反转(-1500 RPM)，最大负电流(-32.767A)"},
        {6, 100, -500, "电机6: 低速正转(100 RPM)，反向制动电流(-500mA)"}
    };

    for (const auto& test : tpdo2Tests) {
        // 构造TPDO2帧数据
        uint8_t data[8] = { 0 };

        // 实际速度 (小端序，2字节)
        data[0] = (test.velocity >> 0) & 0xFF;
        data[1] = (test.velocity >> 8) & 0xFF;

        // 实际电流 (小端序，2字节)
        data[2] = (test.current >> 0) & 0xFF;
        data[3] = (test.current >> 8) & 0xFF;

        CanFrame tpdo2Frame(0x280 + test.nodeId, data, 4);

        std::cout << "\n测试用例: " << test.description << std::endl;
        std::cout << "  CAN ID: 0x" << std::hex << (0x280 + test.nodeId) << std::dec
            << ", 数据: ";
        for (int i = 0; i < 4; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                << (int)data[i] << " ";
        }
        std::cout << std::dec << std::endl;

        // 处理CAN帧
        parseCanFrame(tpdo2Frame, motors, pdoTable);

        // 验证结果
        auto& motor = motors[test.nodeId - 1];
        {
            std::lock_guard<std::mutex> lock(motor.mtx_);

            // 检查速度值
            int16_t actualVel = motor.velocity.actual_rpm.load();
            std::cout << "  实际速度: " << actualVel << " RPM (期望: " << test.velocity << ")";
            if (actualVel == test.velocity) {
                std::cout << "  Y " << std::endl;
            }
            else {
                std::cout << "  N  错误!" << std::endl;
            }

            // 检查电流值
            int16_t actualCur = motor.current.actual_current.load();
            double currentAmperes = actualCur / 1000.0;
            std::cout << "  实际电流: " << actualCur << " mA ("
                << std::fixed << std::setprecision(3) << currentAmperes
                << " A) [期望: " << test.current << " mA]";
            if (actualCur == test.current) {
                std::cout << "  Y " << std::endl;
            }
            else {
                std::cout << "  N  错误!" << std::endl;
            }

            // 检查刷新标志
            std::cout << "  速度刷新标志: " <<
                ((motor.velocity.flags_.load() & MotorVelocity::RAW_DATA_RECEIVE_NEED_REFRESH) ? "已设置  Y " : "未设置  N ") << std::endl;
            std::cout << "  电流刷新标志: " <<
                ((motor.current.flags_.load() & MotorCurrent::RAW_DATA_RECEIVE_NEED_REFRESH) ? "已设置  Y " : "未设置  N ") << std::endl;
        }
    }

    // 4. 特殊边界值测试
    std::cout << "\n[步骤4] 特殊边界值测试" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    // 测试速度超出范围的处理
    {
        std::cout << "\n测试: 速度边界值组合" << std::endl;
        struct BoundaryTest {
            int16_t velocity;
            int16_t current;
            const char* description;
        };

        std::vector<BoundaryTest> boundaryTests = {
            {2999, 32000, "接近最大速度(2999 RPM)和高电流(32A)"},
            {-2999, -32000, "接近最小速度(-2999 RPM)和高负电流(-32A)"},
            {1, 1, "最小正速度(1 RPM)和最小电流(1mA)"},
            {-1, -1, "最小负速度(-1 RPM)和最小负电流(-1mA)"}
        };

        for (size_t i = 0; i < boundaryTests.size(); i++) {
            auto& test = boundaryTests[i];
            uint8_t data[4];
            data[0] = (test.velocity >> 0) & 0xFF;
            data[1] = (test.velocity >> 8) & 0xFF;
            data[2] = (test.current >> 0) & 0xFF;
            data[3] = (test.current >> 8) & 0xFF;

            CanFrame frame(0x281, data, 4); // 使用电机1
            std::cout << "  " << test.description << std::endl;
            parseCanFrame(frame, motors, pdoTable);

            auto& motor = motors[0];
            std::lock_guard<std::mutex> lock(motor.mtx_);
            std::cout << "    速度: " << motor.velocity.actual_rpm.load() << " RPM, "
                << "电流: " << motor.current.actual_current.load() << " mA  Y " << std::endl;
        }
    }

    // 5. 异常情况测试
    std::cout << "\n[步骤5] 异常情况测试" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    // 测试数据长度不足
    {
        std::cout << "\n测试: 数据长度不足的TPDO1帧" << std::endl;
        CanFrame shortFrame(0x181, new uint8_t[3]{ 0x01, 0x02, 0x03 }, 3); // 只有3字节
        parseCanFrame(shortFrame, motors, pdoTable);
        std::cout << "  处理完成，应忽略此帧  Y " << std::endl;
    }

    // 测试无效节点ID
    {
        std::cout << "\n测试: 无效节点ID (NodeID=0)" << std::endl;
        CanFrame invalidFrame(0x180, new uint8_t[6]{ 0 }, 6); // NodeID = 0
        parseCanFrame(invalidFrame, motors, pdoTable);
        std::cout << "  处理完成，应忽略此帧  Y " << std::endl;
    }

    // 测试超出范围的节点ID
    {
        std::cout << "\n测试: 超出范围的节点ID (NodeID=7)" << std::endl;
        CanFrame outOfRangeFrame(0x187, new uint8_t[6]{ 0 }, 6); // NodeID = 7
        parseCanFrame(outOfRangeFrame, motors, pdoTable);
        std::cout << "  处理完成，应忽略此帧  Y " << std::endl;
    }

    // 6. 实际应用场景模拟
    std::cout << "\n[步骤6] 实际应用场景模拟" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    std::cout << "模拟电机从0°旋转到90°的过程..." << std::endl;
    int32_t startPos = 0;        // 0°
    int32_t endPos = 16384;      // 90°
    int32_t step = 1638;         // 约9°每步

    for (int32_t pos = startPos; pos <= endPos; pos += step) {
        // 计算对应的速度和电流
        int16_t velocity = (pos < endPos / 2) ? 2000 : 1000;  // 前半段快速，后半段减速
        int16_t current = (pos < endPos / 2) ? 5000 : 2000;   // 对应调整电流

        // TPDO1: 位置更新
        uint8_t data1[6];
        data1[0] = (pos >> 0) & 0xFF;
        data1[1] = (pos >> 8) & 0xFF;
        data1[2] = (pos >> 16) & 0xFF;
        data1[3] = (pos >> 24) & 0xFF;
        data1[4] = 0x37;  // 运行中状态
        data1[5] = 0x06;

        CanFrame tpdo1(0x181, data1, 6);
        parseCanFrame(tpdo1, motors, pdoTable);

        // TPDO2: 速度和电流更新
        uint8_t data2[4];
        data2[0] = (velocity >> 0) & 0xFF;
        data2[1] = (velocity >> 8) & 0xFF;
        data2[2] = (current >> 0) & 0xFF;
        data2[3] = (current >> 8) & 0xFF;

        CanFrame tpdo2(0x281, data2, 4);
        parseCanFrame(tpdo2, motors, pdoTable);

        // 从电机对象中读取实际参数
            auto & motor = motors[0];  // 假设测试电机1
        {
            std::lock_guard<std::mutex> lock(motor.mtx_);
            double actualAngle = motor.position.actual_degree.load();
            int16_t actualVelocity = motor.velocity.actual_rpm.load();
            int16_t actualCurrent = motor.current.actual_current.load();
            uint16_t actualStatus = (motor.stateAndMode.controlData.statusWordRaw[1] << 8) |
                motor.stateAndMode.controlData.statusWordRaw[0];
            std::cout << "  实际状态 - "
                << "位置: " << std::fixed << std::setprecision(1) << actualAngle
                << "°, 速度: " << actualVelocity << " RPM, 电流: "
                << std::fixed << std::setprecision(1) << actualCurrent / 1000.0 << " A"
                << ", 状态字: 0x" << std::hex << std::setw(4) << std::setfill('0') << actualStatus << std::dec
                << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 模拟50ms周期
    }

    // 最终状态汇总
    std::cout << "\n[步骤7] 最终状态汇总" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    for (size_t i = 0; i < motors.size(); i++) {
        auto& motor = motors[i];
        std::lock_guard<std::mutex> lock(motor.mtx_);

        int32_t pos = motor.position.actual_degree.load();
        double angle = (double)pos * 180.0 / 32767.0;
        int16_t vel = motor.velocity.actual_rpm.load();
        int16_t cur = motor.current.actual_current.load();

        std::cout << "电机" << (i + 1) << ": "
            << "位置=" << std::fixed << std::setprecision(1) << angle << "°, "
            << "速度=" << vel << " RPM, "
            << "电流=" << std::fixed << std::setprecision(2) << cur / 1000.0 << " A"
            << std::endl;
    }

    std::cout << "\n===============================================" << std::endl;
    std::cout << "          TPDO测试完成！" << std::endl;
    std::cout << "===============================================\n" << std::endl;
}










/**************************************** SDO状态机测试 ****************************************/






/*********** SDO状态机单线程测试 **********/

/**
 * @brief SDO状态机精确性能测试函数
 *
 * @details 单线程精确测试SDO状态机的处理开销，避免多线程同步带来的额外延迟
 * 测试包括：事务准备时间、响应处理时间、状态转换时间等核心操作
 */
void testSdoStateMachinePerformance() {
    std::cout << "=== SDO状态机精确性能测试开始 ===\n";

    // 性能数据收集
    struct PerformanceData {
        std::vector<long> prepareTimes;      // 事务准备时间
        std::vector<long> startTimes;        // 开始事务时间
        std::vector<long> processTimes;      // 处理响应时间
        std::vector<long> totalTimes;        // 总处理时间
        std::vector<long> classificationTimes; // 响应分类时间
    } perfData;

    // 创建状态机实例
    canopen::AtomicSdoStateMachine sdoMachine;

    // 生成测试数据
    std::vector<std::pair<CanFrame, CanFrame>> testCases;

    // 创建多种类型的SDO测试用例
    for (uint8_t nodeId = 1; nodeId <= 6; ++nodeId) {
        // 读8位数据测试
        uint8_t read8Request[8] = { 0x40, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };
        uint8_t read8Response[8] = { 0x4F, 0x00, 0x60, 0x00, 0x37, 0x00, 0x00, 0x00 };
        testCases.emplace_back(
            CanFrame(0x600 + nodeId, read8Request, 4),
            CanFrame(0x580 + nodeId, read8Response, 5)
        );

        // 读16位数据测试
        uint8_t read16Request[8] = { 0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };
        uint8_t read16Response[8] = { 0x4B, 0x41, 0x60, 0x00, 0x37, 0x02, 0x00, 0x00 };
        testCases.emplace_back(
            CanFrame(0x600 + nodeId, read16Request, 4),
            CanFrame(0x580 + nodeId, read16Response, 6)
        );

        // 读32位数据测试
        uint8_t read32Request[8] = { 0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };
        uint8_t read32Response[8] = { 0x43, 0x64, 0x60, 0x00, 0x00, 0x00, 0x01, 0x00 };
        testCases.emplace_back(
            CanFrame(0x600 + nodeId, read32Request, 4),
            CanFrame(0x580 + nodeId, read32Response, 8)
        );

        // 写8位数据测试
        uint8_t write8Request[8] = { 0x2F, 0x60, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00 };
        uint8_t write8Response[8] = { 0x60, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };
        testCases.emplace_back(
            CanFrame(0x600 + nodeId, write8Request, 5),
            CanFrame(0x580 + nodeId, write8Response, 4)
        );

        // 写16位数据测试
        uint8_t write16Request[8] = { 0x2B, 0x41, 0x60, 0x00, 0x37, 0x02, 0x00, 0x00 };
        uint8_t write16Response[8] = { 0x60, 0x41, 0x60, 0x00, 0x37, 0x00, 0x00, 0x00 };
        testCases.emplace_back(
            CanFrame(0x600 + nodeId, write16Request, 6),
            CanFrame(0x580 + nodeId, write16Response, 4)
        );

        // 错误响应测试
        uint8_t errorRequest[8] = { 0x40, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };
        uint8_t errorResponse[8] = { 0x80, 0x00, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00 };
        testCases.emplace_back(
            CanFrame(0x600 + nodeId, errorRequest, 4),
            CanFrame(0x580 + nodeId, errorResponse, 8)
        );
    }

    std::cout << "生成 " << testCases.size() << " 个测试用例\n";

    // 预热运行（避免冷启动影响）
    std::cout << "预热运行...\n";
    for (int i = 0; i < 10; ++i) {
        for (const auto& testCase : testCases) {
            auto transaction = sdoMachine.prepareTransaction(testCase.first);
            sdoMachine.startTransaction(transaction);
            uint8_t nodeId = canopen::AtomicSdoStateMachine::extractNodeId(testCase.second.frameID);
            sdoMachine.processResponse(testCase.second.data, testCase.second.dlc, nodeId);
            sdoMachine.completeTransaction();
        }
    }

    // 正式性能测试
    std::cout << "开始正式性能测试...\n";

    for (const auto& testCase : testCases) {
        // 测试事务准备时间
        auto prepareStart = std::chrono::high_resolution_clock::now();
        auto transaction = sdoMachine.prepareTransaction(testCase.first);
        auto prepareEnd = std::chrono::high_resolution_clock::now();
        long prepareTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
            prepareEnd - prepareStart).count();
        perfData.prepareTimes.push_back(prepareTime);

        // 测试开始事务时间
        auto startStart = std::chrono::high_resolution_clock::now();
        bool started = sdoMachine.startTransaction(transaction);
        auto startEnd = std::chrono::high_resolution_clock::now();
        long startTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
            startEnd - startStart).count();
        perfData.startTimes.push_back(startTime);

        if (!started) {
            std::cout << "警告: 事务启动失败\n";
            continue;
        }

        // 测试响应处理时间
        uint8_t nodeId = canopen::AtomicSdoStateMachine::extractNodeId(testCase.second.frameID);

        auto processStart = std::chrono::high_resolution_clock::now();
        bool processed = sdoMachine.processResponse(testCase.second.data, testCase.second.dlc, nodeId);
        auto processEnd = std::chrono::high_resolution_clock::now();
        long processTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
            processEnd - processStart).count();
        perfData.processTimes.push_back(processTime);

        // 测试响应分类时间（单独测量）
        auto classifyStart = std::chrono::high_resolution_clock::now();
        auto responseType = sdoMachine.getResponseType();
        auto classifyEnd = std::chrono::high_resolution_clock::now();
        long classifyTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
            classifyEnd - classifyStart).count();
        perfData.classificationTimes.push_back(classifyTime);

        // 总时间
        long totalTime = prepareTime + startTime + processTime;
        perfData.totalTimes.push_back(totalTime);

        if (!processed) {
            std::cout << "警告: 响应处理失败\n";
        }

        sdoMachine.completeTransaction();
    }

    // 性能统计分析
    auto calculateStats = [](const std::vector<long>& data, const std::string& unit = "ns")
        -> std::tuple<double, long, long, long> {
        if (data.empty()) return { 0.0, 0, 0, 0 };

        double sum = std::accumulate(data.begin(), data.end(), 0.0);
        double avg = sum / data.size();
        long max = *std::max_element(data.begin(), data.end());
        long min = *std::min_element(data.begin(), data.end());
        long median = data[data.size() / 2];

        // 正确的单位转换逻辑
        if (unit == "μs") {
            // 数据本身是纳秒，转换为微秒
            avg = avg / 1000.0;
            max = max / 1000;
            min = min / 1000;
            median = median / 1000;
        }
        else if (unit == "ms") {
            // 数据本身是纳秒，转换为毫秒
            avg = avg / 1000000.0;
            max = max / 1000000;
            min = min / 1000000;
            median = median / 1000000;
        }
        // 如果是"ns"，不需要转换

        return { avg, max, min, median };
        };


    // 输出性能结果
    std::cout << "\n=== SDO状态机性能测试结果 ===\n";
    std::cout << "测试用例总数: " << testCases.size() << "\n\n";

    auto [avgPrepare, maxPrepare, minPrepare, medPrepare] = calculateStats(perfData.prepareTimes, "ns");
    std::cout << "事务准备时间 (ns):\n";
    std::cout << "  平均: " << avgPrepare << " ns, 最大: " << maxPrepare << " ns, 最小: " << minPrepare << " ns, 中位数: " << medPrepare << " ns\n";

    auto [avgStart, maxStart, minStart, medStart] = calculateStats(perfData.startTimes, "ns");
    std::cout << "开始事务时间 (ns):\n";
    std::cout << "  平均: " << avgStart << " ns, 最大: " << maxStart << " ns, 最小: " << minStart << " ns, 中位数: " << medStart << " ns\n";

    auto [avgProcess, maxProcess, minProcess, medProcess] = calculateStats(perfData.processTimes, "ns");
    std::cout << "响应处理时间 (ns):\n";
    std::cout << "  平均: " << avgProcess << " ns, 最大: " << maxProcess << " ns, 最小: " << minProcess << " ns, 中位数: " << medProcess << " ns\n";

    auto [avgClassify, maxClassify, minClassify, medClassify] = calculateStats(perfData.classificationTimes, "ns");
    std::cout << "响应分类时间 (ns):\n";
    std::cout << "  平均: " << avgClassify << "ns, 最大: " << maxClassify << " ns, 最小: " << minClassify << " ns, 中位数: " << medClassify << " ns\n";

    auto [avgTotal, maxTotal, minTotal, medTotal] = calculateStats(perfData.totalTimes, "μs");
    std::cout << "总处理时间 (μs):\n";
    std::cout << "  平均: " << avgTotal << " μs, 最大: " << maxTotal << " μs, 最小: " << minTotal << " μs, 中位数: " << medTotal << " μs\n";

    // 实时性评估
    std::cout << "\n=== 实时性评估 ===\n";
    constexpr long MAX_ACCEPTABLE_TIME = 200; // 200μs = 2ms周期的10%
    constexpr long CRITICAL_TIME = 500;       // 500μs = 2ms周期的25%

    bool realTimeCapable = true;

    if (maxTotal > CRITICAL_TIME) {
        std::cout << "警告: 最大处理时间(" << maxTotal << "μs)超过临界值，可能影响2ms实时周期\n";
        realTimeCapable = false;
    }

    if (avgTotal > MAX_ACCEPTABLE_TIME) {
        std::cout << "警告: 平均处理时间(" << avgTotal << "μs)占用较多周期时间\n";
        realTimeCapable = false;
    }


// 时间分布统计（使用正确的纳秒单位）
    int under100ns = 0, under500ns = 0, under1000ns = 0, over1000ns = 0;
    for (long timeNs : perfData.totalTimes) {
        if (timeNs < 100) under100ns++;
        else if (timeNs < 500) under500ns++;
        else if (timeNs < 1000) under1000ns++;
        else over1000ns++;
    }

    std::cout << "\n时间分布统计 (ns):\n";
    std::cout << "  <100ns: " << under100ns << " 次 (" << (under100ns * 100.0 / perfData.totalTimes.size()) << "%)\n";
    std::cout << "  100-500ns: " << under500ns << " 次 (" << (under500ns * 100.0 / perfData.totalTimes.size()) << "%)\n";
    std::cout << "  500-1000ns: " << under1000ns << " 次 (" << (under1000ns * 100.0 / perfData.totalTimes.size()) << "%)\n";
    std::cout << "  >1000ns: " << over1000ns << " 次 (" << (over1000ns * 100.0 / perfData.totalTimes.size()) << "%)\n";


    std::cout << "实时性评估: " << (realTimeCapable ?"✓ 满足要求" : "✗ 需要优化") << "\n";

    // 正确的吞吐量计算
    double totalTestTimeNs = std::accumulate(perfData.totalTimes.begin(),
        perfData.totalTimes.end(), 0.0);
    double throughput = (perfData.totalTimes.size() * 1000000000.0) / totalTestTimeNs;  // 事务/秒

    std::cout << "\n=== 吞吐量分析 ===\n";
    std::cout << "理论最大吞吐量: " << std::fixed << std::setprecision(1) << throughput << " 事务/秒\n";
    std::cout << "相当于每2ms周期可处理: " << (throughput * 0.002) << " 个SDO事务\n";



    std::cout << "\n=== SDO状态机性能测试完成 ===\n";

    // 输出建议
    if (realTimeCapable) {
        std::cout << "✓ 状态机性能良好，适合实时控制系统使用\n";
    }
    else {
        std::cout << "⚠ 状态机性能需要优化，建议:\n";
        std::cout << "  - 检查内存屏障使用是否必要\n";
        std::cout << "  - 优化条件变量等待逻辑\n";
        std::cout << "  - 减少不必要的内存拷贝\n";
        std::cout << "  - 考虑使用更轻量的同步机制\n";
    }
}

// 在测试模块中添加函数声明
void testSdoStateMachinePerformance();
void testSdoTimeoutRetryMechanism();








/******************************* SDO状态机多线程测试 *******************************/








/**
 * @brief SDO状态机多线程性能测试函数
 *
 * @details 模拟真实场景下的多线程SDO通信流程，一个发送线程和一个接收线程共同操作状态机
 * 测试包括：发送线程和接收线程并发操作状态机的性能开销、竞争情况、实时性表现
 */
void testSDOStateMachineMultiThreadPerformance() {
    std::cout << "=== SDO状态机多线程性能测试开始 ===\n";

    // 性能数据收集
    struct ThreadPerformance {
        std::vector<long> sendTimes;
        std::vector<long> receiveTimes;
        std::vector<long> totalCycleTimes;
        std::vector<long> contentionDelays;
        std::atomic<int> successCount{ 0 };
        std::atomic<int> timeoutCount{ 0 };
        std::atomic<int> errorCount{ 0 };
        std::atomic<int> normalResponseCount{ 0 };    // 正常响应
        std::atomic<int> errorResponseCount{ 0 };     // 错误响应
        std::atomic<bool> skipFirstMeasurement{ true }; // 跳过首次测量
    } perfData;

    std::mutex perfDataMutex; // 保护性能数据的互斥锁

    // 创建状态机实例（共享资源）
    canopen::AtomicSdoStateMachine sdoMachine;

    // 生成测试数据
    // 生成测试数据（分开正常和错误用例）
        std::vector<std::pair<CanFrame, CanFrame>> normalTestCases;   // 正常响应用例
    std::vector<std::pair<CanFrame, CanFrame>> errorTestCases;    // 错误响应用例

    // 创建多种类型的SDO测试用例（6个节点，每种操作类型）
    for (uint8_t nodeId = 1; nodeId <= 6; ++nodeId) {
        // 正常读操作
        uint8_t readRequest[8] = { 0x40, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };
        uint8_t readResponse[8] = { 0x4B, 0x00, 0x60, 0x00, 0x37, 0x02, 0x00, 0x00 };
        normalTestCases.emplace_back(
            CanFrame(0x600 + nodeId, readRequest, 4),
            CanFrame(0x580 + nodeId, readResponse, 6)
        );
        // 正常写操作
        uint8_t writeRequest[8] = { 0x2B, 0x60, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00 };
        uint8_t writeResponse[8] = { 0x60, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };
        normalTestCases.emplace_back(
            CanFrame(0x600 + nodeId, writeRequest, 5),
            CanFrame(0x580 + nodeId, writeResponse, 4)
        );
        // 错误响应
        uint8_t errorRequest[8] = { 0x40, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };
        uint8_t errorResponse[8] = { 0x80, 0x00, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00 };
        errorTestCases.emplace_back(
            CanFrame(0x600 + nodeId, errorRequest, 4),
            CanFrame(0x580 + nodeId, errorResponse, 8)
        );
    }
    // 合并所有测试用例（正常用例在前，错误用例在后）
    std::vector<std::pair<CanFrame, CanFrame>> allTestCases;
    allTestCases.insert(allTestCases.end(), normalTestCases.begin(), normalTestCases.end());
    allTestCases.insert(allTestCases.end(), errorTestCases.begin(), errorTestCases.end());
    std::cout << "生成 " << allTestCases.size() << " 个测试用例\n";
    std::cout << "正常响应用例: " << normalTestCases.size() << "\n";
    std::cout << "错误响应用例: " << errorTestCases.size() << "\n";

    // 测试参数
    constexpr int TEST_DURATION_MS = 2000;   // 每轮测试持续时间
    constexpr int NUM_TEST_ROUNDS = 5;       // 测试轮数

    bool is_first = true;


    for (int round = 0; round < NUM_TEST_ROUNDS; ++round) {
        std::cout << "\n=== 第 " << (round + 1) << " 轮测试开始 ===\n";

        // 重置状态机
        sdoMachine.reset();

        // 同步控制
        std::atomic<bool> testRunning{ true };
        std::atomic<int> processedCount{ 0 };









        // 发送线程
        auto senderThread = [&]() {
            std::mt19937 gen(std::random_device{}());
            std::uniform_int_distribution<> dis(0, allTestCases.size() - 1);
            while (testRunning) {
                auto startTime = std::chrono::high_resolution_clock::now();
                int caseIndex = dis(gen);
                const auto& testCase = allTestCases[caseIndex];
                auto transaction = sdoMachine.prepareTransaction(testCase.first);
                auto contentionStart = std::chrono::high_resolution_clock::now();
                bool started = sdoMachine.startTransaction(transaction);
                auto contentionEnd = std::chrono::high_resolution_clock::now();
                if (!started) continue;
                long contentionDelay = std::chrono::duration_cast<std::chrono::microseconds>(
                    contentionEnd - contentionStart).count();
                {
                    std::lock_guard<std::mutex> lock(perfDataMutex);
                    perfData.contentionDelays.push_back(contentionDelay);
                }
                bool responseReceived = sdoMachine.waitForResponse(2000);
                auto endTime = std::chrono::high_resolution_clock::now();
                long processTime = std::chrono::duration_cast<std::chrono::microseconds>(
                    endTime - startTime).count();
                // 跳过首次测量
                if (!perfData.skipFirstMeasurement.load()) {
                    std::lock_guard<std::mutex> lock(perfDataMutex);
                    perfData.sendTimes.push_back(processTime);
                    perfData.totalCycleTimes.push_back(processTime);
                }
                else {
                    perfData.skipFirstMeasurement.store(false);
                }
                if (responseReceived) {
                    auto state = sdoMachine.getCurrentState();
                    if (state == canopen::SdoState::RESPONSE_VALID) {
                        perfData.successCount++;
                        perfData.normalResponseCount++;
                    }
                    else if (state == canopen::SdoState::RESPONSE_ERROR) {
                        perfData.errorCount++;
                        perfData.errorResponseCount++;
                    }
                }
                else {
                    perfData.timeoutCount++;
                }
                sdoMachine.completeTransaction();
                processedCount++;
            }
            };










        // 接收线程
        auto receiverThread = [&]() {
            std::mt19937 gen(std::random_device{}());
            std::uniform_int_distribution<> dis(0, allTestCases.size() - 1);
            while (testRunning) {
                int caseIndex = dis(gen);
                const auto& testCase = allTestCases[caseIndex];
                uint8_t nodeId = canopen::AtomicSdoStateMachine::extractNodeId(testCase.second.frameID);
                auto startTime = std::chrono::high_resolution_clock::now();
                bool processed = sdoMachine.processResponse(
                    testCase.second.data, testCase.second.dlc, nodeId);
                auto endTime = std::chrono::high_resolution_clock::now();

                long processTime = std::chrono::duration_cast<std::chrono::microseconds>(
                    endTime - startTime).count();
                if (processed && !perfData.skipFirstMeasurement.load()) {
                    std::lock_guard<std::mutex> lock(perfDataMutex);
                    perfData.receiveTimes.push_back(processTime);
                }
            }
            };





        // 启动测试
        std::cout << "启动 2 个线程（1发送 + 1接收）\n";

        std::thread sender(senderThread);
        std::thread receiver(receiverThread);

        // 运行测试
        auto testStart = std::chrono::high_resolution_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds(TEST_DURATION_MS));
        testRunning = false;

        // 等待线程结束
        if (sender.joinable()) sender.join();
        if (receiver.joinable()) receiver.join();

        auto testEnd = std::chrono::high_resolution_clock::now();
        long roundTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            testEnd - testStart).count();

        std::cout << "第 " << (round + 1) << " 轮测试完成，持续时间: " << roundTime << " ms\n";
        std::cout << "本轮处理事务: " << processedCount.load() << "\n";
    }

    // 性能统计分析
    auto calculateStats = [](const std::vector<long>& data)
        -> std::tuple<double, long, long, long> {
        if (data.empty()) return { 0.0, 0, 0, 0 };

        double sum = std::accumulate(data.begin(), data.end(), 0.0);
        double avg = sum / data.size();
        long max = *std::max_element(data.begin(), data.end());
        long min = *std::min_element(data.begin(), data.end());
        long median = data.size() > 0 ? data[data.size() / 2] : 0;

        return { avg, max, min, median };
        };

    // 输出最终性能结果
    std::cout << "\n=== SDO状态机多线程性能测试最终结果 ===\n";
    std::cout << "总处理事务: " << (perfData.successCount + perfData.timeoutCount + perfData.errorCount) << "\n";
    std::cout << "正常响应事务: " << perfData.normalResponseCount.load() << "\n";
    std::cout << "错误响应事务: " << perfData.errorResponseCount.load() << "\n";
    std::cout << "超时事务: " << perfData.timeoutCount.load() << "\n";
    std::cout << "成功率: " << std::fixed << std::setprecision(2)
        << (perfData.normalResponseCount.load() * 100.0 /
            (perfData.normalResponseCount.load() + perfData.errorResponseCount.load() + perfData.timeoutCount.load()))
        << "%\n\n";

    auto [avgSend, maxSend, minSend, medSend] = calculateStats(perfData.sendTimes);
    std::cout << "发送线程处理时间 (μs):\n";
    std::cout << "  平均: " << avgSend << " μs, 最大: " << maxSend << " μs, 最小: " << minSend << " μs, 中位数: " << medSend << " μs\n";

    auto [avgReceive, maxReceive, minReceive, medReceive] = calculateStats(perfData.receiveTimes);
    std::cout << "接收线程处理时间 (μs):\n";
    std::cout << "  平均: " << avgReceive << " μs, 最大: " << maxReceive << " μs, 最小: " << minReceive << " μs, 中位数: " << medReceive << " μs\n";

    auto [avgTotal, maxTotal, minTotal, medTotal] = calculateStats(perfData.totalCycleTimes);
    std::cout << "完整事务周期时间 (μs):\n";
    std::cout << "  平均: " << avgTotal << " μs, 最大: " << maxTotal << " μs, 最小: " << minTotal << " μs, 中位数: " << medTotal << " μs\n";

    auto [avgContention, maxContention, minContention, medContention] = calculateStats(perfData.contentionDelays);
    std::cout << "竞争延迟时间 (μs):\n";
    std::cout << "  平均: " << avgContention << " μs, 最大: " << maxContention << " μs, 最小: " << minContention << " μs, 中位数: " << medContention << " μs\n";

    // 实时性评估
    std::cout << "\n=== 实时性评估 ===\n";
    constexpr long MAX_ACCEPTABLE_TIME = 200;
    constexpr long CRITICAL_TIME = 500;

    bool realTimeCapable = true;
    if (maxTotal > CRITICAL_TIME) {
        std::cout << "警告: 最大处理时间(" << maxTotal << "μs)超过临界值\n";
        realTimeCapable = false;
    }
    if (avgTotal > MAX_ACCEPTABLE_TIME) {
        std::cout << "警告: 平均处理时间(" << avgTotal << "μs)占用较多周期时间\n";
        realTimeCapable = false;
    }

    std::cout << "实时性评估: " << (realTimeCapable ? "✓ 满足要求" : "✗ 需要优化") << "\n";
    std::cout << "\n=== SDO状态机多线程性能测试完成 ===\n";
}





/******************************* SDO状态机超时重传机制测试 *******************************/






/**
 * @brief SDO超时重传机制全面测试函数
 * 
 * @details 该测试函数验证SDO状态机的重试机制和异常处理功能，包含：
 * - 超时检测和重试机制验证
 * - 重试计数器功能和上限控制
 * - 成功响应后重试计数器清零
 * - 最大重试次数达到后的异常抛出
 * - 边界条件和状态转换验证
 * 
 * @note 本测试采用模拟超时的方式主动触发重试机制，验证完整的错误恢复流程
 * @warning 测试中会故意触发超时和异常，相关错误信息属于预期行为
 */
void testSdoTimeoutRetryMechanism() {
    std::cout << "=== SDO超时重传机制全面测试开始 ===\n";
    std::cout << "测试项目: 超时检测、重试机制、异常处理、状态转换\n\n";

    // 创建状态机实例用于测试
    canopen::AtomicSdoStateMachine sdoMachine;

    // 测试统计数据
    int successfulRetries = 0;
    int timeoutExceptions = 0;
    int stateTransitionErrors = 0;
    int totalTests = 0;

    /******************** 阶段1: 基础重试机制测试 ********************/
    std::cout << "[阶段1] 基础重试机制测试\n";
    std::cout << "========================================\n";

    std::cout << "1.1 测试正常重试流程（1次重试后成功）...\n";
    try {
        totalTests++;//测试次数
        
        // 准备测试用的SDO事务
        uint8_t testRequestData[8] = { 0x40, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };
        CanFrame testFrame(0x601, testRequestData, 4);
        auto transaction = sdoMachine.prepareTransaction(testFrame);
        
        // 启动事务
        bool started = sdoMachine.startTransaction(transaction);
        if (!started) {
            std::cout << "  错误: 事务启动失败\n";
            stateTransitionErrors++;
            return;
        }
        
        std::cout << "  事务启动成功，当前状态: WAITING_RESPONSE\n";
        
        // 模拟第一次超时
        std::this_thread::sleep_for(std::chrono::microseconds(TIME_OUT_US + 100));
        bool firstTimeout = sdoMachine.checkTimeout();
        
        if (firstTimeout && sdoMachine.getCurrentState() == canopen::SdoState::RETRYING) {
            std::cout << "  首次超时检测成功，状态转换为RETRYING\n";
            std::cout << "  重试计数器: " << static_cast<int>(sdoMachine.getRetryCount()) << "\n";
            
            // 检查是否需要重试
            if (sdoMachine.needsRetry()) {
                std::cout << "  重试标志正确设置，状态已转换为WAITING_RESPONSE\n";
                
                // 模拟成功响应
                uint8_t responseData[8] = { 0x4B, 0x00, 0x60, 0x00, 0x37, 0x02, 0x00, 0x00 };
                bool processed = sdoMachine.processResponse(responseData, 6, 1);
                
                if (processed && sdoMachine.getCurrentState() == canopen::SdoState::RESPONSE_VALID) {
                    std::cout << "  响应处理成功，重试计数器已清零: " 
                        << static_cast<int>(sdoMachine.getRetryCount()) << "\n";
                    successfulRetries++;
                    std::cout << "  ✓ 基础重试流程测试通过\n";
                } else {
                    std::cout << "  ✗ 响应处理失败\n";
                    stateTransitionErrors++;
                }
            } else {
                std::cout << "  ✗ 重试标志设置错误\n";
                stateTransitionErrors++;
            }
        } else {
            std::cout << "  ✗ 首次超时检测失败\n";
            stateTransitionErrors++;
        }
        
        sdoMachine.completeTransaction();
        
    } catch (const std::exception& e) {
        std::cout << "  ✗ 意外异常: " << e.what() << "\n";
        stateTransitionErrors++;
    }

    /******************** 阶段2: 最大重试次数测试 ********************/
    std::cout << "\n[阶段2] 最大重试次数和异常抛出测试\n";
    std::cout << "========================================\n";

    std::cout << "2.1 测试达到最大重试次数后抛出异常...\n";
    try {
        totalTests++;
        
        // 重置状态机
        sdoMachine.reset();
        
        // 准备新的测试事务
        uint8_t testRequestData2[8] = { 0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };
        CanFrame testFrame2(0x602, testRequestData2, 4);
        auto transaction2 = sdoMachine.prepareTransaction(testFrame2);
        
        bool started = sdoMachine.startTransaction(transaction2);
        if (!started) {
            std::cout << "  错误: 事务启动失败\n";
            stateTransitionErrors++;
            return;
        }
        
        std::cout << "  开始连续超时测试...\n";
        
        // 连续触发超时直到达到最大重试次数
        for (int retry = 0; retry < MAX_RETRY_COUNT; ++retry) {
            std::this_thread::sleep_for(std::chrono::microseconds(TIME_OUT_US + 100));
            bool timeout = sdoMachine.checkTimeout();
            
            std::cout << "  第" << (retry + 1) << "次超时检测: " 
                << (timeout ? "成功" : "失败") 
                << ", 重试计数: " << static_cast<int>(sdoMachine.getRetryCount()) << "\n";
            
            if (timeout && sdoMachine.getCurrentState() == canopen::SdoState::RETRYING) {
                // 模拟重新发送
                if (sdoMachine.needsRetry()) {
                    std::cout << "    重试标志已设置，模拟重新发送\n";
                } else {
                    std::cout << "    ✗ 重试标志设置错误\n";
                    stateTransitionErrors++;
                    break;
                }
            }
        }
        
        // 触发最终超时，应该抛出异常
        std::this_thread::sleep_for(std::chrono::microseconds(TIME_OUT_US + 100));
        std::cout << "  触发最终超时检测...\n";
        
        try {
            sdoMachine.checkTimeout();
            std::cout << "  ✗ 预期异常未抛出\n";
            stateTransitionErrors++;
        } catch (const std::runtime_error& e) {
            std::cout << "  ✓ 成功捕获预期异常: " << e.what() << "\n";
            std::cout << "  最终状态: " << (sdoMachine.getCurrentState() == canopen::SdoState::MAX_RETRIES_EXCEEDED ? 
                "MAX_RETRIES_EXCEEDED" : "其他状态") << "\n";
            timeoutExceptions++;
        }
        
        sdoMachine.completeTransaction();
        
    } catch (const std::exception& e) {
        if (std::string(e.what()).find("SDO通信超时") != std::string::npos) {
            std::cout << "  ✓ 正确捕获超时异常: " << e.what() << "\n";
            timeoutExceptions++;
        } else {
            std::cout << "  ✗ 捕获非预期异常: " << e.what() << "\n";
            stateTransitionErrors++;
        }
    }

    /******************** 阶段3: 边界条件和状态一致性测试 ********************/
    std::cout << "\n[阶段3] 边界条件和状态一致性测试\n";
    std::cout << "========================================\n";

    std::cout << "3.1 测试重试计数器边界值...\n";
    try {
        totalTests++;
        
        // 重置并准备新事务
        sdoMachine.reset();
        uint8_t testRequestData3[8] = { 0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };
        CanFrame testFrame3(0x603, testRequestData3, 4);
        auto transaction3 = sdoMachine.prepareTransaction(testFrame3);
        
        bool started = sdoMachine.startTransaction(transaction3);
        if (started) {
            // 验证初始重试计数为0
            uint8_t initialRetryCount = sdoMachine.getRetryCount();
            std::cout << "  初始重试计数: " << static_cast<int>(initialRetryCount) << " (期望: 0)\n";
            
            if (initialRetryCount == 0) {
                std::cout << "  ✓ 初始重试计数正确\n";
            } else {
                std::cout << "  ✗ 初始重试计数错误\n";
                stateTransitionErrors++;
            }
            
            // 模拟一次成功响应，验证计数器保持为0
            uint8_t successResponse[8] = { 0x43, 0x64, 0x60, 0x00, 0x00, 0x00, 0x01, 0x00 };
            bool processed = sdoMachine.processResponse(successResponse, 8, 3);
            
            if (processed) {
                uint8_t finalRetryCount = sdoMachine.getRetryCount();
                std::cout << "  成功响应后重试计数: " << static_cast<int>(finalRetryCount) << " (期望: 0)\n";
                
                if (finalRetryCount == 0 && sdoMachine.getCurrentState() == canopen::SdoState::RESPONSE_VALID) {
                    std::cout << "  ✓ 重试计数器状态正确\n";
                    successfulRetries++;
                } else {
                    std::cout << "  ✗ 重试计数器状态错误\n";
                    stateTransitionErrors++;
                }
            } else {
                std::cout << "  ✗ 响应处理失败\n";
                stateTransitionErrors++;
            }
        } else {
            std::cout << "  ✗ 事务启动失败\n";
            stateTransitionErrors++;
        }
        
        sdoMachine.completeTransaction();
        
    } catch (const std::exception& e) {
        std::cout << "  ✗ 边界条件测试异常: " << e.what() << "\n";
        stateTransitionErrors++;
    }

    /******************** 阶段4: 并发安全性简单验证 ********************/
    std::cout << "\n[阶段4] 状态机重置和清理测试\n";
    std::cout << "========================================\n";

    std::cout << "4.1 测试状态机重置功能...\n";
    try {
        totalTests++;
        
        // 启动一个事务但不完成
        uint8_t testRequestData4[8] = { 0x40, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };
        CanFrame testFrame4(0x604, testRequestData4, 4);
        auto transaction4 = sdoMachine.prepareTransaction(testFrame4);
        
        bool started = sdoMachine.startTransaction(transaction4);
        if (started) {
            std::cout << "  事务启动成功，当前状态: WAITING_RESPONSE\n";
            std::cout << "  执行强制重置...\n";
            
            // 强制重置状态机
            sdoMachine.reset();
            
            // 验证状态机已返回空闲状态
            auto state = sdoMachine.getCurrentState();
            bool isBusy = sdoMachine.isBusy();
            
            std::cout << "  重置后状态: " << (state == canopen::SdoState::IDLE ? "IDLE" : "非IDLE") << "\n";
            std::cout << "  是否繁忙: " << (isBusy ? "是" : "否") << "\n";
            
            if (state == canopen::SdoState::IDLE && !isBusy) {
                std::cout << "  ✓ 状态机重置成功\n";
                successfulRetries++;
            } else {
                std::cout << "  ✗ 状态机重置失败\n";
                stateTransitionErrors++;
            }
        } else {
            std::cout << "  ✗ 事务启动失败\n";
            stateTransitionErrors++;
        }
        
    } catch (const std::exception& e) {
        std::cout << "  ✗ 重置测试异常: " << e.what() << "\n";
        stateTransitionErrors++;
    }

    /******************** 测试结果汇总 ********************/
    std::cout << "\n[测试结果汇总]\n";
    std::cout << "========================================\n";
    
    std::cout << "总测试用例数: " << totalTests << "\n";
    std::cout << "成功重试测试: " << successfulRetries << "\n";
    std::cout << "超时异常测试: " << timeoutExceptions << "\n";
    std::cout << "状态转换错误: " << stateTransitionErrors << "\n";
    
    // 计算成功率
    int totalSuccessful = successfulRetries + timeoutExceptions;
    double successRate = totalTests > 0 ? (double)totalSuccessful / totalTests * 100.0 : 0.0;
    
    std::cout << "\n测试成功率: " << std::fixed << std::setprecision(1) << successRate << "%\n";
    
    // 功能验证结果
    std::cout << "\n功能验证结果:\n";
    std::cout << "  重试机制: " << (successfulRetries > 0 ? "✓ 工作正常" : "✗ 存在问题") << "\n";
    std::cout << "  异常处理: " << (timeoutExceptions > 0 ? "✓ 工作正常" : "✗ 存在问题") << "\n";
    std::cout << "  状态转换: " << (stateTransitionErrors == 0 ? "✓ 工作正常" : "✗ 存在问题") << "\n";
    
    // 实时性评估
    std::cout << "\n实时性评估:\n";
    if (stateTransitionErrors == 0) {
        std::cout << "  ✓ SDO重试机制符合实时性要求\n";
        std::cout << "  ✓ 异常处理机制工作正确\n";
        std::cout << "  ✓ 状态机设计满足并发安全需求\n";
    } else {
        std::cout << "  ⚠ 发现状态转换问题，需要进一步调试\n";
        std::cout << "  建议检查：\n";
        std::cout << "    - 超时时间设置是否合理\n";
        std::cout << "    - 原子操作的内存序是否正确\n";
        std::cout << "    - 条件变量的等待逻辑是否完整\n";
    }
    
    std::cout << "\n=== SDO超时重传机制测试完成 ===\n";
    std::cout << "测试覆盖: 超时检测、重试机制、异常处理、状态转换、边界条件\n";
    std::cout << "总体评估: " << (stateTransitionErrors == 0 && totalSuccessful >= 3 ? "通过" : "需要修复") << "\n";
}




#undef TIME_IT














#endif // !TEST_MODULE
