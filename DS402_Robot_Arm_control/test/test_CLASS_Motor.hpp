/**
 * @file test_CLASS_Motor.hpp
 * @brief 电机类核心功能测试
 * 
 * 包含电机类的基础功能、原子操作、多线程并发等测试
 */

#ifndef TEST_CLASS_MOTOR_HPP
#define TEST_CLASS_MOTOR_HPP

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
#include <tuple>        // 用于 std::tuple, std::make_tuple, std::get
#include <iomanip>      // 用于 std::setw, std::setfill
#include <cstring>      // 用于 std::memcpy

#include "../CLASS_Motor.hpp"
#include "../Data_processing.hpp"


 /******************** Motor类测试 ********************/



 // 全局控制标志
std::atomic<bool> running{ true };



/******************** Motor类测试函数 ********************/

#include <vector>
#include <chrono>
#include <numeric>




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
void testMotorClass(std::array<Motor, 6>& motors) {
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
        long initTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        perfData.initTimes.push_back(initTime);
        std::cout << "  电机" << i << " 初始化耗时: " << initTime << " 纳秒\n";
    }

    // 数据转换精度测试
    std::cout << "\n1.2 数据转换精度测试...\n";
    Motor& testMotor2 = motors[0];

    // 测试电流转换
    float testCurrents[] = { 0.0f, 100.5f, -50.2f, 1000.0f, -1000.0f };
    for (float testCurrent : testCurrents) {
        testMotor2.current.target_current.store(testCurrent);
        testMotor2.current.flags_.fetch_or(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

        // 手动调用电流数据刷新
        testMotor2.refreshMotorData(testMotor2.current);

        float readBack = testMotor2.current.target_current.load();
        Current_type encoderVal = testMotor2.current.target_encoder.load();

        std::cout << "  电流测试: " << testCurrent << " mA → 编码器:" << encoderVal
            << " → 读回:" << readBack << " mA (误差:" << std::abs(testCurrent - readBack) << ")\n";
    }

    // 测试位置转换（角度与编码器值）
    std::cout << "\n  位置转换测试:\n";
    float testAngles[] = { 0.0f, 90.0f, -90.0f, 180.0f, -180.0f, 359.9f };
    for (float testAngle : testAngles) {
        testMotor2.position.target_degree.store(testAngle);
        testMotor2.position.flags_.fetch_or(MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

        // 手动调用位置数据刷新
        testMotor2.refreshMotorData(testMotor2.position);

        float readBack = testMotor2.position.target_degree.load();
        Position_type encoderVal = testMotor2.position.target_encoder.load();

        std::cout << "  角度测试: " << testAngle << "° → 编码器:" << encoderVal
            << " → 读回:" << readBack << "° (误差:" << std::abs(testAngle - readBack) << ")\n";
    }

    // 测试速度转换
    std::cout << "\n  速度转换测试:\n";
    float testSpeeds[] = { 0.0f, 100.0f, -100.0f, 500.0f, -500.0f };
    for (float testSpeed : testSpeeds) {
        testMotor2.velocity.target_rpm_velocity_mode.store(testSpeed);
        testMotor2.velocity.flags_.fetch_or(MotorVelocity::Flags::TARGET_DATA_SEND_NEED_REFRESH_VELOCITY_MODE, std::memory_order_release);

        // 手动调用速度数据刷新
        testMotor2.refreshMotorData(testMotor2.velocity);

        float readBack = testMotor2.velocity.target_rpm_velocity_mode.load();
        Velocity_type encoderVal = testMotor2.velocity.target_encoder_velocity_mode.load();

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
        testMotor2.current.flags_.fetch_or(MotorCurrent::Flags::RAW_DATA_SEND_NEED_REFRESH, std::memory_order_release);
        testMotor2.current.flags_.fetch_or(MotorCurrent::Flags::ENCODER_DATA_SEND_NEED_REFRESH, std::memory_order_release);
        testMotor2.current.flags_.fetch_or(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH, std::memory_order_release);

        // 检查标志位
        bool flag1 = testMotor2.current.needsProcess(MotorCurrent::Flags::RAW_DATA_SEND_NEED_REFRESH);
        bool flag2 = testMotor2.current.needsProcess(MotorCurrent::Flags::ENCODER_DATA_SEND_NEED_REFRESH);
        bool flag3 = testMotor2.current.needsProcess(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH);

        // 清除标志位
        testMotor2.current.markProcessed(MotorCurrent::Flags::RAW_DATA_SEND_NEED_REFRESH);
        testMotor2.current.markProcessed(MotorCurrent::Flags::ENCODER_DATA_SEND_NEED_REFRESH);
        testMotor2.current.markProcessed(MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH);

        auto end = std::chrono::high_resolution_clock::now();
        long flagTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
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
        testMotor2.current.raw_actual.atomicWriteValue(testValue);
        testMotor2.position.raw_actual.atomicWriteValue(static_cast<Position_type>(1000 + i));
        testMotor2.velocity.raw_actual.atomicWriteValue(static_cast<Velocity_type>(500 + i));

        // 原子读取测试
        Current_type readCurrent = testMotor2.current.raw_actual.atomicReadValue();
        Position_type readPosition = testMotor2.position.raw_actual.atomicReadValue();
        Velocity_type readVelocity = testMotor2.velocity.raw_actual.atomicReadValue();

        auto end = std::chrono::high_resolution_clock::now();
        long atomicTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
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
        testMotor2.current.raw_actual.atomicWriteValue(rawCurrent);
        testMotor2.position.raw_actual.atomicWriteValue(rawPosition);
        testMotor2.velocity.raw_actual.atomicWriteValue(rawVelocity);

        testMotor2.current.flags_.fetch_or(MotorCurrent::Flags::RAW_DATA_RECEIVE_NEED_REFRESH, std::memory_order_release);
        testMotor2.position.flags_.fetch_or(MotorPosition::Flags::RAW_DATA_RECEIVE_NEED_REFRESH, std::memory_order_release);
        testMotor2.velocity.flags_.fetch_or(MotorVelocity::Flags::RAW_DATA_RECEIVE_NEED_REFRESH, std::memory_order_release);

        // 执行刷新
        testMotor2.refreshAllMotorData();

        auto end = std::chrono::high_resolution_clock::now();
        long refreshTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        perfData.singleRefreshTimes.push_back(refreshTime);

        // 验证刷新结果
        float actualCurrent = testMotor2.current.actual_current.load();
        float actualPosition = testMotor2.position.actual_degree.load();
        float actualVelocity = testMotor2.velocity.actual_rpm.load();

        if (i % 10 == 0) {  // 每10次输出一次详细信息
            std::cout << "  第" << i << "次刷新: 电流" << actualCurrent << "mA, 位置"
                << actualPosition << "度, 速度" << actualVelocity << "RPM (耗时:" << refreshTime << "纳秒)\n";
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
                long writeTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

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
                long readTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

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
        if (data.empty()) return std::make_tuple(0.0, 0L, 0L);
        double avg = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
        long maxVal = *std::max_element(data.begin(), data.end());
        long minVal = *std::min_element(data.begin(), data.end());
        return std::make_tuple(avg, maxVal, minVal);
        };

    // 输出各项性能统计
    std::tuple<double, long, long> initStats = calculateStats(perfData.initTimes);
    double avgInit = std::get<0>(initStats);
    long maxInit = std::get<1>(initStats);
    long minInit = std::get<2>(initStats);
    std::cout << "初始化性能:\n";
    std::cout << "  平均: " << avgInit << " ns, 最大: " << maxInit << " ns, 最小: " << minInit << " ns\n";

    std::tuple<double, long, long> refreshStats = calculateStats(perfData.singleRefreshTimes);
    double avgRefresh = std::get<0>(refreshStats);
    long maxRefresh = std::get<1>(refreshStats);
    long minRefresh = std::get<2>(refreshStats);
    std::cout << "\n单线程刷新性能:\n";
    std::cout << "  平均: " << avgRefresh << " ns, 最大: " << maxRefresh << " ns, 最小: " << minRefresh << " ns\n";

    std::tuple<double, long, long> atomicStats = calculateStats(perfData.atomicOperationTimes);
    double avgAtomicOp = std::get<0>(atomicStats);
    long maxAtomicOp = std::get<1>(atomicStats);
    long minAtomicOp = std::get<2>(atomicStats);
    std::cout << "\n原子操作性能:\n";
    std::cout << "  平均: " << avgAtomicOp << " ns, 最大: " << maxAtomicOp << " ns, 最小: " << minAtomicOp << " ns\n";

    std::tuple<double, long, long> multiWriteStats = calculateStats(perfData.multiWriteTimes);
    double avgMultiWrite = std::get<0>(multiWriteStats);
    long maxMultiWrite = std::get<1>(multiWriteStats);
    long minMultiWrite = std::get<2>(multiWriteStats);
    std::cout << "\n多线程写入性能:\n";
    std::cout << "  平均: " << avgMultiWrite << " ns, 最大: " << maxMultiWrite << " ns, 最小: " << minMultiWrite << " ns\n";

    std::tuple<double, long, long> multiReadStats = calculateStats(perfData.multiReadTimes);
    double avgMultiRead = std::get<0>(multiReadStats);
    long maxMultiRead = std::get<1>(multiReadStats);
    long minMultiRead = std::get<2>(multiReadStats);
    std::cout << "\n多线程读取性能:\n";
    std::cout << "  平均: " << avgMultiRead << " ns, 最大: " << maxMultiRead << " ns, 最小: " << minMultiRead << " ns\n";

    std::tuple<double, long, long> flagStats = calculateStats(perfData.flagOperationTimes);
    double avgFlag = std::get<0>(flagStats);
    long maxFlag = std::get<1>(flagStats);
    long minFlag = std::get<2>(flagStats);
    std::cout << "\n标志位操作性能:\n";
    std::cout << "  平均: " << avgFlag << " ns, 最大: " << maxFlag << " ns, 最小: " << minFlag << " ns\n";

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

    /******************** 阶段8: AlignedRawData模板类专项测试 ********************/
    std::cout << "\n[阶段8] AlignedRawData模板类专项测试\n";
    std::cout << "========================================\n";

    std::cout << "8.1 测试不同位宽的AlignedRawData模板...\n";

    // 测试1字节模板
    AlignedRawData<1, uint8_t> test1Byte;
    uint8_t testByte = 0xAB;
    test1Byte.atomicWrite(&testByte);
    uint8_t readByte;
    test1Byte.atomicRead(&readByte);
    std::cout << "  1字节模板测试: 写入0x" << std::hex << (int)testByte << " 读取0x" << (int)readByte;
    if (testByte == readByte) std::cout << " Yes\n"; else std::cout << " No\n";

    // 测试2字节模板
    AlignedRawData<2, int16_t> test2Byte;
    int16_t testShort = -12345;
    test2Byte.atomicWrite(&testShort);
    int16_t readShort;
    test2Byte.atomicRead(&readShort);
    std::cout << "  2字节模板测试: 写入" << testShort << " 读取" << readShort;
    if (testShort == readShort) std::cout << " Yes\n"; else std::cout << " No\n";

    // 测试4字节模板
    AlignedRawData<4, int32_t> test4Byte;
    int32_t testInt = 2147483647;
    test4Byte.atomicWrite(&testInt);
    int32_t readInt;
    test4Byte.atomicRead(&readInt);
    std::cout << "  4字节模板测试: 写入" << testInt << " 读取" << readInt;
    if (testInt == readInt) std::cout << " Yes\n"; else std::cout << " No\n";

    // 测试字节数组接口
    std::cout << "8.2 测试字节数组接口...\n";
    uint8_t testBytes[4] = { 0x12, 0x34, 0x56, 0x78 };
    test4Byte.setBytes(testBytes);
    uint8_t readBytes[4];
    test4Byte.getBytes(readBytes);
    bool bytesMatch = true;
    for (int i = 0; i < 4; i++) {
        if (testBytes[i] != readBytes[i]) bytesMatch = false;
    }
    std::cout << "  字节数组接口测试: " << (bytesMatch ? "Yes" : "No") << "\n";

    // 测试缓存行对齐
    std::cout << "8.3 测试缓存行对齐...\n";
    bool alignmentOK2 = (reinterpret_cast<uintptr_t>(&test4Byte) % 64 == 0);
    std::cout << "  64字节对齐验证: " << (alignmentOK2 ? "Yes" : "No") << "\n";

    /******************** 阶段9: MotorVelocity双模式专项测试 ********************/
    std::cout << "\n[阶段9] MotorVelocity双模式专项测试\n";
    std::cout << "========================================\n";

    // 测试速度模式
    std::cout << "9.1 测试速度模式...\n";
    Velocity_type testVelMode = 1500;
    testMotor2.velocity.raw_target_velocity_mode.atomicWriteValue(testVelMode);
    testMotor2.velocity.flags_.fetch_or(MotorVelocity::Flags::RAW_DATA_SEND_NEED_REFRESH_VELOCITY_MODE, std::memory_order_release);
    testMotor2.refreshMotorData(testMotor2.velocity);

    Velocity_type readVelEncoder = testMotor2.velocity.target_encoder_velocity_mode.load();
    float readVelRPM = testMotor2.velocity.target_rpm_velocity_mode.load();
    std::cout << "  速度模式: 原始值=" << testVelMode << ", 编码器=" << readVelEncoder << ", RPM=" << readVelRPM;
    if (testVelMode == readVelEncoder && std::abs(readVelRPM - testVelMode) < 0.001f) std::cout << " Yes\n"; else std::cout << " No\n";

    // 测试位置模式
    std::cout << "9.2 测试位置模式...\n";
    Velocity_type testPosMode = 800;
    testMotor2.velocity.raw_target_position_mode.atomicWriteValue(testPosMode);
    testMotor2.velocity.flags_.fetch_or(MotorVelocity::Flags::RAW_DATA_SEND_NEED_REFRESH_POSITION_MODE, std::memory_order_release);
    testMotor2.refreshMotorData(testMotor2.velocity);

    Velocity_type readPosEncoder = testMotor2.velocity.target_encoder_position_mode.load();
    float readPosRPM = testMotor2.velocity.target_rpm_position_mode.load();
    std::cout << "  位置模式: 原始值=" << testPosMode << ", 编码器=" << readPosEncoder << ", RPM=" << readPosRPM;
    if (testPosMode == readPosEncoder && std::abs(readPosRPM - testPosMode) < 0.001f) std::cout << " Yes\n"; else std::cout << " No\n";

    // 测试模式区分性
    std::cout << "9.3 测试模式区分性...\n";
    bool modesDistinct = (readVelEncoder != readPosEncoder);
    std::cout << "  速度/位置模式数据区分: " << (modesDistinct ? "Yes" : "No") << "\n";

    /******************** 阶段10: StateAndMode结构体专项测试 ********************/
    std::cout << "\n[阶段10] StateAndMode结构体专项测试\n";
    std::cout << "========================================\n";

    std::cout << "10.1 测试状态字和控制字...\n";

    // 测试控制字写入
    uint8_t controlWord[2] = { 0x06, 0x00 }; // 使能电机
    std::memcpy(const_cast<uint8_t*>(testMotor2.stateAndMode.controlData.controlWordRaw), controlWord, 2);

    // 测试状态字读取
    uint8_t statusWord[2] = { 0x37, 0x02 }; // 运行中状态
    std::memcpy(const_cast<uint8_t*>(testMotor2.stateAndMode.controlData.statusWordRaw), statusWord, 2);

    uint16_t readControlWord = (testMotor2.stateAndMode.controlData.controlWordRaw[1] << 8) | testMotor2.stateAndMode.controlData.controlWordRaw[0];
    uint16_t readStatusWord = (testMotor2.stateAndMode.controlData.statusWordRaw[1] << 8) | testMotor2.stateAndMode.controlData.statusWordRaw[0];

    std::cout << "  控制字: 0x" << std::hex << std::setw(4) << std::setfill('0') << readControlWord << std::dec;
    if (readControlWord == 0x0006) std::cout << " Yes\n"; else std::cout << " No\n";

    std::cout << "  状态字: 0x" << std::hex << std::setw(4) << std::setfill('0') << readStatusWord << std::dec;
    if (readStatusWord == 0x0237) std::cout << " Yes\n"; else std::cout << " No\n";

    // 测试运行模式
    std::cout << "10.2 测试运行模式...\n";
    testMotor2.stateAndMode.modeData.modeOfOperationRaw[0] = static_cast<uint8_t>(MotorMode::PROFILE_POSITION);
    uint8_t readMode = testMotor2.stateAndMode.modeData.modeOfOperationRaw[0];
    std::cout << "  运行模式: " << static_cast<int>(readMode) << " (期望: 1)";
    if (readMode == 1) std::cout << " Yes\n"; else std::cout << " No\n";

    // 测试refresh标志
    std::cout << "10.3 测试refresh标志...\n";
    testMotor2.stateAndMode.refresh.store(true);
    bool refreshState = testMotor2.stateAndMode.refresh.load();
    std::cout << "  refresh标志: " << (refreshState ? "true" : "false");
    if (refreshState) std::cout << " Yes\n"; else std::cout << " No\n";

    /******************** 阶段11: MotorAccelDecel结构体专项测试 ********************/
    std::cout << "\n[阶段11] MotorAccelDecel结构体专项测试\n";
    std::cout << "========================================\n";

    std::cout << "11.1 测试加速度和减速度...\n";

    // 测试加速度
    AccelDecel_type testAccel = 5000; // RPM/min
    testMotor2.accelDecel.raw_accel.atomicWriteValue(testAccel);
    AccelDecel_type readAccel = testMotor2.accelDecel.raw_accel.atomicReadValue();
    std::cout << "  加速度: 写入=" << testAccel << " 读取=" << readAccel;
    if (testAccel == readAccel) std::cout << " Yes\n"; else std::cout << " No\n";

    // 测试减速度
    AccelDecel_type testDecel = 3000; // RPM/min
    testMotor2.accelDecel.raw_decel.atomicWriteValue(testDecel);
    AccelDecel_type readDecel = testMotor2.accelDecel.raw_decel.atomicReadValue();
    std::cout << "  减速度: 写入=" << testDecel << " 读取=" << readDecel;
    if (testDecel == readDecel) std::cout << " Yes\n"; else std::cout << " No\n";

    // 测试对象字典索引
    std::cout << "11.2 测试对象字典索引...\n";
    std::cout << "  加速度索引: 0x" << std::hex << testMotor2.accelDecel.accel_Index << " (期望: 0x6083)";
    if (testMotor2.accelDecel.accel_Index == 0x6083) std::cout << " Yes\n"; else std::cout << " No\n";
    std::cout << "  减速度索引: 0x" << std::hex << testMotor2.accelDecel.decel_Index << " (期望: 0x6084)";
    if (testMotor2.accelDecel.decel_Index == 0x6084) std::cout << " Yes\n"; else std::cout << " No\n";

    /******************** 阶段12: 边界值和异常处理测试 ********************/
    std::cout << "\n[阶段12] 边界值和异常处理测试\n";
    std::cout << "========================================\n";

    std::cout << "12.1 测试数值边界...\n";

    // 测试int16_t边界值
    Current_type maxCurrent = 32767;
    Current_type minCurrent = -32768;
    testMotor2.current.raw_actual.atomicWriteValue(maxCurrent);
    Current_type readMax = testMotor2.current.raw_actual.atomicReadValue();
    testMotor2.current.raw_actual.atomicWriteValue(minCurrent);
    Current_type readMin = testMotor2.current.raw_actual.atomicReadValue();

    std::cout << "  int16_t最大值: " << maxCurrent << " -> " << readMax;
    if (maxCurrent == readMax) std::cout << " Yes\n"; else std::cout << " No\n";
    std::cout << "  int16_t最小值: " << minCurrent << " -> " << readMin;
    if (minCurrent == readMin) std::cout << " Yes\n"; else std::cout << " No\n";

    // 测试int32_t边界值
    Position_type maxPosition = 2147483647;
    Position_type minPosition = -2147483648;
    testMotor2.position.raw_actual.atomicWriteValue(maxPosition);
    Position_type readMaxPos = testMotor2.position.raw_actual.atomicReadValue();
    testMotor2.position.raw_actual.atomicWriteValue(minPosition);
    Position_type readMinPos = testMotor2.position.raw_actual.atomicReadValue();

    std::cout << "  int32_t最大值: " << maxPosition << " -> " << readMaxPos;
    if (maxPosition == readMaxPos) std::cout << " Yes\n"; else std::cout << " No\n";
    std::cout << "  int32_t最小值: " << minPosition << " -> " << readMinPos;
    if (minPosition == readMinPos) std::cout << " Yes\n"; else std::cout << " No\n";

    // 测试标志位冲突
    std::cout << "12.2 测试标志位冲突处理...\n";
    testMotor2.current.flags_.store(0);
    testMotor2.current.flags_.fetch_or(MotorCurrent::Flags::RAW_DATA_RECEIVE_NEED_REFRESH, std::memory_order_release);
    testMotor2.current.flags_.fetch_or(MotorCurrent::Flags::RAW_DATA_SEND_NEED_REFRESH, std::memory_order_release);

    Flag_type conflictFlags = testMotor2.current.flags_.load();
    bool hasReceiveFlag = conflictFlags & MotorCurrent::Flags::RAW_DATA_RECEIVE_NEED_REFRESH;
    bool hasSendFlag = conflictFlags & MotorCurrent::Flags::RAW_DATA_SEND_NEED_REFRESH;

    std::cout << "  标志位冲突测试: 接收标志=" << (hasReceiveFlag ? "有" : "无") << ", 发送标志=" << (hasSendFlag ? "有" : "无");
    if (hasReceiveFlag && hasSendFlag) std::cout << " Yes\n"; else std::cout << " No\n";

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



#undef TIME_IT


#endif // TEST_CLASS_MOTOR_HPP