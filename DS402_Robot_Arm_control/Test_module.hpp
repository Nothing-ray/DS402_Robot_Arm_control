#ifndef TEST_MODULE
#define TEST_MODULE

#include <array>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>


#include "CLASS_Motor.hpp"
#include "PDO_config.hpp"
#include "Data_processing.hpp"
#include "CAN_frame.hpp"
#include "CAN_processing.hpp"

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






#undef TIME_IT














#endif // !TEST_MODULE
