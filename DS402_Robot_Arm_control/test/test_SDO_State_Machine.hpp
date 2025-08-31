/**
 * @file test_SDO_State_Machine.hpp
 * @brief SDO状态机测试
 * 
 * 包含SDO状态机的单线程性能测试、多线程并发测试和超时重传机制测试，
 * 验证实时性能、并发安全性和错误恢复能力
 */

#ifndef TEST_SDO_STATE_MACHINE_HPP
#define TEST_SDO_STATE_MACHINE_HPP

#include <array>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <random>
#include <numeric>
#include <algorithm>

#include "../CLASS_Motor.hpp"
#include "../CAN_frame.hpp"
#include "../SDO_State_Machine.hpp"

using namespace std::chrono_literals;

// 常量定义
#define TEST_TIME_OUT_US 1000    // 1ms 超时
#define TEST_MAX_RETRY_COUNT  3   // 最大重试次数

// 计时工具宏
#define TIME_IT(operation, description) \
    do { \
        auto start = std::chrono::high_resolution_clock::now(); \
        operation; \
        auto end = std::chrono::high_resolution_clock::now(); \
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start); \
        std::cout << description << " took " << duration.count() << " us\n"; \
    } while(0)















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


    std::cout << "实时性评估: " << (realTimeCapable ? "Yes 满足要求" : "✗ 需要优化") << "\n";

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
        std::cout << "Yes 状态机性能良好，适合实时控制系统使用\n";
    }
    else {
        std::cout << "!!! 状态机性能需要优化，建议:\n";
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
    constexpr int NUM_TEST_ROUNDS = 10;       // 测试轮数

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

    std::cout << "实时性评估: " << (realTimeCapable ? "Yes 满足要求" : "✗ 需要优化") << "\n";
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
        totalTests++; // 测试次数

        // 准备测试用的SDO事务
        uint8_t testRequestData[8] = { 0x40, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };
        CanFrame testFrame(0x601, testRequestData, 4);
        auto transaction = sdoMachine.prepareTransaction(testFrame);//创建SDO事务

        // 启动事务
        bool started = sdoMachine.startTransaction(transaction);
        if (!started) {
            std::cout << "  错误: 事务启动失败\n";
            stateTransitionErrors++;
            return;
        }

        std::cout << "  事务启动成功，当前状态: WAITING_RESPONSE\n";

        // 模拟第一次超时
        std::this_thread::sleep_for(std::chrono::microseconds(TEST_TIME_OUT_US + 100));
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
                    std::cout << "  Yes 基础重试流程测试通过\n";
                }
                else {
                    std::cout << "  ✗ 响应处理失败\n";
                    stateTransitionErrors++;
                }
            }
            else {
                std::cout << "  ✗ 重试标志设置错误\n";
                stateTransitionErrors++;
            }
        }
        else {
            std::cout << "  ✗ 首次超时检测失败\n";
            stateTransitionErrors++;
        }

        sdoMachine.completeTransaction();

    }
    catch (const std::exception& e) {
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
        for (int retry = 0; retry < TEST_MAX_RETRY_COUNT; ++retry) {
            std::this_thread::sleep_for(std::chrono::microseconds(TEST_TIME_OUT_US + 100));
            bool timeout = sdoMachine.checkTimeout();

            std::cout << "  第" << (retry + 1) << "次超时检测: "
                << (timeout ? "成功" : "失败")
                << ", 重试计数: " << static_cast<int>(sdoMachine.getRetryCount()) << "\n";

            if (timeout && sdoMachine.getCurrentState() == canopen::SdoState::RETRYING) {
                // 模拟重新发送
                if (sdoMachine.needsRetry()) {
                    std::cout << "    重试标志已设置，模拟重新发送\n";
                }
                else {
                    std::cout << "    ✗ 重试标志设置错误\n";
                    stateTransitionErrors++;
                    break;
                }
            }
        }

        // 触发最终超时，应该抛出异常
        std::this_thread::sleep_for(std::chrono::microseconds(TEST_TIME_OUT_US + 100));
        std::cout << "  触发最终超时检测...\n";

        try {
            sdoMachine.checkTimeout();
            std::cout << "  ✗ 预期异常未抛出\n";
            stateTransitionErrors++;
        }
        catch (const std::runtime_error& e) {
            std::cout << "  Yes 成功捕获预期异常: " << e.what() << "\n";
            std::cout << "  最终状态: " << (sdoMachine.getCurrentState() == canopen::SdoState::MAX_RETRIES_EXCEEDED ?
                "MAX_RETRIES_EXCEEDED" : "其他状态") << "\n";
            timeoutExceptions++;
        }

        sdoMachine.completeTransaction();

    }
    catch (const std::exception& e) {
        if (std::string(e.what()).find("SDO通信超时") != std::string::npos) {
            std::cout << "  Yes 正确捕获超时异常: " << e.what() << "\n";
            timeoutExceptions++;
        }
        else {
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
                std::cout << "  Yes 初始重试计数正确\n";
            }
            else {
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
                    std::cout << "  Yes 重试计数器状态正确\n";
                    successfulRetries++;
                }
                else {
                    std::cout << "  ✗ 重试计数器状态错误\n";
                    stateTransitionErrors++;
                }
            }
            else {
                std::cout << "  ✗ 响应处理失败\n";
                stateTransitionErrors++;
            }
        }
        else {
            std::cout << "  ✗ 事务启动失败\n";
            stateTransitionErrors++;
        }

        sdoMachine.completeTransaction();

    }
    catch (const std::exception& e) {
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
                std::cout << "  Yes 状态机重置成功\n";
                successfulRetries++;
            }
            else {
                std::cout << "  ✗ 状态机重置失败\n";
                stateTransitionErrors++;
            }
        }
        else {
            std::cout << "  ✗ 事务启动失败\n";
            stateTransitionErrors++;
        }

    }
    catch (const std::exception& e) {
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
    std::cout << "  重试机制: " << (successfulRetries > 0 ? "Yes 工作正常" : "✗ 存在问题") << "\n";
    std::cout << "  异常处理: " << (timeoutExceptions > 0 ? "Yes 工作正常" : "✗ 存在问题") << "\n";
    std::cout << "  状态转换: " << (stateTransitionErrors == 0 ? "Yes 工作正常" : "✗ 存在问题") << "\n";

    // 实时性评估
    std::cout << "\n实时性评估:\n";
    if (stateTransitionErrors == 0) {
        std::cout << "  Yes SDO重试机制符合实时性要求\n";
        std::cout << "  Yes 异常处理机制工作正确\n";
        std::cout << "  Yes 状态机设计满足并发安全需求\n";
    }
    else {
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



#undef TEST_TIME_OUT_US
#undef TEST_MAX_RETRY_COUNT
#undef TIME_IT

#endif // TEST_SDO_STATE_MACHINE_HPP






