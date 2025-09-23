/**
 * @file Send_Thread.hpp
 * @brief 发送线程实现 - CANopen DS402协议机械臂驱动程序
 * 
 * @details 本文件实现了机械臂控制系统的发送线程基础框架，负责处理SDO和PDO帧的发送。
 * 采用2ms同步周期设计，通过依赖注入方式访问全局资源。
 * 
 * @note 发送线程不负责全局资源的初始化，只通过引用访问
 * @warning 所有全局资源必须由主线程初始化后再启动发送线程
 */

#ifndef SEND_THREAD_HPP
#define SEND_THREAD_HPP

#include <atomic>
#include <thread>
#include <chrono>
#include <vector>
#include <queue>
#include <iostream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <map>

#include "CAN_frame.hpp"
#include "CircularBuffer.hpp"
#include "SDO_State_Machine.hpp"
#include "CLASS_Motor.hpp"
#include "Serial_Module.hpp"
#include "PDO_config.hpp"

// 配置宏定义
#ifndef SEND_THREAD_CONFIG_H
#define SEND_THREAD_CONFIG_H

// 超时处理详细模式（默认关闭）
// #define ENABLE_DETAILED_TIMEOUT_HANDLING

//调试输出开关（修改后面的值）
#define ENABLE_DEBUG_OUTPUT false

// 周期计时统计开关（默认开启，测量每个周期的处理时间）
#define ENABLE_CYCLE_TIMING


// 调试输出控制（零开销宏）
#if ENABLE_DEBUG_OUTPUT
#define DEBUG_PRINT(msg) do { std::cout << msg << std::endl; } while(0)
#define DEBUG_PRINT_IF(cond, msg) do { if (cond) { std::cout << msg << std::endl; } } while(0)
#else
#define DEBUG_PRINT(msg) do { } while(0)
#define DEBUG_PRINT_IF(cond, msg) do { } while(0)
#endif

// SDO测试模式（禁用PDO处理）
#ifdef TESTING_SDO_ONLY
#define DISABLE_PDO_PROCESSING
#endif

#endif // SEND_THREAD_CONFIG_H


//多少个周期进行一次性能统计
#define CYCLE_COUNT 20 



/**
 * @brief 发送线程管理类
 * 
 * 负责管理CAN帧的发送流程，通过依赖注入方式访问全局资源。
 * 采用单事务设计确保总线访问的确定性。
 */
class SendThread {
private:
    // 通过引用访问全局资源（由主线程初始化）
    CircularBuffer& sendBuffer_;        ///< 发送环形缓冲区
    CircularBuffer& planBuffer_;        ///< 规划环形缓冲区
    std::array<Motor, 6>& motors_;
    SerialPortManager& serialManager_;  ///< 串口管理器引用
    
    // PDO映射表引用（用于构建CAN帧）
    const std::vector<PdoMappingEntry>& pdoMappingTable_;  ///< PDO映射表引用

        
    // 线程控制
    std::atomic<bool> running_{false};
    std::thread sendThread_;
    canopen::AtomicSdoStateMachine sdoStateMachine_;
    
    // SDO批次管理（简化状态机）
    std::atomic<uint8_t> sdoBatchRemaining_{0};        ///< 当前批次剩余帧数（0-255，足够覆盖正常场景）
    std::atomic<bool> sdoBatchActive_{false};           ///< 批次激活标志
    
    // 配置参数
    static constexpr uint8_t MOTOR_ARRAY_SIZE = 6;  // 电机数组固定大小
    uint8_t motorCount_;
    
        
    // 预分配的PDO数据结构（固定大小数组，零分配开销）
    static constexpr size_t PDO_MAX_MOTORS = 12;  // PDO系统支持的最大电机数
    static constexpr size_t MAX_PDO_SLOTS = PDO_MAX_MOTORS * PDO_CHANNELS_PER_MOTOR;
    
    struct PdoDataSlot {
        uint32_t cobId;
        std::array<uint8_t, 8> data;
        bool isValid;
        bool hasValidData;  // 新增：标记是否有有效数据需要发送

        PdoDataSlot() : cobId(0), isValid(false), hasValidData(false) {
            data.fill(0);
        }
    };
    
    alignas(64) std::array<PdoDataSlot, MAX_PDO_SLOTS> pdoDataMap_;  // 缓存行对齐
    
    // 预分配的CAN帧数组（栈上分配，零堆开销）
    std::array<CanFrame, MAX_PDO_SLOTS> framesToSend_;
    size_t validFrameCount_{0};  // 有效帧计数
    
    // 性能监控（调试用）
    std::chrono::steady_clock::time_point lastCycleTime_;
    uint32_t cycleCount_{0};

    // 全局帧数计数器（用于调试输出）
    std::atomic<uint64_t> globalFrameCounter_{0};

    // 预构建的同步帧（避免重复构建开销）
    CanFrame syncFrame_;

#ifdef ENABLE_CYCLE_TIMING
    // 周期计时统计成员变量
    std::chrono::high_resolution_clock::time_point cycleStartTime_;      ///< 当前周期开始时间点
    std::atomic<uint64_t> cycleProcessingTime_{0};                    ///< 当前周期处理时间（微秒）
    std::atomic<uint64_t> minProcessingTime_{UINT64_MAX};            ///< 最小处理时间（微秒）
    std::atomic<uint64_t> maxProcessingTime_{0};                     ///< 最大处理时间（微秒）
    std::atomic<uint64_t> totalProcessingTime_{0};                   ///< 累计处理时间（微秒）
    std::atomic<uint64_t> timingSampleCount_{0};                     ///< 计时样本计数
    std::atomic<uint32_t> timingReportCounter_{0};                  ///< 报告计数器（控制输出频率）

    // 计时统计配置
    static constexpr uint32_t TIMING_REPORT_INTERVAL = CYCLE_COUNT;         ///< 计时报告间隔（周期数）
    static constexpr uint64_t WARNING_THRESHOLD_US = 1500;         ///< 处理时间警告阈值（微秒）

    /**
     * @brief 更新计时统计信息
     *
     * @details 更新最小、最大、累计处理时间和样本计数
     *
     * @param processingTimeUs 当前周期处理时间（微秒）
     */
    inline void updateTimingStatistics(uint64_t processingTimeUs) {
        // 更新样本计数
        timingSampleCount_.fetch_add(1, std::memory_order_relaxed);

        // 更新累计处理时间
        totalProcessingTime_.fetch_add(processingTimeUs, std::memory_order_relaxed);

        // 更新最小处理时间
        uint64_t currentMin = minProcessingTime_.load(std::memory_order_relaxed);
        while (processingTimeUs < currentMin) {
            if (minProcessingTime_.compare_exchange_weak(currentMin, processingTimeUs,
                                                   std::memory_order_relaxed)) {
                break;
            }
        }

        // 更新最大处理时间
        uint64_t currentMax = maxProcessingTime_.load(std::memory_order_relaxed);
        while (processingTimeUs > currentMax) {
            if (maxProcessingTime_.compare_exchange_weak(currentMax, processingTimeUs,
                                                   std::memory_order_relaxed)) {
                break;
            }
        }
    }

    /**
     * @brief 输出计时统计信息
     *
     * @details 格式化输出计时统计信息，包括当前、最小、最大、平均处理时间
     */
    inline void outputTimingStatistics() {
        uint64_t currentTime = cycleProcessingTime_.load(std::memory_order_relaxed);
        uint64_t minTime = minProcessingTime_.load(std::memory_order_relaxed);
        uint64_t maxTime = maxProcessingTime_.load(std::memory_order_relaxed);
        uint64_t totalTime = totalProcessingTime_.load(std::memory_order_relaxed);
        uint64_t sampleCount = timingSampleCount_.load(std::memory_order_relaxed);

        if (sampleCount > 0) {
            uint64_t avgTime = totalTime / sampleCount;

            std::cout << "[TIMING][SendThread]: 周期#" << cycleCount_
                      << ": 当前=" << currentTime << "μs"
                      << ", 最小=" << minTime << "μs"
                      << ", 最大=" << maxTime << "μs"
                      << ", 平均=" << avgTime << "μs"
                      << ", 样本数=" << sampleCount << std::endl;
        }
    }
#endif
    
    // 错误状态管理
    enum class ThreadError {
        NONE = 0,
        COMMUNICATION_ERROR,
        BATCH_ERROR,
        SDO_TIMEOUT,
        HARDWARE_ERROR
    };
    
    std::atomic<ThreadError> lastError_{ThreadError::NONE};
    std::atomic<uint32_t> errorCount_{0};
    std::string lastErrorMessage_;
    

    /***********************************    主循环    ***********************************/
    /**
     * @brief 发送线程主循环
     * 
     * @details 重构后的主循环采用清晰的职责分离设计：
     * - 周期管理：负责2ms同步控制和性能监控
     * - 任务调度：协调SDO和PDO处理的优先级
     * - 错误处理：统一的错误恢复机制
     * 
     * 处理优先级：
     * 1. SDO帧（批次优先，单线程确保总线确定性）
     * 2. PDO帧（SDO处理完成后进行）
     * 
     */
    void threadLoop() {
        constexpr std::chrono::milliseconds SYNC_PERIOD{2};
        
        while (running_.load(std::memory_order_relaxed)) {
            auto cycleStart = std::chrono::steady_clock::now();

#ifdef ENABLE_CYCLE_TIMING
            // 记录周期开始时间（高精度计时）
            cycleStartTime_ = std::chrono::high_resolution_clock::now();
#endif

            // 周期监控和性能统计
            performCycleMonitoring(cycleStart);
            
            // 任务调度和处理
            scheduleAndProcessTasks();
            
            // 高精度周期控制：忙等待确保确定性
            waitForNextCycle(cycleStart, SYNC_PERIOD);
        }
    }
    

    /***********************************    函数模块    ***********************************/
    
    



    /**
     * @brief 周期监控和性能统计
     * 
     * @details 负责周期性能监控，每1000个周期输出一次统计信息
     * 采用柯理化设计，将监控逻辑从主循环中分离
     */
    inline void performCycleMonitoring(const std::chrono::steady_clock::time_point& cycleStart) {
        // 周期性能监控（每1000个周期输出一次）
        if (cycleCount_++ % 1000 == 0 && cycleCount_ > 1) {
            auto actualPeriod = cycleStart - lastCycleTime_;
            DEBUG_PRINT("[DEBUG][SendThread]: 周期统计: 实际周期="
                      << std::chrono::duration_cast<std::chrono::microseconds>(actualPeriod).count()
                      << "μs, 期望=" << 2000 << "μs");
        }
        lastCycleTime_ = cycleStart;

#ifdef ENABLE_CYCLE_TIMING
        // 周期计时统计（每TIMING_REPORT_INTERVAL个周期输出一次）
        timingReportCounter_.fetch_add(1, std::memory_order_relaxed);
        if (timingReportCounter_.load(std::memory_order_relaxed) >= TIMING_REPORT_INTERVAL) {
            outputTimingStatistics();
            timingReportCounter_.store(0, std::memory_order_relaxed);
        }
#endif
    }
    
    /**
     * @brief 任务调度和处理
     * 
     * @details 负责SDO和PDO处理的调度，采用一次性状态检查：
     * - 避免重复的原子操作和缓冲区检查
     * - 基于状态直接决策，消除边界情况
     * 
     */
    inline void scheduleAndProcessTasks() {
        // 1. 一次性获取处理状态（避免重复原子操作和缓冲区检查）
        const bool hasBatch = sdoBatchActive_.load(std::memory_order_relaxed);
        const bool hasSdoFrames = planBuffer_.getUsedSpace() >= CAN_FRAME_SIZE;
        const bool shouldProcessSdo = hasBatch || hasSdoFrames;   //处理SDO帧
        const bool shouldProcessPdo = !hasBatch && !hasSdoFrames; //处理PDO帧
        
        // 2. 优先处理SDO帧（配置和模式切换优先）
        if (shouldProcessSdo) {
            size_t sdoFramesProcessed = processSdoFrames();
            
            DEBUG_PRINT_IF(sdoFramesProcessed > 0, 
                          "[DEBUG][SendThread::scheduleAndProcessTasks]: "
                          "SDO处理完成，帧数: " << sdoFramesProcessed);
        }
        
        // 3. SDO处理完成后处理PDO帧
        if (shouldProcessPdo) {
            processPdoFrames();

            DEBUG_PRINT("[DEBUG][SendThread::scheduleAndProcessTasks]: PDO处理完成");
        }

        // 4. 发送同步帧（周期结束标志）
        sendSyncFrame();
    }

    /**
     * @brief 发送同步帧
     *
     * @details 在每个周期结束时发送预构建的CANopen SYNC帧（COB-ID: 0x0080）
     * 作为本轮通信的结束标志和下一轮通信的开始信号
     */
    inline void sendSyncFrame() {
        try {
            // 发送预构建的同步帧（避免重复构建开销）
            serialManager_.sendFrame(syncFrame_);

            DEBUG_PRINT("[DEBUG][SendThread::sendSyncFrame]: 同步帧已发送");

            // 更新全局帧计数器
            globalFrameCounter_.fetch_add(1, std::memory_order_relaxed);

        } catch (const std::exception& e) {
            // 同步帧发送失败不影响主流程，仅记录错误
            DEBUG_PRINT("[ERROR][SendThread::sendSyncFrame]: 同步帧发送失败: " << e.what());
        }
    }
    
    /**
     * @brief 高精度周期等待
     * 
     * @details 使用忙等待确保精确的2ms周期，满足实时性要求
     * 采用柯理化设计，将周期控制逻辑从主循环中分离
     * 
     * @param cycleStart 当前周期开始时间
     * @param syncPeriod 同步周期（2ms）
     */
    inline void waitForNextCycle(const std::chrono::steady_clock::time_point& cycleStart,
                                const std::chrono::milliseconds& syncPeriod) {
#ifdef ENABLE_CYCLE_TIMING
        // 计算并记录当前周期的处理时间
        auto cycleEndTime = std::chrono::high_resolution_clock::now();
        auto processingDuration = std::chrono::duration_cast<std::chrono::microseconds>(
            cycleEndTime - cycleStartTime_);
        uint64_t processingTimeUs = processingDuration.count();

        // 更新当前周期处理时间
        cycleProcessingTime_.store(processingTimeUs, std::memory_order_relaxed);

        // 更新统计信息
        updateTimingStatistics(processingTimeUs);

        // 检查是否超过警告阈值
        if (processingTimeUs > WARNING_THRESHOLD_US) {
            std::cout << "[WARNING][SendThread]: 周期处理时间过长: "
                      << processingTimeUs << "μs (阈值: " << WARNING_THRESHOLD_US << "μs)" << std::endl;
        }
#endif

        auto elapsed = std::chrono::steady_clock::now() - cycleStart;
        if (elapsed < syncPeriod) {
            auto target = cycleStart + syncPeriod;

            // 使用忙等待确保精确的2ms周期
            while (std::chrono::steady_clock::now() < target) {
                // 短暂暂停避免CPU占用100%
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
        }
    }
     

    /**
     * @brief SDO帧处理结果枚举
     * 
     * @details 定义SDO帧处理的各种可能结果，用于精确控制发送流程。
     * 采用枚举类确保类型安全，避免魔法数字。
     * 
     * 枚举值说明：
     * - SUCCESS: 帧处理成功，SDO事务已启动并成功发送
     * - RETRY_NEEDED: 需要重试，当前帧因状态机繁忙等原因需要延迟处理
     * - NO_FRAME: 无可用帧，缓冲区为空或数据不足
     * - SDO_ERROR: 处理错误，发生严重错误需要上层处理
     * 
     * @note 这些结果值直接影响发送线程的决策流程
     * @warning SDO_ERROR状态通常需要上层进行错误恢复处理
     */
    enum class SdoProcessResult {
        SUCCESS,        ///< 处理成功：SDO事务启动并发送完成
        RETRY_NEEDED,   ///< 需要重试：状态机繁忙或暂时不可用
        NO_FRAME,       ///< 无可用帧：缓冲区为空或数据不足
        SDO_ERROR           ///< 处理错误：发生严重错误需要恢复
    };
    


    /**
 * @brief 处理单帧SDO（字节级操作 + 柯理化设计）
 * 
 * @details 实现单个SDO帧的完整处理流程，从数据读取到串口发送。
 * 采用高性能设计和错误处理机制，确保实时性和可靠性。
 *
 * 核心设计特点：
 * - **零中间拷贝**：使用copyFrom直接在缓冲区间传输数据，避免中间缓冲区
 * - **原子性操作**：使用copyFrom确保数据传输的原子性
 * - **字节级处理**：直接操作字节数据，避免CanFrame对象创建开销
 * - **柯理化设计**：返回处理结果供上层决策，实现逻辑分离
 * - **错误恢复**：完善的异常处理和状态清理机制
 *
 * 处理流程：
 * 1. 使用peek预检查数据可用性（不消费）
 * 2. 通过prepareTransactionFromBytes准备SDO事务
 * 3. 启动事务并验证状态机状态
 * 4. 成功后直接复制数据到发送缓冲区并发送
 * 5. 异常情况下进行状态清理和错误报告
 * 
 * @return SdoProcessResult 处理结果：
 *         - SUCCESS: 帧处理成功，SDO事务已启动
 *         - RETRY_NEEDED: 状态机繁忙，需要延迟处理
 *         - NO_FRAME: 缓冲区无可用数据
 *         - SDO_ERROR: 处理过程中发生错误
 * 
 * @exception std::exception 处理过程中可能抛出各种异常
 * 
 * @note 性能优化点：
 * - 使用copyFrom实现缓冲区间零拷贝传输
 * - 避免任何动态内存分配和中间缓冲区
 * - 最小化函数调用开销和内存拷贝次数
 * 
 * @warning 重要约束：
 * - 必须确保缓冲区有完整的13字节CAN帧数据
 * - 事务失败时必须正确清理状态机
 * - 发送失败时需要记录错误信息
 * 
 * @see SdoProcessResult SDO状态机相关类
 */
    inline SdoProcessResult processSingleSdoFrame() {
        // 步骤1：数据预检查（使用peek确保原子性）
        // 采用peek操作查看数据但不消费，用于SDO事务准备
        auto frameData = planBuffer_.peek();

        DEBUG_PRINT("[DEBUG][processSingleSdoFrame]: 开始处理SDO帧");

        // 检查数据可用性：缓冲区为空或数据不足时返回NO_FRAME
        if (!frameData.has_value()) {
            DEBUG_PRINT("[DEBUG][processSingleSdoFrame]: 缓冲区无数据");
            return SdoProcessResult::NO_FRAME;
        }

        try {
            // 步骤2：SDO事务准备（字节级操作，避免对象创建）
            // 直接使用字节数据准备事务，避免CanFrame对象的构造开销
            auto transaction = sdoStateMachine_.prepareTransactionFromBytes(frameData->data());

            // 步骤3：事务启动（使用新的统一接口）
            // 使用新的createAndStartTransaction接口，符合重构后的状态机设计
            DEBUG_PRINT("[DEBUG][processSingleSdoFrame]: 尝试启动事务");
            if (!sdoStateMachine_.createAndStartTransaction(
                transaction.node_id,
                transaction.index,
                transaction.subindex,
                transaction.request_data)) {
                DEBUG_PRINT("[WARN][SendThread::processSingleSdoFrame]: 事务启动失败，状态机繁忙");
                return SdoProcessResult::RETRY_NEEDED;
            }
            DEBUG_PRINT("[DEBUG][processSingleSdoFrame]: 事务启动成功");

            // 步骤4：数据复制和发送（使用copyFrom直接传输到发送缓冲区）
            // 直接从planBuffer复制数据到sendBuffer，避免中间缓冲区，提高性能
            DEBUG_PRINT("[DEBUG][processSingleSdoFrame]: 开始数据复制，预期13字节");
            size_t bytesCopied = sendBuffer_.copyFrom(planBuffer_, CAN_FRAME_SIZE);

            DEBUG_PRINT("[DEBUG][processSingleSdoFrame]: 实际复制字节数: " << bytesCopied);

            // 验证数据完整性：确保成功复制完整的13字节CAN帧
            if (bytesCopied != CAN_FRAME_SIZE) {
                DEBUG_PRINT("[ERROR][SendThread::processSingleSdoFrame]: 数据复制失败，预期13字节，实际: " << bytesCopied);
                DEBUG_PRINT("[ERROR][SendThread::processSingleSdoFrame]: 可能原因：缓冲区数据不完整或空间不足");
                // 清理状态机，避免事务悬挂
                sdoStateMachine_.completeTransaction();
                return SdoProcessResult::SDO_ERROR;
            }

            // 步骤5：串口发送（使用同步发送接口）
            // 通过串口管理器发送数据，复用批量发送接口保证一致性
            DEBUG_PRINT("[DEBUG][processSingleSdoFrame]: 开始串口发送，预期13字节");
            size_t bytesSent = serialManager_.sendBufferSync(sendBuffer_, bytesCopied, false);

            // 验证发送完整性：确保所有字节成功发送
            if (bytesSent != bytesCopied) {
                DEBUG_PRINT("[ERROR][SendThread::processSingleSdoFrame]: SDO帧发送失败，预期" << bytesCopied << "字节，实际: " << bytesSent);
                DEBUG_PRINT("[ERROR][SendThread::processSingleSdoFrame]: 可能原因：串口未连接或发送缓冲区问题");
                // 发送失败时清理状态机，避免资源泄漏
                sdoStateMachine_.completeTransaction();
                return SdoProcessResult::SDO_ERROR;
            }

            DEBUG_PRINT("[DEBUG][processSingleSdoFrame]: 串口发送成功，发送字节数: " << bytesSent);

            // 成功完成SDO帧处理，记录调试信息
            DEBUG_PRINT("[DEBUG][SendThread::processSingleSdoFrame]: SDO帧发送成功，等待响应");

            // 简化状态检查（仅用于调试）
            auto currentState = sdoStateMachine_.getCurrentState();
            DEBUG_PRINT("[DEBUG][processSingleSdoFrame]: 发送完成后的状态机状态: " << static_cast<int>(currentState));

            // 返回成功状态，SDO事务已启动并成功发送
            return SdoProcessResult::SUCCESS;

        } catch (const std::exception& e) {
            // 异常处理：捕获所有可能的异常并返回错误状态
            DEBUG_PRINT("[ERROR][SendThread::processSingleSdoFrame]: 处理异常: " << e.what());
            return SdoProcessResult::SDO_ERROR;
        }
    }
    
    /**
     * @brief 处理SDO帧批次（集成超时重试和批次管理）
     * 
     * @details 批次化处理SDO帧，确保：
     * - 优先处理现有SDO事务的超时重试
     * - 按批次大小限制新SDO帧处理数量
     * - 处理完指定批次后转向PDO处理
     * - 批次期间产生的SDO帧计入下一批次
     * - 使用peek查看数据但不消费，确保事务性操作
     * 
     * @return 成功处理的SDO帧数量（包括重试和新帧）
     * @throws std::runtime_error 当SDO帧处理失败时
     * 
     * @note 符合柯理化设计原则，支持精确批次控制
     * @warning SDO处理必须在PDO之前完成，批次大小由规划线程设定
     */
    inline size_t processSdoFrames() {
        size_t totalProcessed = 0;
        
        // 1. 首先处理现有SDO事务的超时重试（必须优先处理）
        bool retryInProgress = false;
        try {
            // 检查当前SDO事务是否超时
            bool needsRetry = sdoStateMachine_.checkTimeout();
            
            if (needsRetry) {
                uint8_t retryCount = sdoStateMachine_.getRetryCount();
                DEBUG_PRINT("[DEBUG][SendThread::processSdoFrames]: SDO超时检测，当前重试次数: " 
                          << static_cast<int>(retryCount) << "/" << MAX_RETRY_COUNT);
            }
            
            // 如果需要重试且状态机允许重试，优先处理重试
            if (needsRetry && sdoStateMachine_.needsRetry()) {
                retryInProgress = true;
                DEBUG_PRINT("[DEBUG][SendThread::processSdoFrames]: SDO需要重试，将在下一周期重新发送");
                return 0; // 重试优先级最高，本周期不处理新帧
            }
            
        } catch (const std::runtime_error& e) {
            // 超过最大重试次数，清理状态并重新抛出异常
            DEBUG_PRINT("[ERROR][SendThread::processSdoFrames]: SDO通信失败，强制清理状态: " << e.what());

            resetBatchState();
            sdoStateMachine_.reset(); // 重置SDO状态机，清除事务
            throw; // 重新抛出异常，确保机械臂安全
            
        } catch (const std::exception& e) {
            // 其他异常，记录并重置状态机
            DEBUG_PRINT("[ERROR][SendThread::processSdoFrames]: 意外异常: " << e.what());
            
            resetBatchState();
            sdoStateMachine_.reset();
            return 0;
        }
        
        // 如果没有重试需求，开始处理新SDO帧
        if (retryInProgress) {
            return 0;
        }
        
        // 2. 简化批次状态管理
        uint8_t remainingInBatch = sdoBatchRemaining_.load(std::memory_order_relaxed);
        bool batchActive = sdoBatchActive_.load(std::memory_order_relaxed);
        
        // 3. 批次模式处理
        if (batchActive && remainingInBatch > 0) {
            // 批次模式：按剩余批次处理
            for (uint8_t i = 0; i < remainingInBatch; ++i) {
                if (planBuffer_.getUsedSpace() < CAN_FRAME_SIZE) {
                    // 缓冲区无足够数据，退出批次
                    break;
                }
                
                auto result = processSingleSdoFrame();
                
                switch (result) {
                    case SdoProcessResult::SUCCESS:
                        sdoBatchRemaining_.fetch_sub(1, std::memory_order_relaxed);
                        totalProcessed++;
                        break;
                    case SdoProcessResult::RETRY_NEEDED:
                        // 需要重试，暂停批次处理
                        return totalProcessed;
                    case SdoProcessResult::NO_FRAME:
                        // 无可用帧，退出批次
                        return totalProcessed;
                    case SdoProcessResult::SDO_ERROR:
                        // 处理错误，重置批次并通知上层
                        setLastError(ThreadError::BATCH_ERROR, "SDO帧处理失败");
                        sdoBatchRemaining_.store(0, std::memory_order_relaxed);
                        sdoBatchActive_.store(false, std::memory_order_relaxed);
                        return 0;
                }
            }
            
            // 检查批次是否自动完成
            if (sdoBatchRemaining_.load(std::memory_order_relaxed) == 0) {
                sdoBatchActive_.store(false, std::memory_order_relaxed);
            }
        } else {
            // 4. 非批次模式处理
            while (planBuffer_.getUsedSpace() >= CAN_FRAME_SIZE) {
                auto result = processSingleSdoFrame();
                
                switch (result) {
                    case SdoProcessResult::SUCCESS:
                        totalProcessed++;
                        break;
                    case SdoProcessResult::RETRY_NEEDED:
                        // 需要重试，跳出循环
                        return totalProcessed;
                    case SdoProcessResult::NO_FRAME:
                        // 无可用帧，处理完成
                        return totalProcessed;
                    case SdoProcessResult::SDO_ERROR:
                        // 处理错误，设置错误状态并返回
                        setLastError(ThreadError::SDO_TIMEOUT, "SDO帧处理失败");
                        return 0;
                }
            }
        }
        
        return totalProcessed;
    }
    
    /**
     * @brief 重置批次状态
     * 
     * @details 安全地重置批次相关状态，用于错误恢复
     */
    inline void resetBatchState() {
        sdoBatchRemaining_.store(0, std::memory_order_relaxed);
        sdoBatchActive_.store(false, std::memory_order_relaxed);
    }
    
    /**
     * @brief 设置错误状态
     * 
     * @details 安全地设置错误状态，供上层查询
     */
    inline void setLastError(ThreadError error, const std::string& message) {
        lastError_.store(error, std::memory_order_relaxed);
        errorCount_.fetch_add(1, std::memory_order_relaxed);
        lastErrorMessage_ = message;
        
        DEBUG_PRINT("[ERROR][SendThread]: " << message);
    }
    
    /**
     * @brief 处理PDO帧（集成PDO映射表处理）
     */
    void processPdoFrames() {
#ifndef DISABLE_PDO_PROCESSING
        size_t processedFrames = buildAndSendPdoFrames();
        
        DEBUG_PRINT_IF(processedFrames > 0, 
                      "[DEBUG][SendThread::processPdoFrames]: PDO处理完成，帧数: " << processedFrames);
#else
        // SDO测试模式：禁用PDO处理
        DEBUG_PRINT("[DEBUG][SendThread::processPdoFrames]: PDO处理已禁用（SDO测试模式）");
#endif
    }
    
    
public:
    /**
     * @brief 构造函数
     *
     * @param sendBuffer 发送环形缓冲区引用
     * @param planBuffer 规划环形缓冲区引用
     * @param motors 电机数组引用（固定大小6个）
     * @param motorCount 电机数量（必须为6）
     * @param serialManager 串口管理器引用
     * @param pdoMappingTable PDO映射表引用
     */
    SendThread(CircularBuffer& sendBuffer,
               CircularBuffer& planBuffer,
               std::array<Motor, 6>& motors,
               uint8_t motorCount,
               SerialPortManager& serialManager,
               const std::vector<PdoMappingEntry>& pdoMappingTable)
        : sendBuffer_(sendBuffer)
        , planBuffer_(planBuffer)
        , motors_(motors)
        , motorCount_(motorCount)
        , serialManager_(serialManager)
        , pdoMappingTable_(pdoMappingTable)
    {
        // 基础参数验证
        if (motorCount != MOTOR_ARRAY_SIZE) {
            throw std::invalid_argument("[ERROR][SendThread]: 电机数量必须为6");
        }

        // 预构建同步帧（避免重复构建开销）
        syncFrame_ = CanFrame(0x0080, nullptr, 0);

        // 简化的PDO数据结构初始化
        DEBUG_PRINT_IF(ENABLE_DEBUG_OUTPUT, "[DEBUG][SendThread]: 初始化PDO数据槽位");

        size_t slotIndex = 0;
        for (uint8_t motorIndex = 1; motorIndex <= motorCount_; ++motorIndex) {
            for (uint8_t pdoIndex = 1; pdoIndex <= PDO_CHANNELS_PER_MOTOR; ++pdoIndex) {
                uint32_t cobId = toRpdoCobId(motorIndex, pdoIndex);
                pdoDataMap_[slotIndex].cobId = cobId;
                pdoDataMap_[slotIndex].data.fill(0);
                pdoDataMap_[slotIndex].isValid = true;
                pdoDataMap_[slotIndex].hasValidData = false;
                slotIndex++;
            }
        }

#ifdef ENABLE_CYCLE_TIMING
        // 初始化计时统计成员变量
        cycleStartTime_ = std::chrono::high_resolution_clock::time_point{};
        cycleProcessingTime_.store(0, std::memory_order_relaxed);
        minProcessingTime_.store(UINT64_MAX, std::memory_order_relaxed);
        maxProcessingTime_.store(0, std::memory_order_relaxed);
        totalProcessingTime_.store(0, std::memory_order_relaxed);
        timingSampleCount_.store(0, std::memory_order_relaxed);
        timingReportCounter_.store(0, std::memory_order_relaxed);

        DEBUG_PRINT("[DEBUG][SendThread]: 计时统计功能已启用，报告间隔: "
                  << TIMING_REPORT_INTERVAL << " 周期");
#endif
    }
    
    /**
     * @brief 析构函数
     */
    ~SendThread() {
        stop();
    }
    
    /**
     * @brief 启动发送线程
     */
    void start() {
        if (!running_.exchange(true, std::memory_order_relaxed)) {
            // 线程启动前验证资源状态
            // std::array总是已初始化，无需空检查
            
            sendThread_ = std::thread(&SendThread::threadLoop, this);
            
            DEBUG_PRINT("[INFO][SendThread::start]: 线程已启动");
        }
    }
    
    /**
     * @brief 停止发送线程
     */
    void stop() {
        if (running_.exchange(false, std::memory_order_relaxed)) {
            if (sendThread_.joinable()) {
                sendThread_.join();
            }
            
            DEBUG_PRINT("[INFO][SendThread::stop]: 线程已停止");
        }
    }
    
    /**
     * @brief 检查线程是否运行
     * @return 运行状态
     */
    bool isRunning() const {
        return running_.load(std::memory_order_relaxed);
    }
    
    /**
     * @brief 设置SDO批次大小（供规划线程调用）
     * 
     * @details 用于设定当前周期需要处理的SDO帧数量。
     * 采用原子操作实现线程安全的批次控制。
     * 
     * @param batchSize 批次大小（0表示无批次限制）
     * 
     * @note 使用原子操作确保线程安全
     * @warning 批次大小限制为0-255，超出会被截断
     */
    void setSdoBatchSize(uint8_t batchSize) {
        if (batchSize > 0) {
            sdoBatchRemaining_.store(batchSize, std::memory_order_relaxed);
            sdoBatchActive_.store(true, std::memory_order_relaxed);
        } else {
            // 批次大小为0时停用批次模式
            sdoBatchActive_.store(false, std::memory_order_relaxed);
        }
        
        DEBUG_PRINT("[DEBUG][SendThread::setSdoBatchSize]: 批次设置，大小: " << static_cast<int>(batchSize));
    }
    
    /**
     * @brief 获取当前SDO批次状态
     * @return 批次信息结构体，包含剩余帧数、是否正在进行批次处理
     */
    struct SdoBatchStatus {
        uint8_t remainingFrames;   ///< 剩余待处理帧数
        bool isInProgress;         ///< 是否正在进行批次处理
        
        SdoBatchStatus(uint8_t remaining, bool progress)
            : remainingFrames(remaining), isInProgress(progress) {}
    };
    
    SdoBatchStatus getSdoBatchStatus() const {
        uint8_t remaining = sdoBatchRemaining_.load(std::memory_order_relaxed);
        bool inProgress = sdoBatchActive_.load(std::memory_order_relaxed);
        
        return SdoBatchStatus(remaining, inProgress);
    }
     
    /**
     * @brief 获取SDO状态机引用
     * @return SDO状态机引用
     */
    canopen::AtomicSdoStateMachine& getSdoStateMachine() {
        return sdoStateMachine_;
    }
    
    /**
     * @brief 获取最后一个错误
     * @return 错误类型
     */
    ThreadError getLastError() const {
        return lastError_.load(std::memory_order_relaxed);
    }
    
    /**
     * @brief 获取错误计数
     * @return 错误发生次数
     */
    uint32_t getErrorCount() const {
        return errorCount_.load(std::memory_order_relaxed);
    }

    /**
     * @brief 获取全局帧计数器
     * @return 已发送的帧总数
     */
    uint64_t getGlobalFrameCounter() const {
        return globalFrameCounter_.load(std::memory_order_relaxed);
    }
    
    /**
     * @brief 获取错误消息
     * @return 错误描述
     */
    std::string getLastErrorMessage() const {
        return lastErrorMessage_;
    }
    
    /**
     * @brief 清除错误状态
     */
    void clearError() {
        lastError_.store(ThreadError::NONE, std::memory_order_relaxed);
        lastErrorMessage_.clear();
    }

#ifdef ENABLE_CYCLE_TIMING
    /**
     * @brief 获取性能统计信息
     * @return 统计信息结构体
     */
    struct PerformanceStats {
        uint64_t currentCycleTime;      ///< 当前周期处理时间（微秒）
        uint64_t minProcessingTime;    ///< 最小处理时间（微秒）
        uint64_t maxProcessingTime;    ///< 最大处理时间（微秒）
        uint64_t avgProcessingTime;    ///< 平均处理时间（微秒）
        uint64_t totalSamples;         ///< 总样本数
        uint64_t cycleCount;           ///< 周期计数
    };

    /**
     * @brief 获取当前性能统计信息
     * @return PerformanceStats 性能统计信息
     */
    PerformanceStats getPerformanceStats() const {
        uint64_t current = cycleProcessingTime_.load(std::memory_order_relaxed);
        uint64_t minTime = minProcessingTime_.load(std::memory_order_relaxed);
        uint64_t maxTime = maxProcessingTime_.load(std::memory_order_relaxed);
        uint64_t totalTime = totalProcessingTime_.load(std::memory_order_relaxed);
        uint64_t sampleCount = timingSampleCount_.load(std::memory_order_relaxed);

        uint64_t avgTime = (sampleCount > 0) ? (totalTime / sampleCount) : 0;

        return PerformanceStats{
            current,
            (minTime == UINT64_MAX) ? 0 : minTime,
            maxTime,
            avgTime,
            sampleCount,
            cycleCount_
        };
    }

    /**
     * @brief 重置性能统计信息
     */
    void resetPerformanceStats() {
        cycleProcessingTime_.store(0, std::memory_order_relaxed);
        minProcessingTime_.store(UINT64_MAX, std::memory_order_relaxed);
        maxProcessingTime_.store(0, std::memory_order_relaxed);
        totalProcessingTime_.store(0, std::memory_order_relaxed);
        timingSampleCount_.store(0, std::memory_order_relaxed);
        timingReportCounter_.store(0, std::memory_order_relaxed);

        DEBUG_PRINT("[DEBUG][SendThread]: 性能统计已重置");
    }

    /**
     * @brief 检查处理时间是否在正常范围内
     * @return true表示处理时间正常，false表示超时或异常
     */
    bool isProcessingTimeNormal() const {
        uint64_t currentTime = cycleProcessingTime_.load(std::memory_order_relaxed);
        return currentTime <= WARNING_THRESHOLD_US;
    }
#endif
    
    /**
     * @brief 检查线程健康状态
     * @return true表示线程健康，false表示有错误
     */
    bool isHealthy() const {
        return lastError_.load(std::memory_order_relaxed) == ThreadError::NONE;
    }
    
    // ====================== PDO处理函数实现 ======================

    /**
     * @brief 直接计算PDO数据槽位索引 (O(1)无查找开销)
     *
     * @details 根据电机索引和PDO索引直接计算槽位位置，完全消除查找开销。
     * 在固定电机数量配置下，这是最高效的访问方式。
     *
     * @param motorIndex 电机索引 (1-6)
     * @param pdoIndex PDO索引 (1-2)
     * @return size_t 槽位索引
     *
     * @note 此函数假设电机数量固定为6，PDO通道固定为2
     * @warning 仅在已知有效范围内调用，不进行边界检查
     */
    inline size_t calculatePdoSlotIndex(uint8_t motorIndex, uint8_t pdoIndex) {
        return (motorIndex - 1) * PDO_CHANNELS_PER_MOTOR + (pdoIndex - 1);
    }

    /**
     * @brief 批量清零特定电机的PDO槽位
     *
     * @details 直接计算并清零指定电机的所有PDO槽位，消除查找开销。
     *
     * @param motorIndex 电机索引 (1-6)
     */
    inline void clearMotorPdoSlots(uint8_t motorIndex) {
        size_t baseIndex = (motorIndex - 1) * PDO_CHANNELS_PER_MOTOR;
        for (size_t i = 0; i < PDO_CHANNELS_PER_MOTOR; ++i) {
            size_t slotIndex = baseIndex + i;
            pdoDataMap_[slotIndex].data.fill(0);
            pdoDataMap_[slotIndex].hasValidData = false;
        }
    }

    /**
     * @brief 验证PDO数据完整性
     *
     * @details 简化版本：仅在调试模式下验证数据一致性
     * 生产环境中此函数为空实现，消除运行时开销
     *
     * @return bool 始终返回true（生产模式）
     *
     * @note 该函数仅用于调试，在发布版本中被优化为空操作
     */
    inline bool validatePdoDataIntegrity() {
#if ENABLE_DEBUG_OUTPUT
        bool hasErrors = false;
        std::unordered_set<uint32_t> foundCobIds;

        for (size_t i = 0; i < MAX_PDO_SLOTS; ++i) {
            if (!pdoDataMap_[i].isValid) continue;

            // 仅在调试时检查重复COB-ID
            if (foundCobIds.find(pdoDataMap_[i].cobId) != foundCobIds.end()) {
                DEBUG_PRINT("[ERROR][SendThread::validatePdoDataIntegrity]: 发现重复COB-ID: 0x"
                          << std::hex << pdoDataMap_[i].cobId << std::dec);
                hasErrors = true;
            }
            foundCobIds.insert(pdoDataMap_[i].cobId);
        }

        return !hasErrors;
#else
        return true; // 生产模式直接返回成功
#endif
    }

    /**
     * @brief 基于PDO映射表构建并发送CAN帧的实现
     *
     * 优化要点：
     * 1. 按需清零：只清零实际使用的PDO槽位，避免无效内存操作
     * 2. 单次扫描：合并数据检查和帧构建，消除重复扫描
     * 3. 栈分配：使用固定大小数组，完全消除堆分配风险
     * 4. 基于标志位：使用hasValidData标志而非数据内容决定发送
     */
    inline size_t buildAndSendPdoFrames() {
        size_t totalFramesProcessed = 0;

        // 步骤1：直接清零所有PDO槽位 - 消除查找开销
        for (uint8_t motorIndex = 1; motorIndex <= motorCount_; ++motorIndex) {
            clearMotorPdoSlots(motorIndex);
        }
        
        // 步骤2：简化的数据准备逻辑 - 直接映射电机数据到PDO槽位
        for (const auto& mapping : pdoMappingTable_) {
            // 只处理RPDO（发送帧）
            if (!mapping.isTx) {
                // 映射表由buildArmMappingTable()生成，索引早已合法
                // 获取对应电机引用
                const Motor& motor = motors_[mapping.motorIndex - 1];

                // 直接计算槽位索引，消除查找开销
                size_t slotIndex = calculatePdoSlotIndex(mapping.motorIndex, mapping.pdoIndex);
                PdoDataSlot* targetSlot = &pdoDataMap_[slotIndex];

                // 从电机读取数据到PDO缓冲区
                if (!readMotorDataToPdoBuffer(motor, mapping, targetSlot->data.data())) {
                    DEBUG_PRINT_IF(ENABLE_DEBUG_OUTPUT, "[WARN][SendThread::buildAndSendPdoFrames]: 数据读取失败，电机="
                              << static_cast<int>(mapping.motorIndex) << ", 使用默认数据");

                    // 简化的默认数据设置 - 直接内存填充
                    std::memset(&targetSlot->data[mapping.offsetInPdo], 0, mapping.size);

                    // 控制字特殊处理
                    if (mapping.index == OD_CONTROL_WORD) {
                        targetSlot->data[mapping.offsetInPdo] = 0x00;
                        targetSlot->data[mapping.offsetInPdo + 1] = 0x00;
                    }
                }

                // 标记该槽位有有效数据需要发送
                targetSlot->hasValidData = true;

                DEBUG_PRINT_IF(ENABLE_DEBUG_OUTPUT, "[DEBUG][SendThread::buildAndSendPdoFrames]: 电机="
                             << static_cast<int>(mapping.motorIndex) << " 数据就绪");
            }
        }
        
        // 步骤3：简化的帧构建逻辑 - 直接遍历已知有效的槽位
        validFrameCount_ = 0;  // 重置有效帧计数

        // 直接遍历已知的有效槽位范围，避免无效槽位检查
        for (size_t i = 0; i < motorCount_ * PDO_CHANNELS_PER_MOTOR; ++i) {
            if (pdoDataMap_[i].hasValidData) {
                // 直接在数组中构建CAN帧（零堆分配）
                CanFrame& frame = framesToSend_[validFrameCount_];
                frame = CanFrame(pdoDataMap_[i].cobId, pdoDataMap_[i].data.data(), 8);

                globalFrameCounter_.fetch_add(1, std::memory_order_relaxed);
                DEBUG_PRINT_IF(ENABLE_DEBUG_OUTPUT, "[DEBUG][SendThread::buildAndSendPdoFrames]: 构建帧"
                          << static_cast<int>(validFrameCount_));

                validFrameCount_++;
            }
        }
        
        // 步骤4：简化数据完整性验证（仅调试模式）
        #if ENABLE_DEBUG_OUTPUT
        validatePdoDataIntegrity(); // 仅在调试时验证
        #endif

        // 步骤5：直接批量发送CAN帧
        if (validFrameCount_ > 0) {
            DEBUG_PRINT_IF(ENABLE_DEBUG_OUTPUT, "[DEBUG][SendThread::buildAndSendPdoFrames]: 发送"
                      << static_cast<int>(validFrameCount_) << "帧");

            try {
                // 直接使用预分配数组发送，避免vector拷贝开销
                size_t sentFrames = serialManager_.sendFramesBatch(
                    {framesToSend_.data(), framesToSend_.data() + validFrameCount_});

                totalFramesProcessed = sentFrames;
            } catch (const std::exception& e) {
                setLastError(ThreadError::COMMUNICATION_ERROR,
                           std::string("PDO批量发送失败: ") + e.what());
                return 0;
            }
        }

        return totalFramesProcessed;
    }
    
    /**
     * @brief 从电机类读取数据到PDO数据缓冲区的实现
     * 
     * @details 该函数根据PDO映射表条目从电机对象中读取指定的数据，并将其复制到PDO数据缓冲区的正确位置。
     *          支持多种类型的数据读取，包括位置、速度、电流和控制字等。该函数采用类型安全的方式
     *          处理不同数据类型的原子操作，确保多线程环境下的数据一致性。
     * 
     * @param motor 电机对象的常量引用，包含要读取的数据
     * @param mapping PDO映射表条目，定义了数据源和目标位置
     * @param dataBuffer 指向PDO数据缓冲区的指针，用于存储读取的数据
     * 
     * @return bool 成功返回true，失败返回false
     * 
     * @note 
     * - 函数内部进行参数有效性检查，包括空指针检查和偏移量边界检查
     * - 使用原子操作读取电机数据，确保线程安全
     * - 支持的数据类型包括目标位置、控制字、目标速度（两种模式）、目标电流
     * - 异常处理机制确保程序稳定性
     * 
     * @warning 
     * - 调用者需确保dataBuffer有足够的空间（至少offsetInPdo + mapping.size字节）
     * - mapping参数必须包含有效的对象字典索引和偏移量信息
     * 
     * @par 支持的对象字典索引：
     * - OD_TARGET_POSITION: 目标位置数据（32位整数）
     * - OD_CONTROL_WORD: 控制字数据（16位整数）
     * - OD_TARGET_VELOCITY_VELOCITY_MODE: 目标速度（速度模式）
     * - OD_TARGET_CURRENT: 目标电流数据
     * - OD_TARGET_VELOCITY_POSITION_MODE: 目标速度（位置模式）
     * 
     * @par 错误处理：
     * - 空指针检查：dataBuffer不能为nullptr
     * - 偏移量检查：offsetInPdo + dataSize不能超过8字节（PDO最大长度）
     * - 异常捕获：任何数据读取异常都会被捕获并记录
     * 
     * @see PdoMappingEntry Motor AlignedRawData
     */
    inline bool readMotorDataToPdoBuffer(const Motor& motor, 
                                        const PdoMappingEntry& mapping, 
                                        uint8_t* dataBuffer) {
        if (!dataBuffer) {
            return false;
        }
        
        try {
            // 根据对象字典索引和字段偏移量确定数据源
            uint16_t odIndex = mapping.index;
            size_t fieldOffset = mapping.motorFieldOffset;
            uint8_t dataSize = mapping.size;
            uint8_t offsetInPdo = mapping.offsetInPdo;
            
            // 确保偏移量有效
            if (offsetInPdo + dataSize > 8) {
                DEBUG_PRINT("[ERROR][SendThread::readMotorDataToPdoBuffer]: PDO偏移量越界，偏移: " 
                          << static_cast<int>(offsetInPdo) << ", 大小: " << static_cast<int>(dataSize));
                return false;
            }
            
            // 根据对象字典索引确定数据源和读取方式
            if (odIndex == OD_TARGET_POSITION) {
                // 目标位置数据
                Position_type value = motor.position.raw_target.atomicReadValue();
                std::memcpy(dataBuffer + offsetInPdo, &value, dataSize);
            }
            else if (odIndex == OD_CONTROL_WORD) {
                // 控制字数据
                uint16_t value;
                std::memcpy(&value, const_cast<const void*>(static_cast<const volatile void*>(motor.stateAndMode.controlData.controlWordRaw)), 2);
                std::memcpy(dataBuffer + offsetInPdo, &value, dataSize);
            }
            else if (odIndex == OD_TARGET_VELOCITY_VELOCITY_MODE) {
                // 目标速度数据（速度模式）
                Velocity_type value = motor.velocity.raw_target_velocity_mode.atomicReadValue();
                std::memcpy(dataBuffer + offsetInPdo, &value, dataSize);
            }
            else if (odIndex == OD_TARGET_CURRENT) {
                // 目标电流数据
                Current_type value = motor.current.raw_target.atomicReadValue();
                std::memcpy(dataBuffer + offsetInPdo, &value, dataSize);
            }
            else if (odIndex == OD_TARGET_VELOCITY_POSITION_MODE) {
                // 目标速度数据（位置模式）
                Velocity_type value = motor.velocity.raw_target_position_mode.atomicReadValue();
                std::memcpy(dataBuffer + offsetInPdo, &value, dataSize);
            }
            else {
                DEBUG_PRINT("[WARN][SendThread::readMotorDataToPdoBuffer]: 未知的对象字典索引: 0x" 
                          << std::hex << odIndex << std::dec);
                return false;
            }
            
            return true;
            
        } catch (const std::exception& e) {
            DEBUG_PRINT("[ERROR][SendThread::readMotorDataToPdoBuffer]: 读取数据异常: " << e.what());
            return false;
        }
    }
};

#undef ENABLE_DEBUG_OUTPUT

#endif // SEND_THREAD_HPP