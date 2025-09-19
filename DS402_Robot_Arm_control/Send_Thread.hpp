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
#define ENABLE_DEBUG_OUTPUT true


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
    static constexpr size_t PDO_CHANNELS_PER_MOTOR = 2;
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

        // motorCount_ 已在初始化列表中设置，无需重复赋值
        // motorCount_ = motorCount;  // 注释掉重复赋值
        
        // 预分配PDO数据结构 - 填充所有可能的COB-ID槽位

        // 预填充所有可能的COB-ID槽位（零分配运行时）
        // 修复：确保完整的初始化，避免未定义行为
        DEBUG_PRINT("[DEBUG][SendThread]: 初始化PDO数据映射表，最大槽位数: " << static_cast<int>(MAX_PDO_SLOTS));
        for (size_t i = 0; i < MAX_PDO_SLOTS; ++i) {
            pdoDataMap_[i].cobId = 0;
            pdoDataMap_[i].data.fill(0);
            pdoDataMap_[i].isValid = false;
            pdoDataMap_[i].hasValidData = false;  // 初始化新增字段

            // 添加初始化验证
            if (i == 0) {
                DEBUG_PRINT("[DEBUG][SendThread]: 槽位[0]初始化完成: COB-ID=0x0, isValid=false, hasValidData=false");
            }
        }
        
        // 预填充当前电机配置的COB-ID
        size_t slotIndex = 0;
        DEBUG_PRINT("[DEBUG][SendThread]: 开始初始化PDO数据槽位，电机数量: " << static_cast<int>(motorCount_));
        for (uint8_t motorIndex = 1; motorIndex <= motorCount_; ++motorIndex) {
            for (uint8_t pdoIndex = 1; pdoIndex <= PDO_CHANNELS_PER_MOTOR; ++pdoIndex) {
                uint32_t cobId = toRpdoCobId(motorIndex, pdoIndex);
                pdoDataMap_[slotIndex].cobId = cobId;
                pdoDataMap_[slotIndex].data.fill(0);
                pdoDataMap_[slotIndex].isValid = true;
                pdoDataMap_[slotIndex].hasValidData = false;  // 初始化新增字段

                // 添加COB-ID初始化验证调试输出
                DEBUG_PRINT("[DEBUG][SendThread]: 初始化槽位[" << static_cast<int>(slotIndex)
                          << "]: 电机" << static_cast<int>(motorIndex)
                          << ", PDO" << static_cast<int>(pdoIndex)
                          << ", COB-ID=0x" << std::hex << cobId << std::dec);
                slotIndex++;
            }
        }

        // 添加COB-ID初始化完整性验证
        DEBUG_PRINT("[DEBUG][SendThread]: PDO数据槽位初始化完成，验证COB-ID范围:");
        for (size_t i = 0; i < MAX_PDO_SLOTS; ++i) {
            if (pdoDataMap_[i].isValid) {
                uint32_t cobId = pdoDataMap_[i].cobId;
                DEBUG_PRINT("[DEBUG][SendThread]: 槽位[" << static_cast<int>(i)
                          << "] COB-ID=0x" << std::hex << cobId << std::dec);

                // 验证COB-ID是否在预期范围内
                if (cobId < 0x200 || cobId > 0x506) {
                    DEBUG_PRINT("[ERROR][SendThread]: 检测到非法COB-ID: 0x" << std::hex << cobId
                              << " 在槽位[" << static_cast<int>(i) << "] - 应在0x200-0x506范围内" << std::dec);
                }
            }
        }

        DEBUG_PRINT("[INFO][SendThread]: 构造函数完成，电机数量: " << static_cast<int>(motorCount_));
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
    
    /**
     * @brief 检查线程健康状态
     * @return true表示线程健康，false表示有错误
     */
    bool isHealthy() const {
        return lastError_.load(std::memory_order_relaxed) == ThreadError::NONE;
    }
    
    // ====================== PDO处理函数实现 ======================

    /**
     * @brief 高效查找PDO数据槽位
     *
     * @details 根据COB-ID快速查找对应的PDO数据槽位，采用线性查找算法。
     * 由于MAX_PDO_SLOTS较小（通常为24），线性查找具有更好的缓存性能。
     *
     * @param cobId 要查找的COB-ID
     * @return PdoDataSlot* 找到的槽位指针，未找到返回nullptr
     *
     * @note 该函数替代了原本内嵌在数据填充逻辑中的查找代码
     * @warning 返回指针的生命周期与pdoDataMap_相同，调用者需注意使用时效性
     */
    inline PdoDataSlot* findPdoSlot(uint32_t cobId) {
        for (size_t i = 0; i < MAX_PDO_SLOTS; ++i) {
            if (pdoDataMap_[i].isValid && pdoDataMap_[i].cobId == cobId) {
                return &pdoDataMap_[i];
            }
        }
        return nullptr;
    }

    /**
     * @brief 验证PDO数据完整性
     *
     * @details 检查PDO数据的一致性，包括：
     * - 有数据但标志位未设置的情况
     * - 标志位设置但数据全为零的情况
     * - 无效COB-ID或重复COB-ID的情况
     *
     * @return bool 验证成功返回true，发现问题返回false
     *
     * @note 该函数主要用于调试和诊断，可在发送前调用确保数据正确性
     * @warning 验证失败时会产生调试输出，但不会阻止PDO发送
     */
    inline bool validatePdoDataIntegrity() {
        bool hasErrors = false;
        std::unordered_set<uint32_t> foundCobIds;

        for (size_t i = 0; i < MAX_PDO_SLOTS; ++i) {
            if (!pdoDataMap_[i].isValid) continue;

            // 检查重复COB-ID
            if (foundCobIds.find(pdoDataMap_[i].cobId) != foundCobIds.end()) {
                DEBUG_PRINT("[ERROR][SendThread::validatePdoDataIntegrity]: 发现重复COB-ID: 0x"
                          << std::hex << pdoDataMap_[i].cobId << std::dec);
                hasErrors = true;
            }
            foundCobIds.insert(pdoDataMap_[i].cobId);

            // 验证数据一致性
            bool allZero = true;
            for (uint8_t byte : pdoDataMap_[i].data) {
                if (byte != 0) {
                    allZero = false;
                    break;
                }
            }

            // 检查逻辑1：有数据但标志位未设置
            if (!allZero && !pdoDataMap_[i].hasValidData) {
                DEBUG_PRINT("[WARN][SendThread::validatePdoDataIntegrity]: 检测到未标记的有效数据，COB-ID=0x"
                          << std::hex << pdoDataMap_[i].cobId << std::dec);
                // 这不是错误，但值得关注
            }

            // 检查逻辑2：标志位设置但数据全为零
            if (allZero && pdoDataMap_[i].hasValidData) {
                DEBUG_PRINT("[INFO][SendThread::validatePdoDataIntegrity]: 标记有效但数据全零，COB-ID=0x"
                          << std::hex << pdoDataMap_[i].cobId << std::dec);
                // 这是正常情况，比如目标位置为0
            }

            DEBUG_PRINT_IF(ENABLE_DEBUG_OUTPUT,
                         "[DEBUG][SendThread::validatePdoDataIntegrity]: 验证COB-ID=0x"
                         << std::hex << pdoDataMap_[i].cobId
                         << ", 有效=" << pdoDataMap_[i].hasValidData
                         << ", 全零=" << allZero << std::dec);
        }

        return !hasErrors;
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

        // 步骤1：修复后的清零逻辑 - 基于COB-ID一次性清零，避免重复清零
        std::unordered_set<uint32_t> processedCobIds;
        for (const auto& mapping : pdoMappingTable_) {
            if (!mapping.isTx) {  // 只处理RPDO
                uint32_t cobId = toRpdoCobId(mapping.motorIndex, mapping.pdoIndex);

                // 只清零每个COB-ID一次，避免重复清零同一PDO帧
                if (processedCobIds.find(cobId) == processedCobIds.end()) {
                    processedCobIds.insert(cobId);

                    // 查找并清零对应的槽位
                    for (size_t i = 0; i < MAX_PDO_SLOTS; ++i) {
                        if (pdoDataMap_[i].isValid && pdoDataMap_[i].cobId == cobId) {
                            pdoDataMap_[i].data.fill(0);
                            pdoDataMap_[i].hasValidData = false;  // 重置有效数据标志
                            break;
                        }
                    }
                }
            }
        }
        
        // 步骤2：修复后的数据准备逻辑 - 填充PDO数据并增加完整性检查
        for (const auto& mapping : pdoMappingTable_) {
            // 只处理RPDO（发送帧）
            if (!mapping.isTx) {
                // 映射表由buildArmMappingTable()生成，索引早已合法
                // 获取对应电机引用
                const Motor& motor = motors_[mapping.motorIndex - 1];

                // 生成COB-ID
                uint32_t cobId = toRpdoCobId(mapping.motorIndex, mapping.pdoIndex);

                // 使用高效查找函数查找预分配的槽位
                PdoDataSlot* targetSlot = findPdoSlot(cobId);

                if (!targetSlot) {
                    DEBUG_PRINT("[ERROR][SendThread::buildAndSendPdoFrames]: 找不到COB-ID对应的槽位: 0x"
                              << std::hex << cobId << std::dec);
                    continue;  // 找不到对应的槽位（理论上不应该发生）
                }

                // 从电机读取数据到PDO缓冲区
                if (!readMotorDataToPdoBuffer(motor, mapping, targetSlot->data.data())) {
                    DEBUG_PRINT("[WARN][SendThread::buildAndSendPdoFrames]: 数据读取失败，电机="
                              << static_cast<int>(mapping.motorIndex)
                              << ", 映射=0x" << std::hex << mapping.index << std::dec
                              << " - 使用默认数据");

                    // 数据读取失败时使用默认数据而不是跳过
                    // 根据映射类型设置不同的默认数据
                    switch (mapping.index) {
                        case OD_TARGET_POSITION:
                            // 目标位置默认为0
                            targetSlot->data[mapping.offsetInPdo] = 0;
                            targetSlot->data[mapping.offsetInPdo + 1] = 0;
                            targetSlot->data[mapping.offsetInPdo + 2] = 0;
                            targetSlot->data[mapping.offsetInPdo + 3] = 0;
                            break;
                        case OD_CONTROL_WORD:
                            // 控制字默认为停止状态
                            targetSlot->data[mapping.offsetInPdo] = 0x00;
                            targetSlot->data[mapping.offsetInPdo + 1] = 0x00;
                            break;
                        case OD_TARGET_VELOCITY_VELOCITY_MODE:
                        case OD_TARGET_VELOCITY_POSITION_MODE:
                            // 目标速度默认为0
                            targetSlot->data[mapping.offsetInPdo] = 0;
                            targetSlot->data[mapping.offsetInPdo + 1] = 0;
                            break;
                        case OD_TARGET_CURRENT:
                            // 目标电流默认为0
                            targetSlot->data[mapping.offsetInPdo] = 0;
                            targetSlot->data[mapping.offsetInPdo + 1] = 0;
                            break;
                        default:
                            // 未知映射类型，填充0
                            for (uint8_t j = 0; j < mapping.size; ++j) {
                                targetSlot->data[mapping.offsetInPdo + j] = 0;
                            }
                            break;
                    }

                    DEBUG_PRINT("[DEBUG][SendThread::buildAndSendPdoFrames]: 已设置默认数据，COB-ID=0x"
                              << std::hex << cobId << std::dec);
                }

                // 无论数据读取是否成功，都标记该槽位有有效数据需要发送
                // 这样可以确保即使读取失败，也会发送默认数据而不是空帧
                targetSlot->hasValidData = true;

                DEBUG_PRINT_IF(ENABLE_DEBUG_OUTPUT,
                             "[DEBUG][SendThread::buildAndSendPdoFrames]: 数据填充成功，电机="
                             << static_cast<int>(mapping.motorIndex)
                             << ", COB-ID=0x" << std::hex << cobId << std::dec);
            }
        }
        
        // 步骤3：修复后的帧构建逻辑 - 基于标志位而非数据内容决定发送
        validFrameCount_ = 0;  // 重置有效帧计数

        for (size_t i = 0; i < MAX_PDO_SLOTS; ++i) {
            if (!pdoDataMap_[i].isValid) {
                continue;
            }

            // 修复：基于hasValidData标志位决定是否发送，而不是检查数据是否为零
            // 这样可以确保即使数据全零（比如目标位置为0），只要标志位设置就会发送
            if (pdoDataMap_[i].hasValidData) {
                // 直接在数组中构建CAN帧（零堆分配）
                CanFrame& frame = framesToSend_[validFrameCount_];

                // 使用CAN_frame构造函数直接创建帧 - 简洁高效的设计
                uint32_t cobId = pdoDataMap_[i].cobId;

                // 验证COB-ID范围
                if (cobId < 0x200 || cobId > 0x506) {
                    DEBUG_PRINT("[ERROR][SendThread::buildAndSendPdoFrames]: 检测到非法COB-ID: 0x"
                              << std::hex << cobId << "，跳过此帧" << std::dec);
                    continue;
                }

                // 直接使用构造函数创建CAN帧 - 自动处理所有内部逻辑
                // 使用默认参数：标准帧、数据帧、普通CAN帧、无速率切换
                frame = CanFrame(cobId, pdoDataMap_[i].data.data(), 8);

                // 添加帧构建后的验证
                uint64_t currentGlobalFrame = globalFrameCounter_.fetch_add(1, std::memory_order_relaxed);
                DEBUG_PRINT("[DEBUG][SendThread::buildAndSendPdoFrames]: 构建CAN帧[全局第"
                          << currentGlobalFrame + 1
                          << "帧/当前批次第" << static_cast<int>(validFrameCount_)
                          << "帧]: COB-ID=0x" << std::hex << frame.frameID
                          << ", FrameInfo=0x" << static_cast<int>(frame.frameInfo)
                          << ", DLC=" << std::dec << static_cast<int>(frame.dlc)
                          << ", 数据: 0x" << std::hex
                          << static_cast<int>(frame.data[0]) << " "
                          << static_cast<int>(frame.data[1]) << " "
                          << static_cast<int>(frame.data[2]) << " "
                          << static_cast<int>(frame.data[3]) << " "
                          << static_cast<int>(frame.data[4]) << " "
                          << static_cast<int>(frame.data[5]) << " "
                          << static_cast<int>(frame.data[6]) << " "
                          << static_cast<int>(frame.data[7]) << std::dec);

                validFrameCount_++;
            } else {
                DEBUG_PRINT_IF(ENABLE_DEBUG_OUTPUT,
                             "[DEBUG][SendThread::buildAndSendPdoFrames]: 跳过无效数据帧，COB-ID=0x"
                             << std::hex << pdoDataMap_[i].cobId << std::dec);
            }
        }
        
        // 步骤4：数据完整性验证（可选，调试用）
        #if ENABLE_DEBUG_OUTPUT
        if (!validatePdoDataIntegrity()) {
            DEBUG_PRINT("[WARN][SendThread::buildAndSendPdoFrames]: PDO数据完整性验证发现问题");
        }
        #endif

        // 步骤5：发送前的最终数据完整性验证
        DEBUG_PRINT("[DEBUG][SendThread::buildAndSendPdoFrames]: 开始发送前验证，有效帧数: "
                  << static_cast<int>(validFrameCount_));

        for (size_t i = 0; i < validFrameCount_; ++i) {
            const CanFrame& frame = framesToSend_[i];

            // 验证帧ID范围
            if (frame.frameID < 0x200 || frame.frameID > 0x506) {
                DEBUG_PRINT("[ERROR][SendThread::buildAndSendPdoFrames]: 发送前验证失败 - 非法帧ID: 0x"
                          << std::hex << frame.frameID << std::dec);
                validFrameCount_ = 0; // 取消所有发送
                return 0;
            }

            // 验证DLC
            if (frame.dlc != 8) {
                DEBUG_PRINT("[ERROR][SendThread::buildAndSendPdoFrames]: 发送前验证失败 - 错误DLC: "
                          << static_cast<int>(frame.dlc) << " (期望8)");
                validFrameCount_ = 0; // 取消所有发送
                return 0;
            }
        }

        // 步骤6：批量发送CAN帧
        if (validFrameCount_ > 0) {
            try {
                DEBUG_PRINT("[DEBUG][SendThread::buildAndSendPdoFrames]: 开始批量发送CAN帧，帧数: "
                          << static_cast<int>(validFrameCount_));

                // 直接使用预分配数组发送，避免vector拷贝开销
                size_t sentFrames = serialManager_.sendFramesBatch(
                    {framesToSend_.data(), framesToSend_.data() + validFrameCount_});

                DEBUG_PRINT("[DEBUG][SendThread::buildAndSendPdoFrames]: 批量发送完成，成功帧数: "
                          << static_cast<int>(sentFrames));

                totalFramesProcessed = sentFrames;
            } catch (const std::exception& e) {
                setLastError(ThreadError::COMMUNICATION_ERROR,
                           std::string("PDO批量发送失败: ") + e.what());
                return 0;
            }
        } else {
            DEBUG_PRINT("[DEBUG][SendThread::buildAndSendPdoFrames]: 无有效帧需要发送");
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