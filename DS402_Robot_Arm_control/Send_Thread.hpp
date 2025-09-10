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

#include "CAN_frame.hpp"
#include "CircularBuffer.hpp"
#include "SDO_State_Machine.hpp"
#include "CLASS_Motor.hpp"
#include "Serial_Module.hpp"

// 配置宏定义
#ifndef SEND_THREAD_CONFIG_H
#define SEND_THREAD_CONFIG_H

// 超时处理详细模式（默认关闭）
// #define ENABLE_DETAILED_TIMEOUT_HANDLING

// 调试输出控制（默认开启）
#ifndef DISABLE_DEBUG_OUTPUT
#define ENABLE_DEBUG_OUTPUT
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
    std::vector<Motor>& motors_;
    SerialPortManager& serialManager_;  ///< 串口管理器引用
    
    // 线程控制
    std::atomic<bool> running_{false};
    std::thread sendThread_;
    canopen::AtomicSdoStateMachine sdoStateMachine_;
    
    // SDO批次管理（简化状态机）
    std::atomic<uint8_t> sdoBatchRemaining_{0};        ///< 当前批次剩余帧数（0-255，足够覆盖正常场景）
    std::atomic<bool> sdoBatchActive_{false};           ///< 批次激活标志
    
    // 配置参数
    const uint8_t motorCount_;
    
    // 预分配的13字节CAN帧缓冲区（避免内存分配开销）
    alignas(64) std::array<uint8_t, CAN_FRAME_SIZE> canFrameBuffer_;
    
    // 性能监控（调试用）
    std::chrono::steady_clock::time_point lastCycleTime_;
    uint32_t cycleCount_{0};
    
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
    
    /**
     * @brief 发送线程主循环
     */
    void threadLoop() {
        constexpr std::chrono::milliseconds SYNC_PERIOD{2};
        
        while (running_.load(std::memory_order_relaxed)) {
            auto cycleStart = std::chrono::steady_clock::now();
            
            // 周期性能监控（每1000个周期输出一次）
            if (cycleCount_++ % 1000 == 0 && cycleCount_ > 1) {
                auto actualPeriod = cycleStart - lastCycleTime_;
                #ifdef ENABLE_DEBUG_OUTPUT
                std::cout << "[DEBUG][SendThread]: "
                          << "周期统计: 实际周期=" 
                          << std::chrono::duration_cast<std::chrono::microseconds>(actualPeriod).count() 
                          << "μs, 期望=" << SYNC_PERIOD.count() * 1000 << "μs" << std::endl;
                #endif
            }
            lastCycleTime_ = cycleStart;
            
            // 1. 简化批次状态检查
            bool hasBatch = sdoBatchActive_.load(std::memory_order_relaxed);
            uint8_t batchRemaining = sdoBatchRemaining_.load(std::memory_order_relaxed);
            bool hasSdoFrames = planBuffer_.getUsedSpace() >= CAN_FRAME_SIZE;
            
            // 2. 处理SDO帧（批次优先）
            if (hasBatch || hasSdoFrames) {
                size_t sdoFramesProcessed = processSdoFrames();
                
                // 3. 批次完成后自动转向PDO
                if (!sdoBatchActive_.load(std::memory_order_relaxed) && !hasSdoFrames) {
                    processPdoFrames();
                }
            } else {
                // 无SDO帧需要处理，直接处理PDO
                processPdoFrames();
            }
            
            // 高精度周期控制：忙等待确保确定性
            auto elapsed = std::chrono::steady_clock::now() - cycleStart;
            if (elapsed < SYNC_PERIOD) {
                auto remaining = SYNC_PERIOD - elapsed;
                auto target = cycleStart + SYNC_PERIOD;
                
                // 使用忙等待确保精确的2ms周期
                while (std::chrono::steady_clock::now() < target) {
                    // 短暂暂停避免CPU占用100%
                    std::this_thread::sleep_for(std::chrono::microseconds(10));
                }
            }
        }
    }
     
    /**
     * @brief 处理单帧SDO（字节级操作 + 柯理化设计）
     * 
     * @details 从规划缓冲区读取一帧SDO数据，使用预分配缓冲区和字节级操作
     * - 使用预分配的13字节缓冲区避免内存分配
     * - 直接从环形缓冲区读取到预分配缓冲区
     * - 使用prepareTransactionFromBytes避免CanFrame对象创建
     * - 柯理化设计：返回处理结果供外部决策
     * 
     * @return 处理结果枚举：SUCCESS, RETRY_NEEDED, NO_FRAME, ERROR
     * 
     * @note 使用预分配缓冲区完全避免动态内存分配
     * @warning 必须确保缓冲区有完整的13字节CAN帧数据
     */
    enum class SdoProcessResult {
        SUCCESS,        ///< 处理成功
        RETRY_NEEDED,   ///< 需要重试
        NO_FRAME,       ///< 无可用帧
        ERROR           ///< 处理错误
    };
    
    inline SdoProcessResult processSingleSdoFrame() {
        // 1. 使用peek查看数据但不消费（事务性操作）
        auto frameData = planBuffer_.peek();//从缓冲区中读取（但是不消耗）数据来进行注册

        //未读取到数据的情况
        if (!frameData.has_value()) {
            return SdoProcessResult::NO_FRAME;
        }
        
        try {
            // 2. 使用字节级操作准备SDO事务（柯理化设计）
            // 将读取到的CAN字节注册到状态机
            auto transaction = sdoStateMachine_.prepareTransactionFromBytes(frameData->data());
            
            // 3. 启动事务
            if (!sdoStateMachine_.startTransaction(transaction)) {//未启动的情况
                #ifdef ENABLE_DEBUG_OUTPUT
                std::cout << "[WARN][SendThread::processSingleSdoFrame]: "
                          << "事务启动失败，状态机繁忙" << std::endl;
                #endif
                return SdoProcessResult::RETRY_NEEDED;
            }

            // SDO状态机启动成功
            
            // 4. 事务启动成功，现在可以安全消费数据
            size_t bytesPopped = planBuffer_.popBytes(canFrameBuffer_.data(), CAN_FRAME_SIZE);//从环形缓冲区中读取出要发送的数据
            if (bytesPopped != CAN_FRAME_SIZE) {//读取失败的情况
                #ifdef ENABLE_DEBUG_OUTPUT
                std::cout << "[ERROR][SendThread::processSingleSdoFrame]: "
                          << "数据消费失败，预期13字节，实际: " << bytesPopped << std::endl;
                #endif
                sdoStateMachine_.completeTransaction();
                return SdoProcessResult::ERROR;
            }
            
            // 5. 使用sendBufferSync发送单帧（复用批量发送接口）
            size_t bytesSent = serialManager_.sendBufferSync(sendBuffer_, CAN_FRAME_SIZE, false);//只发送13字节
            
            if (bytesSent != CAN_FRAME_SIZE) {//发送不完整的情况
                #ifdef ENABLE_DEBUG_OUTPUT
                std::cout << "[ERROR][SendThread::processSingleSdoFrame]: "
                          << "SDO帧发送失败，预期13字节，实际: " << bytesSent << std::endl;
                #endif
                sdoStateMachine_.completeTransaction();
                return SdoProcessResult::ERROR;
            }
            
            #ifdef ENABLE_DEBUG_OUTPUT
            std::cout << "[DEBUG][SendThread::processSingleSdoFrame]: "
                      << "SDO帧发送成功，等待响应" << std::endl;
            #endif
            
            return SdoProcessResult::SUCCESS;
            
        } catch (const std::exception& e) {
            #ifdef ENABLE_DEBUG_OUTPUT
            std::cout << "[ERROR][SendThread::processSingleSdoFrame]: "
                      << "处理异常: " << e.what() << std::endl;
            #endif
            return SdoProcessResult::ERROR;
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
            
            #ifdef ENABLE_DEBUG_OUTPUT
            if (needsRetry) {
                uint8_t retryCount = sdoStateMachine_.getRetryCount();
                std::cout << "[DEBUG][SendThread::processSdoFrames]: "
                          << "SDO超时检测，当前重试次数: " << static_cast<int>(retryCount) << "/" 
                          << MAX_RETRY_COUNT << std::endl;
            }
            #endif
            
            // 如果需要重试且状态机允许重试，优先处理重试
            if (needsRetry && sdoStateMachine_.needsRetry()) {
                retryInProgress = true;
                #ifdef ENABLE_DEBUG_OUTPUT
                std::cout << "[DEBUG][SendThread::processSdoFrames]: "
                          << "SDO需要重试，将在下一周期重新发送" << std::endl;
                #endif
                return 0; // 重试优先级最高，本周期不处理新帧
            }
            
        } catch (const std::runtime_error& e) {
            // 超过最大重试次数，清理状态并重新抛出异常
            #ifdef ENABLE_DEBUG_OUTPUT
            std::cout << "[ERROR][SendThread::processSdoFrames]: "
                      << "SDO通信失败，强制清理状态: " << e.what() << std::endl;
            #endif
            
            resetBatchState();
            throw; // 重新抛出异常，确保机械臂安全
            
        } catch (const std::exception& e) {
            // 其他异常，记录并重置状态机
            #ifdef ENABLE_DEBUG_OUTPUT
            std::cout << "[ERROR][SendThread::processSdoFrames]: "
                      << "意外异常: " << e.what() << std::endl;
            #endif
            
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
                    case SdoProcessResult::ERROR:
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
                    case SdoProcessResult::ERROR:
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
        
        #ifdef ENABLE_DEBUG_OUTPUT
        std::cout << "[ERROR][SendThread]: " << message << std::endl;
        #endif
    }
    
    /**
     * @brief 处理PDO帧（占位实现）
     */
    void processPdoFrames() {
        // 占位实现，后续完善
    }
    
    
public:
    /**
     * @brief 构造函数
     * 
     * @param sendBuffer 发送环形缓冲区引用
     * @param planBuffer 规划环形缓冲区引用
     * @param motors 电机向量引用
     * @param motorCount 电机数量
     * @param serialManager 串口管理器引用
     */
    SendThread(CircularBuffer& sendBuffer,
               CircularBuffer& planBuffer,
               std::vector<Motor>& motors,
               uint8_t motorCount,
               SerialPortManager& serialManager)
        : sendBuffer_(sendBuffer)
        , planBuffer_(planBuffer)
        , motors_(motors)
        , motorCount_(motorCount)
        , serialManager_(serialManager)
    {
        // 基础参数验证
        if (motorCount_ == 0 || motorCount_ > 12) {
            throw std::invalid_argument("[ERROR][SendThread]: 电机数量必须在1-12之间");
        }
        
        if (motors_.size() != motorCount_) {
            throw std::invalid_argument("[ERROR][SendThread]: 电机向量大小与电机数量不匹配");
        }
        
        // 初始化预分配缓冲区
        canFrameBuffer_.fill(0);
        
        #ifdef ENABLE_DEBUG_OUTPUT
        std::cout << "[INFO][SendThread]: 构造函数完成，电机数量: " 
                  << static_cast<int>(motorCount_) << std::endl;
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
            if (motors_.empty()) {
                throw std::runtime_error("[ERROR][SendThread::start]: 电机向量未初始化");
            }
            
            sendThread_ = std::thread(&SendThread::threadLoop, this);
            
            #ifdef ENABLE_DEBUG_OUTPUT
            std::cout << "[INFO][SendThread::start]: 线程已启动" << std::endl;
            #endif
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
            
            #ifdef ENABLE_DEBUG_OUTPUT
            std::cout << "[INFO][SendThread::stop]: 线程已停止" << std::endl;
            #endif
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
        
        #ifdef ENABLE_DEBUG_OUTPUT
        std::cout << "[DEBUG][SendThread::setSdoBatchSize]: "
                  << "批次设置，大小: " << static_cast<int>(batchSize) << std::endl;
        #endif
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
};


#undef ENABLE_DEBUG_OUTPUT
#endif // SEND_THREAD_HPP