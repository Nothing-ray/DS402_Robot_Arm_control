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
#include <iostream>
#include <stdexcept>

#include "CAN_frame.hpp"
#include "CAN_Queue.hpp"
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
    boost::lockfree::queue<CanFrame>& sendQueue_;
    std::queue<CanFrame>& planQueue_;
    std::mutex& planQueueMutex_;
    std::condition_variable& planQueueCV_;
    std::vector<Motor>& motors_;
    SerialPortManager& serialManager_;  ///< 串口管理器引用
    
    // 线程控制
    std::atomic<bool> running_{false};
    std::thread sendThread_;
    canopen::AtomicSdoStateMachine sdoStateMachine_;
    
    // 配置参数
    const uint8_t motorCount_;
    
    /**
     * @brief 发送线程主循环
     */
    void threadLoop() {
        constexpr std::chrono::milliseconds SYNC_PERIOD{2};
        
        while (running_.load(std::memory_order_acquire)) {
            auto cycleStart = std::chrono::steady_clock::now();
            
            try {
                // 处理SDO帧（占位实现）
                processSdoFrames();
                
                // 处理PDO帧（占位实现）
                processPdoFrames();
                
                // 检查SDO超时（留空实现）
                checkSdoTimeouts();
                
            } catch (const std::exception& e) {
                std::cout << "[ERROR][SendThread::threadLoop]: " << e.what() << std::endl;
            }
            
            // 不足2ms等待，超时自然跳过
            auto elapsed = std::chrono::steady_clock::now() - cycleStart;
            if (elapsed < SYNC_PERIOD) {
                std::this_thread::sleep_for(SYNC_PERIOD - elapsed);
            }
            // 超时情况：不做特殊处理，自然进入下一周期
        }
    }
    
    /**
     * @brief 处理SDO帧（占位实现）
     */
    void processSdoFrames() {
        // 占位实现，后续完善
    }
    
    /**
     * @brief 处理PDO帧（占位实现）
     */
    void processPdoFrames() {
        // 占位实现，后续完善
    }
    
    /**
     * @brief 检查SDO超时（留空实现）
     */
    void checkSdoTimeouts() {
        // 当前版本：空实现
        // 未来可通过编译宏启用详细超时处理
        #ifdef ENABLE_DETAILED_TIMEOUT_HANDLING
        // 详细的超时统计和处理逻辑
        #endif
    }
    
public:
    /**
     * @brief 构造函数
     * 
     * @param sendQueue 发送队列引用
     * @param planQueue 规划队列引用
     * @param planQueueMutex 规划队列互斥锁引用
     * @param planQueueCV 规划队列条件变量引用
     * @param motors 电机向量引用
     * @param motorCount 电机数量
     * @param serialManager 串口管理器引用
     */
    SendThread(boost::lockfree::queue<CanFrame>& sendQueue,
               std::queue<CanFrame>& planQueue,
               std::mutex& planQueueMutex,
               std::condition_variable& planQueueCV,
               std::vector<Motor>& motors,
               uint8_t motorCount,
               SerialPortManager& serialManager)
        : sendQueue_(sendQueue)
        , planQueue_(planQueue)
        , planQueueMutex_(planQueueMutex)
        , planQueueCV_(planQueueCV)
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
        if (!running_.exchange(true, std::memory_order_acq_rel)) {
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
        if (running_.exchange(false, std::memory_order_acq_rel)) {
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
        return running_.load(std::memory_order_acquire);
    }
    
    /**
     * @brief 获取SDO状态机引用
     * @return SDO状态机引用
     */
    canopen::AtomicSdoStateMachine& getSdoStateMachine() {
        return sdoStateMachine_;
    }
};

#endif // SEND_THREAD_HPP