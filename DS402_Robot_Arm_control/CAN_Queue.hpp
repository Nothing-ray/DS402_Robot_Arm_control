/**
 * @file CAN_Queue.hpp
 * @brief CAN通信全局队列定义
 * 
 * 实现基于CANOpen协议的机械臂控制系统中的三种全局队列：
 * - 发送队列（无锁）：单生产单消费，存储待发送的CAN帧
 * - 接收队列（无锁）：接收线程处理缓冲区数据
 * - 规划队列（有锁）：存储规划线程生成的SDO帧
 */

#ifndef CAN_QUEUE_HPP
#define CAN_QUEUE_HPP

#include <atomic>
#include <mutex>
#include <queue>
#include <vector>
#include <condition_variable>
#include <memory>

#include <boost/lockfree/queue.hpp>

#include "CAN_frame.hpp"

/**
 * @brief 发送队列（无锁队列）
 * 
 * 单生产单消费模式，由发送线程专用 
 * 队列大小：1024帧
 * 使用原始指针确保平凡可复制性
 */
extern boost::lockfree::queue<CanFrame*> sendQueue;

/**
 * @brief 接收队列（无锁队列）
 * 
 * 接收线程将数据从缓冲区转移至此队列
 * 队列大小：1024帧
 * 使用原始指针确保平凡可复制性
 */
extern boost::lockfree::queue<CanFrame*> receiveQueue;

/**
 * @brief 规划队列（有锁队列）
 * 
 * 存储规划线程生成的SDO帧
 * 发送线程在每个周期优先处理此队列
 * 使用shared_ptr确保一致性
 */
extern std::queue<std::shared_ptr<CanFrame>> planQueue;
extern std::mutex planQueueMutex;
extern std::condition_variable planQueueCV;

/**
 * @brief 向发送队列添加CAN帧
 * @param frame 待发送的CAN帧
 * @return 添加成功返回true，队列满返回false
 */
inline bool pushToSendQueue(const CanFrame& frame) {
    CanFrame* framePtr = new CanFrame(frame);
    return sendQueue.push(framePtr);
}

/**
 * @brief 批量向发送队列添加CAN帧
 * @param frames CAN帧向量
 * @return 成功添加的帧数量
 */
inline size_t pushBatchToSendQueue(const std::vector<CanFrame>& frames) {
    size_t count = 0;
    for (const auto& frame : frames) {
        CanFrame* framePtr = new CanFrame(frame);
        if (sendQueue.push(framePtr)) {
            ++count;
        } else {
            delete framePtr; // 队列满时释放内存
            break;
        }
    }
    return count;
}

/**
 * @brief 从发送队列取出CAN帧
 * @param framePtr 输出参数，取出的CAN帧指针
 * @return 取出成功返回true，队列空返回false
 */
inline bool popFromSendQueue(CanFrame*& framePtr) {
    return sendQueue.pop(framePtr);
}

/**
 * @brief 批量从发送队列取出CAN帧
 * @param framePtrs 输出参数，取出的CAN帧指针向量
 * @param maxCount 最大取出数量
 * @return 实际取出的帧数量
 */
inline size_t popBatchFromSendQueue(std::vector<CanFrame*>& framePtrs, size_t maxCount) {
    framePtrs.clear();
    framePtrs.reserve(maxCount);
    
    CanFrame* framePtr = nullptr;
    size_t count = 0;
    
    while (count < maxCount && sendQueue.pop(framePtr)) {
        framePtrs.push_back(framePtr);
        ++count;
    }
    
    return count;
}

/**
 * @brief 检查发送队列是否为空
 * @return 队列空返回true
 */
inline bool isSendQueueEmpty() {
    return sendQueue.empty();
}


/**
 * @brief 向接收队列添加CAN帧
 * @param frame 接收到的CAN帧
 * @return 添加成功返回true，队列满返回false
 */
inline bool pushToReceiveQueue(const CanFrame& frame) {
    CanFrame* framePtr = new CanFrame(frame);
    return receiveQueue.push(framePtr);
}

/**
 * @brief 批量向接收队列添加CAN帧
 * @param frames CAN帧向量
 * @return 成功添加的帧数量
 */
inline size_t pushBatchToReceiveQueue(const std::vector<CanFrame>& frames) {
    size_t count = 0;
    for (const auto& frame : frames) {
        CanFrame* framePtr = new CanFrame(frame);
        if (receiveQueue.push(framePtr)) {
            ++count;
        } else {
            delete framePtr; // 队列满时释放内存
            break;
        }
    }
    return count;
}

/**
 * @brief 从接收队列取出CAN帧
 * @param framePtr 输出参数，取出的CAN帧指针
 * @return 取出成功返回true，队列空返回false
 */
inline bool popFromReceiveQueue(CanFrame*& framePtr) {
    return receiveQueue.pop(framePtr);
}

/**
 * @brief 批量从接收队列取出CAN帧
 * @param framePtrs 输出参数，CAN帧指针向量
 * @param maxCount 最大取出数量
 * @return 实际取出的帧数量
 */
inline size_t popBatchFromReceiveQueue(std::vector<CanFrame*>& framePtrs, size_t maxCount) {
    framePtrs.clear();
    framePtrs.reserve(maxCount);
    
    CanFrame* framePtr = nullptr;
    size_t count = 0;
    
    while (count < maxCount && receiveQueue.pop(framePtr)) {
        framePtrs.push_back(framePtr);
        ++count;
    }
    
    return count;
}

/**
 * @brief 检查接收队列是否为空
 * @return 队列空返回true
 */
inline bool isReceiveQueueEmpty() {
    return receiveQueue.empty();
}


/**
 * @brief 向规划队列添加SDO帧
 * @param frame 规划线程生成的SDO帧
 */
inline void pushToPlanQueue(const CanFrame& frame) {
    std::lock_guard<std::mutex> lock(planQueueMutex);
    planQueue.push(std::make_shared<CanFrame>(frame));
    planQueueCV.notify_one();
}

/**
 * @brief 批量向规划队列添加SDO帧
 * @param frames SDO帧向量
 */
inline void pushBatchToPlanQueue(const std::vector<CanFrame>& frames) {
    std::lock_guard<std::mutex> lock(planQueueMutex);
    for (const auto& frame : frames) {
        planQueue.push(std::make_shared<CanFrame>(frame));
    }
    if (!frames.empty()) {
        planQueueCV.notify_one();
    }
}

/**
 * @brief 从规划队列取出SDO帧
 * @param framePtr 输出参数，取出的SDO帧智能指针
 * @param timeoutMs 超时时间（毫秒），0表示不等待
 * @return 取出成功返回true，超时或队列空返回false
 */
inline bool popFromPlanQueue(std::shared_ptr<CanFrame>& framePtr, uint32_t timeoutMs = 0) {
    std::unique_lock<std::mutex> lock(planQueueMutex);
    
    if (timeoutMs == 0) {
        if (planQueue.empty()) {
            return false;
        }
    } else {
        if (!planQueueCV.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                                [] { return !planQueue.empty(); })) {
            return false;
        }
    }
    
    framePtr = planQueue.front();
    planQueue.pop();
    return true;
}

/**
 * @brief 批量从规划队列取出SDO帧
 * @param framePtrs 输出参数，取出的SDO帧智能指针向量
 * @param maxCount 最大取出数量
 * @return 实际取出的帧数量
 */
inline size_t popBatchFromPlanQueue(std::vector<std::shared_ptr<CanFrame>>& framePtrs, size_t maxCount) {
    std::lock_guard<std::mutex> lock(planQueueMutex);
    
    framePtrs.clear();
    size_t count = 0;
    
    while (count < maxCount && !planQueue.empty()) {
        framePtrs.push_back(planQueue.front());
        planQueue.pop();
        ++count;
    }
    
    return count;
}

/**
 * @brief 检查规划队列是否为空
 * @return 队列空返回true
 */
inline bool isPlanQueueEmpty() {
    std::lock_guard<std::mutex> lock(planQueueMutex);
    return planQueue.empty();
}


/**
 * @brief 清空规划队列
 */
inline void clearPlanQueue() {
    std::lock_guard<std::mutex> lock(planQueueMutex);
    while (!planQueue.empty()) {
        planQueue.pop();
    }
}

/**
 * @brief 清空所有队列
 */
inline void clearAllQueues() {
    // 清空无锁队列并释放内存
    CanFrame* dummy = nullptr;
    while (sendQueue.pop(dummy)) {
        delete dummy;
    }
    while (receiveQueue.pop(dummy)) {
        delete dummy;
    }
    
    // 清空有锁队列
    clearPlanQueue();
}

/**
 * @brief 检查所有队列是否都为空
 * @return 所有队列都空返回true
 */
inline bool areAllQueuesEmpty() {
    return isSendQueueEmpty() && isReceiveQueueEmpty() && isPlanQueueEmpty();
}

// 全局队列计数变量（原子操作）
extern std::atomic<size_t> sendQueueCount;
extern std::atomic<size_t> receiveQueueCount;
extern std::atomic<size_t> planQueueCount;


#endif // CAN_QUEUE_HPP