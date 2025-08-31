/**
 * @file CAN_Queue.cpp
 * @brief CAN通信全局队列变量定义
 * 
 * 定义CAN_Queue.hpp中声明的全局队列变量
 */

#include "CAN_Queue.hpp"

// 发送队列定义（无锁队列，大小1024）
boost::lockfree::queue<CanFrame> sendQueue(1024);

// 接收队列定义（无锁队列，大小1024）
boost::lockfree::queue<CanFrame> receiveQueue(1024);

// 规划队列定义（有锁队列）
std::queue<CanFrame> planQueue;
std::mutex planQueueMutex;
std::condition_variable planQueueCV;

// 全局队列计数变量定义
std::atomic<size_t> sendQueueCount(0);
std::atomic<size_t> receiveQueueCount(0);
std::atomic<size_t> planQueueCount(0);