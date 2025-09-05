/**
 * @file CircularBuffer.hpp
 * @brief 高性能环形缓冲区 - 专为CAN帧通信优化
 * 
 * @details 实现固定容量的环形缓冲区，容量1024字节(2的10次方)，
 * 使用位运算优化性能，直接内存操作，条件变量实现线程安全，专为CAN帧通信设计。
 * 
 * 主要特性：
 * - 固定容量1024字节，2的幂次方便于位运算优化
 * - 位运算索引计算（单周期指令），避免昂贵的模运算
 * - 直接内存循环赋值，避免函数调用开销
 * - 条件变量+互斥锁实现高效线程同步
 * - 13字节倍数验证，确保CAN帧完整性
 * - 内联函数优化关键路径性能
 * 
 * @note 容量选择1024字节同时满足：
 * - 2的幂次方容量，位运算优化最佳
 * - 内存充裕（16GB系统），性能优先于内存利用率
 * - 缓存行64字节对齐优化
 */

#ifndef CIRCULAR_BUFFER_HPP
#define CIRCULAR_BUFFER_HPP

#include <vector>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <stdexcept>
#include <algorithm>
#include <cstring>

#include "CAN_frame.hpp"

// 使用1024字节容量以获得位运算优化（2的10次方）
constexpr size_t CIRCULAR_BUFFER_CAPACITY = 1024; // 2^10 = 1024

// CAN帧固定大小
constexpr size_t CAN_FRAME_SIZE = 13;

/**
 * @brief 高性能环形缓冲区类
 * 
 * @details 专为CAN帧通信设计，支持高效的二进制数据操作，
 * 使用位运算优化性能，条件变量实现线程安全。
 */
class CircularBuffer {
private:
    // 第一缓存行：写相关变量（生产者核心使用）
    struct alignas(64) WriterState {
        uint32_t writePos_{0};
        // 编译器会自动填充到64字节
    } writer_;
    
    // 第二缓存行：读相关变量（消费者核心使用）  
    struct alignas(64) ReaderState {
        uint32_t readPos_{0};
        // 编译器会自动填充到64字节
    } reader_;
    
    // 第三缓存行：控制变量（可能被双方访问）
    struct alignas(64) ControlState {
        uint32_t used_{0};
        mutable std::mutex mutex_;  // 添加 mutable 以允许在 const 函数中锁定
        // 编译器会自动填充到64字节
    } control_;
    
    // 第四缓存行：缓冲区起始（确保独立缓存行）
    alignas(64) std::vector<uint8_t> buffer_;
    
    // 条件变量分开对齐
    alignas(64) std::condition_variable notEmpty_;
    alignas(64) std::condition_variable notFull_;
    
    /**
     * @brief 计算环形索引（位运算优化）
     * @param index 原始索引
     * @return 环形缓冲区内的实际索引
     */
    inline uint32_t maskIndex(uint32_t index) const {
        return index & (CIRCULAR_BUFFER_CAPACITY - 1);
    }
    
    /**
     * @brief 验证数据大小是否为13的倍数
     * @param size 数据大小
     * @throw std::invalid_argument 如果大小不是13的倍数
     */
    inline void validateFrameSize(uint32_t size) const {
        if (size % CAN_FRAME_SIZE != 0) {
            throw std::invalid_argument("数据大小必须是13的倍数");
        }
    }
    
public:
    /**
     * @brief 构造函数
     * 
     * @details 初始化832字节容量的环形缓冲区，
     * 确保容量为2的幂次方以便位运算优化。
     */
    CircularBuffer() 
        : buffer_(CIRCULAR_BUFFER_CAPACITY) {}
    
    // 禁止拷贝和移动
    CircularBuffer(const CircularBuffer&) = delete;
    CircularBuffer& operator=(const CircularBuffer&) = delete;
    CircularBuffer(CircularBuffer&&) = delete;
    CircularBuffer& operator=(CircularBuffer&&) = delete;
    
    /**
     * @brief 添加单个CAN帧到缓冲区
     * @param frame 待添加的CAN帧
     * @return 添加成功返回true，缓冲区满返回false
     */
    bool pushFrame(const CanFrame& frame) {
        const auto& binaryData = frame.getBinaryFrame();
        return pushBytes(binaryData.data(), binaryData.size());
    }
    
    /**
     * @brief 批量添加CAN帧到缓冲区
     * @param frames CAN帧向量
     * @return 成功添加的帧数量
     */
    size_t pushFrames(const std::vector<CanFrame>& frames) {
        size_t successCount = 0;
        
        for (const auto& frame : frames) {
            if (pushFrame(frame)) {
                ++successCount;
            } else {
                break; // 缓冲区满，停止添加
            }
        }
        
        return successCount;
    }
    
    /**
     * @brief 添加二进制数据到缓冲区
     * @param data 待添加的数据指针
     * @param size 数据大小（必须是13的倍数）
     * @return 添加成功返回true，缓冲区满返回false
     */
    bool pushBytes(const uint8_t* data, size_t size) {
        validateFrameSize(size);
        
        std::unique_lock<std::mutex> lock(control_.mutex_);
        
        // 等待缓冲区有足够空间
        notFull_.wait(lock, [this, size] { 
            return (CIRCULAR_BUFFER_CAPACITY - control_.used_) >= size; 
        });
        
        // 计算写入位置和连续空间
        uint32_t writeIndex = maskIndex(writer_.writePos_);
        uint32_t contiguousSpace = CIRCULAR_BUFFER_CAPACITY - writeIndex;
        
        if (size <= contiguousSpace) {
            // 单次写入即可完成 - 使用memcpy优化性能
            std::memcpy(&buffer_[writeIndex], data, size);
        } else {
            // 需要分两次写入
            std::memcpy(&buffer_[writeIndex], data, contiguousSpace);
            std::memcpy(&buffer_[0], data + contiguousSpace, size - contiguousSpace);
        }
        
        // 更新写位置和已使用空间
        writer_.writePos_ += size;
        control_.used_ += size;
        
        // 通知等待的读取线程
        notEmpty_.notify_one();
        
        return true;
    }
    
    /**
     * @brief 取出二进制数据（非阻塞）
     * @param buffer 输出缓冲区
     * @param maxSize 最大取出字节数
     * @return 实际取出的字节数
     */
    size_t popBytes(uint8_t* buffer, size_t maxSize) {
        std::unique_lock<std::mutex> lock(control_.mutex_);
        
        if (control_.used_ == 0) {
            return 0;
        }
        
        // 计算实际可读取的字节数
        uint32_t bytesToRead = std::min(static_cast<uint32_t>(maxSize), control_.used_);
        
        // 计算读取位置和连续空间
        uint32_t readIndex = maskIndex(reader_.readPos_);
        uint32_t contiguousSpace = CIRCULAR_BUFFER_CAPACITY - readIndex;
        
        if (bytesToRead <= contiguousSpace) {
            // 单次读取即可完成 - 使用memcpy优化性能
            std::memcpy(buffer, &buffer_[readIndex], bytesToRead);
        } else {
            // 需要分两次读取
            std::memcpy(buffer, &buffer_[readIndex], contiguousSpace);
            std::memcpy(buffer + contiguousSpace, &buffer_[0], bytesToRead - contiguousSpace);
        }
        
        // 更新读位置和已使用空间
        reader_.readPos_ += bytesToRead;
        control_.used_ -= bytesToRead;
        
        // 通知等待的写入线程
        notFull_.notify_one();
        
        return bytesToRead;
    }
    
    /**
     * @brief 取出二进制数据（阻塞，带超时）
     * @param buffer 输出缓冲区
     * @param maxSize 最大取出字节数
     * @param timeoutMs 超时时间（毫秒），0表示无限等待
     * @return 实际取出的字节数
     */
    size_t popBytesBlocking(uint8_t* buffer, size_t maxSize, uint32_t timeoutMs = 0) {
        std::unique_lock<std::mutex> lock(control_.mutex_);
        
        // 等待数据可用
        if (timeoutMs == 0) {
            notEmpty_.wait(lock, [this] { return control_.used_ > 0; });
        } else {
            if (!notEmpty_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                                  [this] { return control_.used_ > 0; })) {
                return 0; // 超时
            }
        }
        
        // 直接在此实现读取逻辑，避免递归锁
        if (control_.used_ == 0) {
            return 0;
        }
        
        // 计算实际可读取的字节数
        uint32_t bytesToRead = std::min(static_cast<uint32_t>(maxSize), control_.used_);
        
        // 计算读取位置和连续空间
        uint32_t readIndex = maskIndex(reader_.readPos_);
        uint32_t contiguousSpace = CIRCULAR_BUFFER_CAPACITY - readIndex;
        
        if (bytesToRead <= contiguousSpace) {
            // 单次读取即可完成 - 使用memcpy优化性能
            std::memcpy(buffer, &buffer_[readIndex], bytesToRead);
        } else {
            // 需要分两次读取
            std::memcpy(buffer, &buffer_[readIndex], contiguousSpace);
            std::memcpy(buffer + contiguousSpace, &buffer_[0], bytesToRead - contiguousSpace);
        }
        
        // 更新读位置和已使用空间
        reader_.readPos_ += bytesToRead;
        control_.used_ -= bytesToRead;
        
        // 通知等待的写入线程
        notFull_.notify_one();
        
        return bytesToRead;
    }
    
    /**
     * @brief 获取可用的完整帧数量
     * @return 可用完整帧数
     */
    inline uint32_t getAvailableFrames() const {
        std::lock_guard<std::mutex> lock(control_.mutex_);
        return control_.used_ / CAN_FRAME_SIZE;
    }
    
    /**
     * @brief 检查缓冲区是否为空
     * @return 空返回true
     */
    inline bool isEmpty() const {
        std::lock_guard<std::mutex> lock(control_.mutex_);
        return control_.used_ == 0;
    }
    
    /**
     * @brief 检查缓冲区是否已满
     * @return 满返回true
     */
    inline bool isFull() const {
        std::lock_guard<std::mutex> lock(control_.mutex_);
        return control_.used_ == CIRCULAR_BUFFER_CAPACITY;
    }
    
    /**
     * @brief 获取剩余空间大小
     * @return 剩余字节数
     */
    inline uint32_t getFreeSpace() const {
        std::lock_guard<std::mutex> lock(control_.mutex_);
        return CIRCULAR_BUFFER_CAPACITY - control_.used_;
    }
    
    /**
     * @brief 获取已使用空间大小
     * @return 已使用字节数
     */
    inline uint32_t getUsedSpace() const {
        std::lock_guard<std::mutex> lock(control_.mutex_);
        return control_.used_;
    }
    
    /**
     * @brief 清空缓冲区
     */
    void clear() {
        std::lock_guard<std::mutex> lock(control_.mutex_);
        reader_.readPos_ = 0;
        writer_.writePos_ = 0;
        control_.used_ = 0;
        
        // 通知所有等待线程
        notFull_.notify_all();
        notEmpty_.notify_all();
    }
    
    /**
     * @brief 获取缓冲区总容量
     * @return 总容量字节数
     */
    constexpr size_t getCapacity() const {
        return CIRCULAR_BUFFER_CAPACITY;
    }
};

#endif // CIRCULAR_BUFFER_HPP