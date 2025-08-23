#ifndef CIRCULAR_BUFFER_HPP
#define CIRCULAR_BUFFER_HPP

#include <vector>
#include <cstdint>
#include <iostream>
#include <cstring>
#include <algorithm>
#include <mutex>  // 引入互斥锁库
#include <atomic>


#define BUFFER_SIZE 512  // 环形缓冲区大小，使用宏定义
#define DEBUG_MODE 0

// 环形缓冲区类
class CircularBuffer {
public:

    std::atomic<bool> isbuffer_busy = false;//用于缓冲区繁忙状态的标志位，需要在调用函数里面进行检测          
                     
                               //尝试考虑在外部操作的可能性，来保证缓冲区的占用模式。     存在风险，需要评估

    CircularBuffer() : head(0), tail(0), size(0) {
        buffer.resize(BUFFER_SIZE);  // 根据宏定义的大小初始化缓冲区
    }

    // 写数据到环形缓冲区
    // 参数：data - 输入的数据，len - 数据长度
    bool write(const uint8_t* data, size_t len) {

        std::lock_guard<std::recursive_mutex> lock(mutex);  // 自动加锁，离开作用域时自动解锁

        isbuffer_busy = true;//将缓冲区繁忙标志位设置为繁忙状态

        
        if (len + size > BUFFER_SIZE) {

            
            // 如果缓冲区溢出，尝试丢弃最旧的数据
            size_t overflow = len + size - BUFFER_SIZE;  //计算多余的量
 
            head = (head + overflow) % BUFFER_SIZE;     //          有优化余地，也许可以直接取消缓冲区溢出的功能

            size -= overflow;    //减少丢弃的数据量，把覆盖完的部分作为容量

        }
        for (size_t i = 0; i < len; ++i) {
            buffer[tail] = data[i];
            tail = (tail + 1) % BUFFER_SIZE;          //除法取余数操作损失性能，尝试转化为乘法操作优化速度      也许可以把这部分计算转化到开头     施工中
        }
        
        size = std::min(size + len, static_cast<size_t>(BUFFER_SIZE));  // 防止 size 超过 BUFFER_SIZE

        isbuffer_busy = false;//写入完释放标志位

        return true;
    }

    // 从缓冲区读取数据
    // 参数：data - 输出数据的指针，len - 需要读取的数据长度
    bool read(uint8_t* data, size_t len) {

        std::lock_guard<std::recursive_mutex> lock(mutex);  // 自动加锁，离开作用域时自动解锁
        isbuffer_busy = true;//将缓冲区繁忙标志位设置为繁忙状态
        if (len > size) {

            

#if DEBUG_MODE
            std::cerr << "Not enough data in buffer!" << std::endl;
#endif
            isbuffer_busy = false;//报错时也要释放缓冲区
            return false;  // 缓冲区数据不足
        }

        for (size_t i = 0; i < len; ++i) {
            data[i] = buffer[head];
            head = (head + 1) % BUFFER_SIZE;           //读取也可以考虑优化
        }
        size -= len;

        isbuffer_busy = false;//将缓冲区繁忙标志位设置为空闲状态

        return true;
    }

    // 获取当前缓冲区大小
    size_t getSize() const {
        return size;
    }



private:
    std::vector<uint8_t> buffer;  // 数据存储
    size_t head;                  // 读指针
    size_t tail;                  // 写指针
    size_t size;                  // 当前缓冲区大小
    mutable std::recursive_mutex mutex;  // 替换为递归锁
};

#endif // CIRCULAR_BUFFER_HPP
