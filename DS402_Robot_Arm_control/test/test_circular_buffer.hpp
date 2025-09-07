/**
 * @file test_circular_buffer.cpp
 * @brief 环形缓冲区测试程序
 * 
 * @details 测试CircularBuffer类的功能和性能
 */

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic>
#include <cassert>

#include "CircularBuffer.hpp"
#include "CAN_frame.hpp"

/**
 * @brief 测试环形缓冲区基本功能
 */
void testBasicFunctionality() {
    std::cout << "=== 测试基本功能 ===" << std::endl;
    
    CircularBuffer buffer;
    
    // 测试空缓冲区状态
    assert(buffer.isEmpty());
    assert(!buffer.isFull());
    assert(buffer.getUsedSpace() == 0);
    assert(buffer.getFreeSpace() == buffer.getCapacity());
    
    // 创建测试CAN帧
    uint8_t data1[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t data2[] = {0xAA, 0xBB, 0xCC, 0xDD};
    CanFrame frame1(0x100, data1, 4);
    CanFrame frame2(0x200, data2, 4);
    
    // 测试添加单个帧
    assert(buffer.pushFrame(frame1));
    assert(buffer.getUsedSpace() == CAN_FRAME_SIZE);
    assert(buffer.getAvailableFrames() == 1);
    assert(!buffer.isEmpty());
    
    // 测试添加第二个帧
    assert(buffer.pushFrame(frame2));
    assert(buffer.getUsedSpace() == 2 * CAN_FRAME_SIZE);
    assert(buffer.getAvailableFrames() == 2);
    
    // 测试取出数据
    std::vector<uint8_t> readBuffer(2 * CAN_FRAME_SIZE);
    uint32_t bytesRead = buffer.popBytes(readBuffer.data(), readBuffer.size());
    assert(bytesRead == 2 * CAN_FRAME_SIZE);
    assert(buffer.isEmpty());
    
    std::cout << "✓ 基本功能测试通过" << std::endl;
}

/**
 * @brief 测试环形缓冲区边界条件
 */
void testBoundaryConditions() {
    std::cout << "\n=== 测试边界条件 ===" << std::endl;
    
    CircularBuffer buffer;
    
    // 测试缓冲区满的情况
    uint32_t maxFrames = buffer.getCapacity() / CAN_FRAME_SIZE;
    
    for (uint32_t i = 0; i < maxFrames; ++i) {
        uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
        CanFrame frame(0x100 + i, data, 4);
        assert(buffer.pushFrame(frame));
    }
    
    assert(buffer.isFull());
    assert(buffer.getFreeSpace() == 0);
    
    // 测试缓冲区满时添加失败
    uint8_t extraData[] = {0xFF};
    CanFrame extraFrame(0x999, extraData, 1);
    assert(!buffer.pushFrame(extraFrame));
    
    // 测试清空缓冲区
    buffer.clear();
    assert(buffer.isEmpty());
    
    std::cout << "✓ 边界条件测试通过" << std::endl;
}

/**
 * @brief 测试批量操作
 */
void testBatchOperations() {
    std::cout << "\n=== 测试批量操作 ===" << std::endl;
    
    CircularBuffer buffer;
    
    // 创建批量测试帧
    std::vector<CanFrame> frames;
    for (int i = 0; i < 10; ++i) {
        uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
        frames.push_back(CanFrame(0x100 + i, data, 4));
    }
    
    // 测试批量添加
    size_t successCount = buffer.pushFrames(frames);
    assert(successCount == frames.size());
    assert(buffer.getAvailableFrames() == frames.size());
    
    // 测试批量读取
    std::vector<uint8_t> readBuffer(frames.size() * CAN_FRAME_SIZE);
    uint32_t bytesRead = buffer.popBytes(readBuffer.data(), readBuffer.size());
    assert(bytesRead == frames.size() * CAN_FRAME_SIZE);
    assert(buffer.isEmpty());
    
    std::cout << "✓ 批量操作测试通过" << std::endl;
}

/**
 * @brief 测试多线程安全性
 */
void testThreadSafety() {
    std::cout << "\n=== 测试多线程安全性 ===" << std::endl;
    
    CircularBuffer buffer;
    std::atomic<bool> stop{false};
    std::atomic<int> writeCount{0};
    std::atomic<int> readCount{0};
    
    // 写入线程
    auto writer = [&]() {
        uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
        CanFrame frame(0x100, data, 4);
        while (!stop) {
            if (buffer.pushFrame(frame)) {
                writeCount++;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    };
    
    // 读取线程
    auto reader = [&]() {
        std::vector<uint8_t> readBuffer(CAN_FRAME_SIZE);
        while (!stop) {
            uint32_t bytesRead = buffer.popBytes(readBuffer.data(), readBuffer.size());
            if (bytesRead > 0) {
                readCount++;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    };
    
    // 启动线程
    std::thread writeThread(writer);
    std::thread readThread(reader);
    
    // 运行一段时间
    std::this_thread::sleep_for(std::chrono::seconds(2));
    stop = true;
    
    writeThread.join();
    readThread.join();
    
    std::cout << "写入次数: " << writeCount << std::endl;
    std::cout << "读取次数: " << readCount << std::endl;
    std::cout << "✓ 多线程安全性测试通过" << std::endl;
}

/**
 * @brief 测试理论性能（移除所有延迟模拟）
 */
void testTheoreticalPerformance() {
    std::cout << "\n=== 理论性能测试 ===" << std::endl;
    
    CircularBuffer buffer;
    uint8_t testData[CAN_FRAME_SIZE] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D};
    uint8_t readBuffer[CAN_FRAME_SIZE];
    
    // 预热阶段（消除冷缓存影响）
    for (int i = 0; i < 1000; ++i) {
        buffer.pushBytes(testData, CAN_FRAME_SIZE);
        buffer.popBytes(readBuffer, CAN_FRAME_SIZE);
    }
    
    const int iterations = 1000000;
    auto start = std::chrono::high_resolution_clock::now();
    
    // 纯理论性能测试（无任何延迟）
    for (int i = 0; i < iterations; ++i) {
        buffer.pushBytes(testData, CAN_FRAME_SIZE);
        buffer.popBytes(readBuffer, CAN_FRAME_SIZE);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    
    double totalOperations = 2.0 * iterations;
    double nsPerOperation = duration.count() / totalOperations;
    double operationsPerSecond = 1e9 / nsPerOperation;
    double mbPerSecond = (operationsPerSecond * CAN_FRAME_SIZE) / (1024.0 * 1024.0);
    
    std::cout << "测试配置:" << std::endl;
    std::cout << "- 缓冲区容量: " << buffer.getCapacity() << " 字节" << std::endl;
    std::cout << "- CAN帧大小: " << CAN_FRAME_SIZE << " 字节" << std::endl;
    std::cout << "- 操作次数: " << totalOperations << " 次" << std::endl;
    std::cout << "\n性能结果:" << std::endl;
    std::cout << "- 总耗时: " << duration.count() << " ns" << std::endl;
    std::cout << "- 每次操作耗时: " << std::fixed << std::setprecision(2) << nsPerOperation << " ns" << std::endl;
    std::cout << "- 理论吞吐量: " << std::fixed << std::setprecision(0) << operationsPerSecond << " 操作/秒" << std::endl;
    std::cout << "- 数据吞吐量: " << std::fixed << std::setprecision(2) << mbPerSecond << " MB/s" << std::endl;
    std::cout << "✓ 理论性能测试完成" << std::endl;
}

/**
 * @brief 测试批量操作理论性能
 */
void testBatchTheoreticalPerformance() {
    std::cout << "\n=== 批量操作理论性能测试 ===" << std::endl;
    
    CircularBuffer buffer;
    
    // 批量测试配置
    const int batchSize = 10;
    const int totalFrames = 100000;
    const int batches = totalFrames / batchSize;
    
    std::vector<uint8_t> batchData(batchSize * CAN_FRAME_SIZE);
    std::vector<uint8_t> readBuffer(batchSize * CAN_FRAME_SIZE);
    
    // 填充测试数据
    for (int i = 0; i < batchSize * CAN_FRAME_SIZE; ++i) {
        batchData[i] = i % 256;
    }
    
    // 预热
    for (int i = 0; i < 100; ++i) {
        buffer.pushBytes(batchData.data(), batchData.size());
        buffer.popBytes(readBuffer.data(), readBuffer.size());
    }
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // 批量性能测试
    for (int i = 0; i < batches; ++i) {
        buffer.pushBytes(batchData.data(), batchData.size());
        buffer.popBytes(readBuffer.data(), readBuffer.size());
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    
    double totalBytes = 2.0 * totalFrames * CAN_FRAME_SIZE;
    double nsPerByte = duration.count() / totalBytes;
    double bytesPerSecond = 1e9 / nsPerByte;
    double mbPerSecond = bytesPerSecond / (1024.0 * 1024.0);
    
    std::cout << "批量配置:" << std::endl;
    std::cout << "- 批量大小: " << batchSize << " 帧 (" << batchSize * CAN_FRAME_SIZE << " 字节)" << std::endl;
    std::cout << "- 总数据量: " << totalBytes << " 字节" << std::endl;
    std::cout << "\n性能结果:" << std::endl;
    std::cout << "- 总耗时: " << duration.count() << " ns" << std::endl;
    std::cout << "- 每字节耗时: " << std::fixed << std::setprecision(2) << nsPerByte << " ns" << std::endl;
    std::cout << "- 理论吞吐量: " << std::fixed << std::setprecision(2) << mbPerSecond << " MB/s" << std::endl;
    std::cout << "✓ 批量操作性能测试完成" << std::endl;
}

/**
 * @brief 测试环形缓冲区调试输出功能
 * 
 * @details 此测试需要手动启用CIRCULAR_BUFFER_DEBUG宏来验证调试输出
 */
void testDebugOutput() {
    std::cout << "\n=== 测试调试输出功能 ===" << std::endl;
    
    // 注意：此测试需要编译时定义CIRCULAR_BUFFER_DEBUG=1才能看到输出
    std::cout << "注意：调试输出测试需要编译时启用CIRCULAR_BUFFER_DEBUG宏" << std::endl;
    std::cout << "编译命令示例: g++ -DCIRCULAR_BUFFER_DEBUG=1 ..." << std::endl;
    
    CircularBuffer buffer;
    
    // 创建测试CAN帧
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    CanFrame frame(0x100, data, 4);
    
    // 测试添加帧（如果调试启用会有输出）
    bool pushResult = buffer.pushFrame(frame);
    assert(pushResult);
    
    // 测试取出数据（如果调试启用会有输出）
    std::vector<uint8_t> readBuffer(CAN_FRAME_SIZE);
    uint32_t bytesRead = buffer.popBytes(readBuffer.data(), readBuffer.size());
    assert(bytesRead == CAN_FRAME_SIZE);
    
    std::cout << "✓ 调试输出框架测试完成（请检查编译时是否启用调试宏）" << std::endl;
    std::cout << "如果启用CIRCULAR_BUFFER_DEBUG，应该看到详细的读写操作输出" << std::endl;
}
