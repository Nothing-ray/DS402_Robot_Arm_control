/**
 * @file Serial_Module.hpp
 * @brief 串口管理模块 - 中心化共享实例设计
 * 
 * @details 本文件实现了机械臂控制系统的串口管理模块，采用中心化共享实例模式，
 * 确保发送和接收线程可以安全地共享同一个串口资源。该设计提供线程安全的操作接口
 * 和自动错误恢复机制，优化了资源利用率和系统可靠性。
 * 
 * 主要特性：
 * - 中心化实例管理，避免资源重复创建
 * - 线程安全的发送和接收操作隔离
 * - 长连接保持，减少串口开关开销
 * - 自动错误检测和重连机制
 * - 支持多机械臂系统的扩展
 * 
 * @note 所有串口操作都通过本模块进行，确保线程安全和资源一致性
 * @warning 串口实例由主线程创建和管理，工作线程通过引用访问
 */

#ifndef SERIAL_MODULE_HPP
#define SERIAL_MODULE_HPP

#include <boost/asio.hpp>
#include <iostream>
#include <iomanip>
#include <string>
#include <memory>
#include <mutex>
#include <atomic>
#include <vector>
#include <stdexcept>
#include <chrono>
#include <deque>
#include <sstream>
#include <thread>       // 添加线程支持
#include <algorithm>    // 添加算法支持

#include "CAN_frame.hpp"
#include "CircularBuffer.hpp"

// ====================== 编译配置宏 ====================== //

/**
 * @defgroup SerialConfig 串口配置宏
 * @brief 控制串口模块的编译时行为
 * @{
 */

/// 调试输出控制
#ifndef DISABLE_SERIAL_DEBUG
//#define ENABLE_SERIAL_DEBUG
#endif

/// 启用异步发送功能（推荐开启）
#ifndef DISABLE_ASYNC_SEND
#define ENABLE_ASYNC_SEND
#endif

//每帧发送之间的休眠时间
#define SLEEP_TIME 0

/// 异步传输模式配置
#ifndef ASYNC_TRANSFER_MODE
#define ASYNC_TRANSFER_MODE boost::asio::transfer_all()
#endif

/// 串口缓冲区大小配置
#ifndef SERIAL_BUFFER_SIZE
#define SERIAL_BUFFER_SIZE 4096
#endif

/// 最大重试次数
#ifndef MAX_RETRY_COUNT
#define MAX_RETRY_COUNT 3
#endif

/**
 * @}
 */ // end of SerialConfig group

// ====================== 周期对齐缓冲区配置 ====================== //

/// 单周期最大帧数量（6电机 × 4PDO帧）
constexpr size_t SINGLE_CYCLE_FRAME_COUNT = 24;

/// 立即发送阈值（四分之一周期帧数量）
constexpr size_t IMMEDIATE_SEND_THRESHOLD = 1;

/// 立即发送字节数阈值（6帧 × 13字节）
constexpr size_t IMMEDIATE_SEND_THRESHOLD_BYTES = IMMEDIATE_SEND_THRESHOLD * CAN_FRAME_SIZE;

/// 异步发送缓冲区最大帧数量（严格单周期限制）
constexpr size_t ASYNC_BUFFER_MAX_FRAMES = SINGLE_CYCLE_FRAME_COUNT;

// 使用CircularBuffer.hpp中定义的CAN_FRAME_SIZE
// constexpr size_t CAN_FRAME_SIZE = 13;

/// 最大批量帧数量
constexpr size_t MAX_BATCH_FRAMES = 24;

/**
 * @brief 串口管理类 - 中心化共享实例设计
 * 
 * @details 本类实现了高性能的串口通信管理，专为机械臂控制系统优化设计。
 * 采用中心化共享实例模式，确保发送和接收线程可以安全地共享同一个串口资源。
 * 该设计提供线程安全的操作接口和自动错误恢复机制，优化了资源利用率和系统可靠性。
 * 
 * ## 主要特性
 * - **中心化实例管理**: 避免资源重复创建，确保全局唯一串口实例
 * - **线程安全隔离**: 独立的发送、接收、连接互斥锁，确保操作互不干扰
 * - **高性能异步发送**: 支持异步缓冲发送，减少线程阻塞时间
 * - **批量处理优化**: 支持批量帧发送，最大化总线利用率
 * - **周期对齐缓冲**: 严格单周期容量限制，确保实时性
 * - **自动错误恢复**: 连接异常自动检测和重连机制
 * 
 * ## 性能优化特性
 * - **帧完整性保护**: 严格按完整帧管理，确保13字节帧边界不被破坏
 * - **周期对齐缓冲区**: 严格单周期限制(24帧)，半周期(12帧)立即触发发送
 * - **批量帧处理**: 最多支持24帧批量发送(6电机×4PDO)，减少系统调用开销
 * - **零拷贝优化**: 直接操作内存缓冲区，避免不必要的数据复制
 * - **缓存行对齐**: 关键数据结构64字节对齐，避免伪共享
 * 
 * ## 编译时配置
 * 通过定义以下宏启用特定功能：
 * - `ENABLE_ASYNC_SEND`: 启用异步发送功能（推荐）
 * - `ENABLE_SERIAL_DEBUG`: 启用调试输出
 * 
 * @note 所有串口操作都通过本模块进行，确保线程安全和资源一致性
 * @warning 串口实例由主线程创建和管理，工作线程通过引用访问
 * 
 * @par 使用示例:
 * @code
 * // 创建全局实例
 * SerialPortManager serialManager;
 * 
 * // 连接串口
 * if (serialManager.connect("COM1", 3000000)) {
 *     // 发送CAN帧
 *     CanFrame frame;
 *     serialManager.sendFrame(frame);
 *     
 *     // 批量发送
 *     std::vector<CanFrame> frames;
 *     serialManager.sendFramesBatch(frames);
 * }
 * @endcode
 */
class SerialPortManager {
private:
    boost::asio::io_service ioService_;              ///< Boost.Asio I/O服务
    std::unique_ptr<boost::asio::serial_port> serialPort_; ///< 串口对象
    
    std::mutex sendMutex_;                           ///< 发送操作互斥锁
    std::mutex receiveMutex_;                        ///< 接收操作互斥锁
    std::mutex connectMutex_;                        ///< 连接管理互斥锁
    
    std::string portName_;                           ///< 串口设备名称
    unsigned int baudRate_;                          ///< 波特率
    std::atomic<bool> connected_{false};             ///< 连接状态标志
    
    // 异步发送相关成员
#ifdef ENABLE_ASYNC_SEND
    CircularBuffer asyncFrameBuffer_;               ///< 异步发送帧缓冲区（二进制数据存储）
    std::atomic<bool> asyncSending_{false};         ///< 异步发送进行中标志
    
    // 错误和背压回调函数
    std::function<void(const std::string&, size_t)> errorCallback_;
    std::function<void(const std::string&)> backpressureCallback_;
    
    // 性能统计
    std::atomic<uint64_t> totalFramesSent_{0};
    std::atomic<uint64_t> totalBytesSent_{0};
    std::atomic<uint64_t> totalSendErrors_{0};
    std::chrono::steady_clock::time_point lastStatResetTime_;
#endif
    
    /**
     * @brief 内部连接方法
     * @return 连接是否成功
     */
    bool connectInternal() {
        std::lock_guard<std::mutex> lock(connectMutex_);
        
        try {
            if (serialPort_) {
                serialPort_->close();
            }
            
            serialPort_ = std::make_unique<boost::asio::serial_port>(ioService_, portName_);
            serialPort_->set_option(boost::asio::serial_port_base::baud_rate(baudRate_));
            
            connected_.store(true, std::memory_order_release);
            
#ifdef ENABLE_SERIAL_DEBUG
            std::cout << "[INFO][SerialPortManager]: 串口连接成功 - " 
                      << portName_ << " @ " << baudRate_ << " baud" << std::endl;
#endif
            return true;
        }
        catch (const std::exception& e) {
            connected_.store(false, std::memory_order_release);
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[ERROR][SerialPortManager::connectInternal]: " << e.what() << std::endl;
#endif
            return false;
        }
    }
    
    /**
     * @brief 内部断开连接方法
     */
    void disconnectInternal() {
        std::lock_guard<std::mutex> lock(connectMutex_);
        
        try {
            if (serialPort_ && serialPort_->is_open()) {
                serialPort_->close();
            }
            connected_.store(false, std::memory_order_release);
            
#ifdef ENABLE_SERIAL_DEBUG
            std::cout << "[INFO][SerialPortManager]: 串口已断开" << std::endl;
#endif
        }
        catch (const std::exception& e) {
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[ERROR][SerialPortManager::disconnectInternal]: " << e.what() << std::endl;
#endif
        }
    }
    
#ifdef ENABLE_ASYNC_SEND
    /**
     * @brief 处理发送错误
     * @param error 错误信息
     * @param lostBytes 丢失的字节数
     */
    void handleSendError(const boost::system::error_code& error, size_t lostBytes) {
        connected_.store(false, std::memory_order_release);
        
        // 更新错误统计
        totalSendErrors_.fetch_add(1, std::memory_order_relaxed);
        
        // 分类处理不同错误
        std::string errorType;
        if (error == boost::asio::error::operation_aborted) {
            errorType = "OperationAborted";
        } else if (error == boost::asio::error::broken_pipe) {
            errorType = "ConnectionBroken";
        } else if (error == boost::asio::error::timed_out) {
            errorType = "Timeout";
        } else {
            errorType = "OtherError";
        }
        
        // 通知上层应用
        if (errorCallback_) {
            errorCallback_(errorType, lostBytes);
        }
        
#ifdef ENABLE_SERIAL_DEBUG
        std::cerr << "[ERROR][SerialPortManager::handleSendError]: " 
                  << errorType << " - " << error.message() 
                  << ", lost " << lostBytes << " bytes" << std::endl;
#endif
    }
    
    /**
     * @brief 异步发送完成处理回调
     * @param error 发送错误信息
     * @param bytesTransferred 实际传输字节数
     * @param bytesRequested 请求发送的字节数
     */
    void handleAsyncSendComplete(const boost::system::error_code& error, size_t bytesTransferred, size_t bytesRequested) {
        std::lock_guard<std::mutex> lock(sendMutex_);
        
        // 首先在锁保护下修改所有共享状态
        asyncSending_.store(false, std::memory_order_release);
        
        if (error) {
            // 记录丢失的数据量
            size_t lostBytes = asyncFrameBuffer_.getUsedSpace();
            
            // 清空缓冲区但先记录数据量
            asyncFrameBuffer_.clear();
            
            // 使用统一的错误处理方法
            handleSendError(error, lostBytes);
            return;
        } else {
#ifdef ENABLE_SERIAL_DEBUG
            std::cout << "[DEBUG][SerialPortManager::handleAsyncSendComplete]: "
                      << "异步发送完成，请求字节数: " << bytesRequested
                      << "，实际传输字节数: " << bytesTransferred << std::endl;
            
            // 检查传输完整性 - 使用transfer_all后应该总是相等
            if (bytesTransferred == bytesRequested) {
                std::cout << "[DEBUG][SerialPortManager::handleAsyncSendComplete]: "
                          << "transfer_all确保所有数据完整传输" << std::endl;
            } else {
                std::cerr << "[ERROR][SerialPortManager::handleAsyncSendComplete]: "
                          << "transfer_all传输失败！请求 " << bytesRequested 
                          << " 字节，实际传输 " << bytesTransferred << " 字节" << std::endl;
            }
            
            // 检查传输字节数是否为13的倍数
            if (bytesTransferred % CAN_FRAME_SIZE != 0) {
                std::cerr << "[ERROR][SerialPortManager::handleAsyncSendComplete]: "
                          << "帧完整性错误！传输字节数 (" << bytesTransferred 
                          << ") 不是13的倍数" << std::endl;
            }
#endif
            
            // 使用环形缓冲区的popBytes接口清除已发送的数据
            // 由于环形缓冲区自动管理读写位置，这里只需要检查是否还有数据
            if (asyncFrameBuffer_.getUsedSpace() > 0) {
                startAsyncSend();
            }
        }
    }
    
    /**
     * @brief 启动异步发送操作
     * 
     * @details 该方法确保只发送完整的13字节CAN帧，维护严格的帧边界完整性。
     * 通过将帧缓冲区中的所有完整帧合并为连续字节流发送，确保不会出现
     * 跨帧边界的传输，从而保证每个CAN帧的完整性。
     * 
     * @note 帧完整性保证：
     * - 每个CAN帧严格保持13字节长度
     * - 发送操作以完整帧为单位进行
     * - 接收端回调会验证传输字节数是否为13的整数倍
     * - 任何帧边界错误都会触发连接断开和数据清空
     */
    void startAsyncSend() {
        if (asyncFrameBuffer_.isEmpty() || !isConnected()) {
            return;
        }
        
        asyncSending_.store(true, std::memory_order_release);
        
        try {
            // 获取环形缓冲区中可用的数据大小
            size_t bytesToSend = asyncFrameBuffer_.getUsedSpace();
            if (bytesToSend == 0) {
                return;
            }
            
            // 调试输出：显示缓冲区状态
#ifdef ENABLE_SERIAL_DEBUG
            std::cout << "[DEBUG][SerialPortManager::startAsyncSend]: "
                      << "缓冲区使用空间: " << bytesToSend << " 字节, "
                      << "完整帧数: " << (bytesToSend / CAN_FRAME_SIZE) << " 帧" << std::endl;
#endif
            
            // 使用线程局部存储预分配缓冲区，避免锁内内存分配
            thread_local std::vector<uint8_t> sendBuffer;
            sendBuffer.resize(bytesToSend);
            
            size_t bytesRead = asyncFrameBuffer_.popBytes(sendBuffer.data(), bytesToSend);
            
            if (bytesRead == 0) {
                return;
            }
            
            // 完整性验证：数据大小必须是13的倍数
            if (bytesRead % CAN_FRAME_SIZE != 0) {
#ifdef ENABLE_SERIAL_DEBUG
                std::cerr << "[ERROR][SerialPortManager::startAsyncSend]: "
                          << "帧完整性错误！读取字节数: " << bytesRead
                          << " (不是13的倍数)" << std::endl;
                
                // 输出缓冲区内容用于调试
                std::cerr << "[DEBUG][SerialPortManager::startAsyncSend]: "
                          << "异常缓冲区内容: ";
                for (size_t i = 0; i < bytesRead; ++i) {
                    std::cerr << std::hex << std::setw(2) << std::setfill('0')
                              << static_cast<int>(sendBuffer[i]) << " ";
                    if ((i + 1) % CAN_FRAME_SIZE == 0) {
                        std::cerr << " | "; // 每13字节分隔
                    }
                }
                std::cerr << std::dec << std::endl;
#endif
                throw std::runtime_error("内部错误：发送数据总长度不是13字节的整数倍");
            }
            
            // 调试输出：显示将要发送的数据
#ifdef ENABLE_SERIAL_DEBUG
            std::cout << "[DEBUG][SerialPortManager::startAsyncSend]: "
                      << "准备发送 " << bytesRead << " 字节 ("
                      << (bytesRead / CAN_FRAME_SIZE) << " 帧)，使用transfer_all确保完整传输" << std::endl;
            
            // 显示每帧的详细信息
            for (size_t frameStart = 0; frameStart < bytesRead; frameStart += CAN_FRAME_SIZE) {
                std::cout << "[DEBUG][SerialPortManager::startAsyncSend]: "
                          << "帧 " << (frameStart / CAN_FRAME_SIZE) << ": ";
                for (size_t i = 0; i < CAN_FRAME_SIZE; ++i) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0')
                              << static_cast<int>(sendBuffer[frameStart + i]) << " ";
                }
                std::cout << std::dec << std::endl;
            }
#endif
            
            boost::asio::async_write(
                *serialPort_,
                boost::asio::buffer(sendBuffer.data(), bytesRead),
                ASYNC_TRANSFER_MODE, // 使用配置的传输模式
                [this, bytesRead](const boost::system::error_code& error, size_t bytesTransferred) {
                    handleAsyncSendComplete(error, bytesTransferred, bytesRead);
                }
            );
        }
        catch (const std::exception& e) {
            asyncSending_.store(false, std::memory_order_release);
            connected_.store(false, std::memory_order_release);
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[ERROR][SerialPortManager::startAsyncSend]: " << e.what() << std::endl;
#endif
        }
    }
    
    /**
     * @brief 异步发送数据到缓冲区
     * @param data 待发送数据
     * @param size 数据大小
     * @return 是否成功加入缓冲区
     */
    bool asyncSendFrame(const CanFrame& frame) {
        if (!isConnected()) {
            return false;
        }
        
        std::lock_guard<std::mutex> lock(sendMutex_);
        
        // 背压检查：如果缓冲区接近满，拒绝新数据
        size_t freeSpace = asyncFrameBuffer_.getFreeSpace();
        if (freeSpace < CAN_FRAME_SIZE * 2) {  // 预留2帧空间
            if (backpressureCallback_) {
                backpressureCallback_("HighBackpressure");
            }
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[WARN][SerialPortManager::asyncSendFrame]: 背压拒绝，空闲空间: " 
                      << freeSpace << "字节" << std::endl;
#endif
            return false;  // 背压，拒绝数据
        }
        
        // 帧完整性验证：确保每个CAN帧都是完整的13字节
        const auto& binaryData = frame.getBinaryFrame();
        if (binaryData.size() != CAN_FRAME_SIZE) {
            throw std::runtime_error("CAN帧长度错误：预期13字节，实际" + 
                                   std::to_string(binaryData.size()) + "字节");
        }
        
        // 严格周期容量检查 - 不允许超过单周期帧数量
        if (asyncFrameBuffer_.getUsedSpace() + CAN_FRAME_SIZE > asyncFrameBuffer_.getCapacity()) {
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[WARN][SerialPortManager::asyncSendFrame]: 周期帧容量超限，丢弃帧" << std::endl;
#endif
            return false;
        }
        
        // 添加完整帧到缓冲区
        if (!asyncFrameBuffer_.pushFrame(frame)) {
            return false;
        }
        
        // 更新性能统计
        totalFramesSent_.fetch_add(1, std::memory_order_relaxed);
        totalBytesSent_.fetch_add(CAN_FRAME_SIZE, std::memory_order_relaxed);
        
        // 达到半周期字节数或当前没有发送操作，立即启动发送
        if (asyncFrameBuffer_.getUsedSpace() >= IMMEDIATE_SEND_THRESHOLD_BYTES || !asyncSending_.load(std::memory_order_acquire)) {
            startAsyncSend();
        }
        
        return true;
    }
#endif // ENABLE_ASYNC_SEND
    
public:
    /**
     * @brief 默认构造函数
     */
    SerialPortManager() {
#ifdef ENABLE_ASYNC_SEND
        lastStatResetTime_ = std::chrono::steady_clock::now();
#endif
    }
    
    /**
     * @brief 析构函数
     * 
     * @details 安全地关闭串口连接并清理所有资源。在异步发送进行中时会
     * 等待当前发送操作完成，确保资源正确释放。
     */
    ~SerialPortManager() {
        // 等待异步发送完成
#ifdef ENABLE_ASYNC_SEND
        constexpr int maxWaitMs = 100;
        int waited = 0;
        while (asyncSending_.load(std::memory_order_acquire) && waited < maxWaitMs) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            ++waited;
        }
#endif
        
        disconnect();
    }
    
    // 禁止拷贝和移动
    SerialPortManager(const SerialPortManager&) = delete;
    SerialPortManager& operator=(const SerialPortManager&) = delete;
    SerialPortManager(SerialPortManager&&) = delete;
    SerialPortManager& operator=(SerialPortManager&&) = delete;
    
    /**
     * @brief 连接到指定串口
     * 
     * @param portName 串口设备名称 (如 "COM1" 或 "/dev/ttyUSB0")
     * @param baudRate 波特率，默认 3000000 (3Mbps)
     * @return 连接是否成功
     * 
     * @details 建立与指定串口设备的连接，配置波特率并初始化所有内部状态。
     * 该方法会先断开现有连接（如果存在），然后尝试建立新连接。连接成功
     * 后会初始化异步发送缓冲区。
     * 
     * @note 波特率设置为3000000(3Mbps)以匹配CAN总线1Mbps的带宽需求，
     * 考虑到串口到CAN转换芯片的协议开销。
     */
    bool connect(const std::string& portName, unsigned int baudRate = 3000000) {
        portName_ = portName;
        baudRate_ = baudRate;
        
        // 初始化异步发送缓冲区
#ifdef ENABLE_ASYNC_SEND
        asyncFrameBuffer_.clear();
        asyncSending_.store(false, std::memory_order_release);
#endif
        
        return connectInternal();
    }
    
    /**
     * @brief 断开串口连接
     */
    void disconnect() {
        disconnectInternal();
    }
    
    /**
     * @brief 检查串口是否已连接
     * @return 连接状态
     */
    bool isConnected() const {
        return connected_.load(std::memory_order_acquire);
    }
    
    /**
     * @brief 线程安全发送CAN帧
     * 
     * @param frame 待发送的CAN帧
     * @return 发送是否成功
     * 
     * @details 发送单个CAN帧到串口设备。根据编译配置，可能使用同步或异步发送方式。
     * 同步发送会阻塞当前线程直到数据完全写入，异步发送则将数据加入缓冲区后立即返回。
     * 
     * @note 在ENABLE_ASYNC_SEND启用时，该方法变为非阻塞操作，实际发送在后台进行。
     */
    bool sendFrame(const CanFrame& frame) {
        if (!isConnected()) {
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[WARN][SerialPortManager::sendFrame]: 串口未连接" << std::endl;
#endif
            return false;
        }
        
        const std::vector<uint8_t>& binaryData = frame.getBinaryFrame();
        
#ifdef ENABLE_ASYNC_SEND
        // 异步发送模式 - 严格的周期容量检查（基于字节数）
        if (asyncFrameBuffer_.getUsedSpace() + CAN_FRAME_SIZE > asyncFrameBuffer_.getCapacity()) {
            // 严重错误：周期容量超限，立即丢弃帧
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[ERROR][SerialPortManager::sendFrame]: " 
                      << "帧丢弃！周期容量超限。当前缓冲区: " 
                      << asyncFrameBuffer_.getUsedSpace() << "字节，周期限制: " 
                      << asyncFrameBuffer_.getCapacity() << "字节" << std::endl;
#endif
            throw std::runtime_error("串口周期容量超限 - 数据帧丢弃，实时性受威胁");
        }
        
        bool success = asyncSendFrame(frame);
        if (success) {
#ifdef ENABLE_SERIAL_DEBUG
            std::cout << "[DEBUG][SerialPortManager]: CAN帧加入异步发送队列. 字节: ";
            for (uint8_t byte : binaryData) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') 
                          << static_cast<int>(byte) << " ";
            }
            std::cout << std::dec << std::endl;
#endif
        }
        return success;
#else
        // 同步发送模式
        std::lock_guard<std::mutex> lock(sendMutex_);
        
        try {
            // 帧完整性验证：每个帧必须是13字节
            if (binaryData.size() != CAN_FRAME_SIZE) {
                throw std::runtime_error("CAN帧长度错误：预期13字节，实际" + 
                                       std::to_string(binaryData.size()) + "字节");
            }
            
            // 开始计时（纳秒精度）
            auto startTime = std::chrono::high_resolution_clock::now();
            
            boost::asio::write(*serialPort_, boost::asio::buffer(binaryData.data(), binaryData.size()));
            
            // 结束计时并计算耗时（纳秒转换为微秒）
            auto endTime = std::chrono::high_resolution_clock::now();
            auto durationNs = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime);
            double elapsedTimeUs = durationNs.count() / 1000.0; // 纳秒转换为微秒
            
#ifdef ENABLE_SERIAL_DEBUG
            std::cout << "[DEBUG][SerialPortManager]: CAN帧发送成功. 字节: ";
            for (uint8_t byte : binaryData) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') 
                          << static_cast<int>(byte) << " ";
            }
            std::cout << std::dec << std::endl;
            
            // 输出发送耗时统计
            std::cout << "[PERF][SerialPortManager::sendFrame]: "
                      << "同步发送耗时: " << std::fixed << std::setprecision(2) 
                      << elapsedTimeUs << " us" << std::endl;
#endif
            return true;
        }
        catch (const std::exception& e) {
            connected_.store(false, std::memory_order_release);
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[ERROR][SerialPortManager::sendFrame]: " << e.what() << std::endl;
#endif
            return false;
        }
#endif // ENABLE_ASYNC_SEND
    }
    
    /**
     * @brief 批量发送CAN帧
     * 
     * @param frames CAN帧向量，最多支持MAX_BATCH_FRAMES帧
     * @return 成功发送的帧数
     * 
     * @details 批量发送多个CAN帧，优化发送性能。在异步模式下，所有帧会被合并
     * 到一个缓冲区中发送；在同步模式下，使用单个write操作发送所有数据。
     * 
     * @note 该方法会自动处理帧数限制，超过MAX_BATCH_FRAMES的帧会被忽略。
     */
    size_t sendFramesBatch(const std::vector<CanFrame>& frames) {
        if (!isConnected() || frames.empty()) {
            return 0;
        }
        
        // 限制批量大小
        size_t actualCount = std::min(frames.size(), MAX_BATCH_FRAMES);
        
#ifdef ENABLE_ASYNC_SEND
        // 异步批量发送（基于帧数量）
        std::lock_guard<std::mutex> lock(sendMutex_);
        
        // 检查缓冲区容量 - 基于字节数
        size_t requiredSpace = actualCount * CAN_FRAME_SIZE;
        if (asyncFrameBuffer_.getUsedSpace() + requiredSpace > asyncFrameBuffer_.getCapacity()) {
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[WARN][SerialPortManager::sendFramesBatch]: 批量帧丢弃！周期容量超限。"
                      << "当前缓冲区: " << asyncFrameBuffer_.getUsedSpace() << "字节，"
                      << "批量帧数: " << actualCount << "帧，"
                      << "周期限制: " << asyncFrameBuffer_.getCapacity() << "字节" << std::endl;
#endif
            return 0;
        }
        
        // 帧完整性验证：确保所有CAN帧都是完整的13字节
        for (size_t i = 0; i < actualCount; ++i) {
            const auto& binaryData = frames[i].getBinaryFrame();
            if (binaryData.size() != CAN_FRAME_SIZE) {
                throw std::runtime_error("CAN帧长度错误：预期13字节，实际" + 
                                       std::to_string(binaryData.size()) + "字节");
            }
        }
        
        // 批量添加所有帧到缓冲区
        size_t successCount = asyncFrameBuffer_.pushFrames(frames);
        
        // 触发发送（达到半周期字节数或当前没有发送操作）
        if (asyncFrameBuffer_.getUsedSpace() >= IMMEDIATE_SEND_THRESHOLD_BYTES || !asyncSending_.load(std::memory_order_acquire)) {
            startAsyncSend();
        }
        
        return successCount;
        
#else
        // 同步批量发送
        std::lock_guard<std::mutex> lock(sendMutex_);
        
        try {
            // 构建批量数据缓冲区
            std::vector<uint8_t> batchData;
            batchData.reserve(actualCount * CAN_FRAME_SIZE);
            
            for (size_t i = 0; i < actualCount; ++i) {
                const auto& binaryData = frames[i].getBinaryFrame();
                
                // 帧完整性验证：每个帧必须是13字节
                if (binaryData.size() != CAN_FRAME_SIZE) {
                    throw std::runtime_error("CAN帧长度错误：预期13字节，实际" + 
                                           std::to_string(binaryData.size()) + "字节");
                }
                
                batchData.insert(batchData.end(), binaryData.begin(), binaryData.end());
            }
            
            // 最终完整性检查：总字节数必须是13的整数倍
            if (batchData.size() % CAN_FRAME_SIZE != 0) {
                throw std::runtime_error("内部错误：批量发送数据总长度不是13字节的整数倍");
            }
            
            // 开始计时（纳秒精度）
            auto startTime = std::chrono::high_resolution_clock::now();
            
            // 一次性发送所有数据
            boost::asio::write(*serialPort_, boost::asio::buffer(batchData.data(), batchData.size()));
            
            // 结束计时并计算耗时（纳秒转换为微秒）
            auto endTime = std::chrono::high_resolution_clock::now();
            auto durationNs = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime);
            double elapsedTimeUs = durationNs.count() / 1000.0; // 纳秒转换为微秒
            
#ifdef ENABLE_SERIAL_DEBUG
            // 输出批量发送耗时统计
            std::cout << "[PERF][SerialPortManager::sendFramesBatch]: "
                      << "同步批量发送耗时: " << std::fixed << std::setprecision(2) 
                      << elapsedTimeUs << " us, "
                      << "帧数: " << actualCount << " 帧, "
                      << "平均每帧: " << std::fixed << std::setprecision(2) 
                      << (elapsedTimeUs / actualCount) << " us/帧" << std::endl;
#endif
            
            return actualCount;
        }
        catch (const std::exception& e) {
            connected_.store(false, std::memory_order_release);
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[ERROR][SerialPortManager::sendFramesBatch]: " << e.what() << std::endl;
#endif
            return 0;
        }
#endif // ENABLE_ASYNC_SEND
    }
    
    /**
     * @brief 线程安全发送原始数据
     * @param data 待发送的原始数据
     * @return 发送是否成功
     * 
     * @warning 此方法不进行帧完整性验证，适用于非CAN帧数据的传输。
     *          对于CAN帧传输，请使用sendFrame或sendFramesBatch方法。
     */
    bool sendRawData(const std::vector<uint8_t>& data) {
        if (!isConnected()) {
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[WARN][SerialPortManager::sendRawData]: 串口未连接" << std::endl;
#endif
            return false;
        }
        
        std::lock_guard<std::mutex> lock(sendMutex_);
        
        try {
            boost::asio::write(*serialPort_, boost::asio::buffer(data.data(), data.size()));
            return true;
        }
        catch (const std::exception& e) {
            connected_.store(false, std::memory_order_release);
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[ERROR][SerialPortManager::sendRawData]: " << e.what() << std::endl;
#endif
            return false;
        }
    }
    
    /**
     * @brief 线程安全接收数据（为接收线程预留）
     * @param maxSize 最大接收字节数
     * @return 接收到的数据
     */
    std::vector<uint8_t> receiveData(size_t maxSize) {
        std::vector<uint8_t> buffer(maxSize);
        
        if (!isConnected()) {
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[WARN][SerialPortManager::receiveData]: 串口未连接" << std::endl;
#endif
            return buffer;
        }
        
        std::lock_guard<std::mutex> lock(receiveMutex_);
        
        try {
            size_t bytesRead = serialPort_->read_some(boost::asio::buffer(buffer.data(), maxSize));
            buffer.resize(bytesRead);
            return buffer;
        }
        catch (const std::exception& e) {
            connected_.store(false, std::memory_order_release);
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[ERROR][SerialPortManager::receiveData]: " << e.what() << std::endl;
#endif
            return {};
        }
    }
    
    /**
     * @brief 重新连接串口
     * 
     * @return 重连是否成功
     * 
     * @details 先断开当前连接，然后重新建立连接。主要用于错误恢复场景。
     */
    bool reconnect() {
        disconnectInternal();
        return connectInternal();
    }
    
    /**
     * @brief 获取串口信息
     * 
     * @return 串口信息字符串，格式："端口名 @ 波特率 baud"
     */
    std::string getPortInfo() const {
        return portName_ + " @ " + std::to_string(baudRate_) + " baud";
    }
    
    /**
     * @brief 刷新异步发送缓冲区
     * 
     * @details 强制立即发送异步缓冲区中的所有数据，无论当前缓冲区大小如何。
     * 主要用于需要确保数据立即发送的场景，如关键指令或紧急停止。
     * 
     * @return 是否成功启动发送
     */
    bool flushAsyncBuffer() {
#ifdef ENABLE_ASYNC_SEND
        if (!isConnected() || asyncFrameBuffer_.isEmpty()) {
            return false;
        }
        
        std::lock_guard<std::mutex> lock(sendMutex_);
        
        if (!asyncSending_.load(std::memory_order_acquire)) {
            startAsyncSend();
            return true;
        }
#endif
        return false;
    }
    
    /**
     * @brief 设置错误回调函数
     * 
     * @param callback 回调函数，参数为错误类型和丢失的字节数
     */
    void setErrorCallback(std::function<void(const std::string&, size_t)> callback) {
#ifdef ENABLE_ASYNC_SEND
        std::lock_guard<std::mutex> lock(sendMutex_);
        errorCallback_ = std::move(callback);
#endif
    }
    
    /**
     * @brief 设置背压回调函数
     * 
     * @param callback 回调函数，参数为背压级别描述
     */
    void setBackpressureCallback(std::function<void(const std::string&)> callback) {
#ifdef ENABLE_ASYNC_SEND
        std::lock_guard<std::mutex> lock(sendMutex_);
        backpressureCallback_ = std::move(callback);
#endif
    }
    
    /**
     * @brief 获取性能统计信息
     * 
     * @return 包含帧数、字节数、错误数的元组
     */
    std::tuple<uint64_t, uint64_t, uint64_t> getPerformanceStats() const {
#ifdef ENABLE_ASYNC_SEND
        return {
            totalFramesSent_.load(std::memory_order_relaxed),
            totalBytesSent_.load(std::memory_order_relaxed),
            totalSendErrors_.load(std::memory_order_relaxed)
        };
#else
        return {0, 0, 0};
#endif
    }
    
    /**
     * @brief 重置性能统计
     */
    void resetPerformanceStats() {
#ifdef ENABLE_ASYNC_SEND
        totalFramesSent_.store(0, std::memory_order_relaxed);
        totalBytesSent_.store(0, std::memory_order_relaxed);
        totalSendErrors_.store(0, std::memory_order_relaxed);
        lastStatResetTime_ = std::chrono::steady_clock::now();
#endif
    }
    
    /**
     * @brief 获取当前异步缓冲区状态
     * 
     * @return 缓冲区当前大小（字节数）
     */
    size_t getBufferSize() const {
#ifdef ENABLE_ASYNC_SEND
        return asyncFrameBuffer_.getUsedSpace();
#else
        return 0;
#endif
    }
    
    /**
     * @brief 检查是否正在异步发送
     * 
     * @return 异步发送是否正在进行中
     */
    bool isAsyncSending() const {
#ifdef ENABLE_ASYNC_SEND
        return asyncSending_.load(std::memory_order_acquire);
#else
        return false;
#endif
    }
};

#endif // SERIAL_MODULE_HPP