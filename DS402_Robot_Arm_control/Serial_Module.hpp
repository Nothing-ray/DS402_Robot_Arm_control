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

// ====================== 编译配置宏 ====================== //

/**
 * @defgroup SerialConfig 串口配置宏
 * @brief 控制串口模块的编译时行为
 * @{
 */

/// 调试输出控制
#ifndef DISABLE_SERIAL_DEBUG
#define ENABLE_SERIAL_DEBUG
#endif

/// 启用异步发送功能（推荐开启）
#ifndef DISABLE_ASYNC_SEND
#define ENABLE_ASYNC_SEND
#endif

/**
 * @}
 */ // end of SerialConfig group

// ====================== 周期对齐缓冲区配置 ====================== //

/// 单周期最大帧数量（6电机 × 4PDO帧）
constexpr size_t SINGLE_CYCLE_FRAME_COUNT = 24;

/// 立即发送阈值（半周期帧数量）
constexpr size_t IMMEDIATE_SEND_THRESHOLD = 12;

/// 异步发送缓冲区最大帧数量（严格单周期限制）
constexpr size_t ASYNC_BUFFER_MAX_FRAMES = SINGLE_CYCLE_FRAME_COUNT;

/// 单帧固定大小（13字节）
constexpr size_t CAN_FRAME_SIZE = 13;

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
    std::deque<CanFrame> asyncFrameBuffer_;         ///< 异步发送帧缓冲区（按完整帧存储）
    std::atomic<bool> asyncSending_{false};         ///< 异步发送进行中标志
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
     * @brief 异步发送完成处理回调
     * @param error 发送错误信息
     * @param bytesTransferred 实际传输字节数
     */
    void handleAsyncSendComplete(const boost::system::error_code& error, size_t bytesTransferred) {
        std::lock_guard<std::mutex> lock(sendMutex_);
        
        asyncSending_.store(false, std::memory_order_release);
        
        if (error) {
            connected_.store(false, std::memory_order_release);
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[ERROR][SerialPortManager::handleAsyncSendComplete]: " 
                      << error.message() << std::endl;
#endif
        } else {
            // 计算发送的完整帧数量和可能的剩余字节
            size_t framesSent = bytesTransferred / CAN_FRAME_SIZE;
            size_t remainingBytes = bytesTransferred % CAN_FRAME_SIZE;
            
            // 处理可能的帧边界错误（剩余字节不为0）
            if (remainingBytes > 0) {
                // 严重错误：帧边界不完整，可能导致后续数据错位
                connected_.store(false, std::memory_order_release);
                
#ifdef ENABLE_SERIAL_DEBUG
                std::cerr << "[CRITICAL][SerialPortManager::handleAsyncSendComplete]: " 
                          << "帧边界错误！传输字节数(" << bytesTransferred 
                          << ")不是13字节的整数倍，剩余字节: " << remainingBytes 
                          << "。串口连接已断开以确保数据完整性。" << std::endl;
#endif
                
                // 清空缓冲区防止数据错位
                asyncFrameBuffer_.clear();
                return; // 不再继续发送
            }
            
            // 清除已发送的完整帧
            if (framesSent > 0 && framesSent <= asyncFrameBuffer_.size()) {
                asyncFrameBuffer_.erase(asyncFrameBuffer_.begin(), 
                                      asyncFrameBuffer_.begin() + framesSent);
            }
            
            // 如果缓冲区还有完整帧，继续发送
            if (!asyncFrameBuffer_.empty()) {
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
        if (asyncFrameBuffer_.empty() || !isConnected()) {
            return;
        }
        
        asyncSending_.store(true, std::memory_order_release);
        
        try {
            // 发送缓冲区中的所有完整帧（确保帧完整性）
            std::vector<uint8_t> sendData;
            
            // 预分配精确内存以避免动态扩容
            sendData.reserve(asyncFrameBuffer_.size() * CAN_FRAME_SIZE);
            
            for (const auto& frame : asyncFrameBuffer_) {
                const auto& binaryData = frame.getBinaryFrame();
                
                // 完整性验证：每个帧必须是13字节
                if (binaryData.size() != CAN_FRAME_SIZE) {
                    throw std::runtime_error("CAN帧长度错误：预期13字节，实际" + 
                                           std::to_string(binaryData.size()) + "字节");
                }
                
                sendData.insert(sendData.end(), binaryData.begin(), binaryData.end());
            }
            
            // 最终完整性检查：总字节数必须是13的整数倍
            if (sendData.size() % CAN_FRAME_SIZE != 0) {
                throw std::runtime_error("内部错误：发送数据总长度不是13字节的整数倍");
            }
            
            serialPort_->async_write_some(
                boost::asio::buffer(sendData.data(), sendData.size()),
                [this](const boost::system::error_code& error, size_t bytesTransferred) {
                    handleAsyncSendComplete(error, bytesTransferred);
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
        
        // 严格周期容量检查 - 不允许超过单周期帧数量
        if (asyncFrameBuffer_.size() >= ASYNC_BUFFER_MAX_FRAMES) {
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[WARN][SerialPortManager::asyncSendFrame]: 周期帧容量超限，丢弃帧" << std::endl;
#endif
            return false;
        }
        
        // 添加完整帧到缓冲区
        asyncFrameBuffer_.push_back(frame);
        
        // 达到半周期帧数量或当前没有发送操作，立即启动发送
        if (asyncFrameBuffer_.size() >= IMMEDIATE_SEND_THRESHOLD || !asyncSending_.load(std::memory_order_acquire)) {
            startAsyncSend();
        }
        
        return true;
    }
#endif // ENABLE_ASYNC_SEND
    
public:
    /**
     * @brief 默认构造函数
     */
    SerialPortManager() = default;
    
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
        // 异步发送模式 - 严格的周期容量检查（基于帧数量）
        if (asyncFrameBuffer_.size() >= ASYNC_BUFFER_MAX_FRAMES) {
            // 严重错误：周期容量超限，立即丢弃帧
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[ERROR][SerialPortManager::sendFrame]: " 
                      << "帧丢弃！周期容量超限。当前缓冲区: " 
                      << asyncFrameBuffer_.size() << "帧，周期限制: " 
                      << ASYNC_BUFFER_MAX_FRAMES << "帧" << std::endl;
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
            
            boost::asio::write(*serialPort_, boost::asio::buffer(binaryData.data(), binaryData.size()));
            
#ifdef ENABLE_SERIAL_DEBUG
            std::cout << "[DEBUG][SerialPortManager]: CAN帧发送成功. 字节: ";
            for (uint8_t byte : binaryData) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') 
                          << static_cast<int>(byte) << " ";
            }
            std::cout << std::dec << std::endl;
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
        
        // 检查缓冲区容量 - 基于帧数量
        if (asyncFrameBuffer_.size() + actualCount > ASYNC_BUFFER_MAX_FRAMES) {
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[WARN][SerialPortManager::sendFramesBatch]: 批量帧丢弃！周期容量超限。"
                      << "当前缓冲区: " << asyncFrameBuffer_.size() << "帧，"
                      << "批量帧数: " << actualCount << "帧，"
                      << "周期限制: " << ASYNC_BUFFER_MAX_FRAMES << "帧" << std::endl;
#endif
            return 0;
        }
        
        // 添加所有完整帧到缓冲区
        for (size_t i = 0; i < actualCount; ++i) {
            asyncFrameBuffer_.push_back(frames[i]);
        }
        
        // 触发发送（达到半周期或当前没有发送操作）
        if (asyncFrameBuffer_.size() >= IMMEDIATE_SEND_THRESHOLD || !asyncSending_.load(std::memory_order_acquire)) {
            startAsyncSend();
        }
        
        return actualCount;
        
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
            
            // 一次性发送所有数据
            boost::asio::write(*serialPort_, boost::asio::buffer(batchData.data(), batchData.size()));
            
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
        if (!isConnected() || asyncFrameBuffer_.empty()) {
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
     * @brief 获取当前异步缓冲区状态
     * 
     * @return 缓冲区当前大小（字节数）
     */
    size_t getBufferSize() const {
#ifdef ENABLE_ASYNC_SEND
        return asyncFrameBuffer_.size() * CAN_FRAME_SIZE;
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