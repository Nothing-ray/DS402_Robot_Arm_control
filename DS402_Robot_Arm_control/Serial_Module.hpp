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


//每帧发送之间的休眠时间
#define SLEEP_TIME 0


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
constexpr size_t IMMEDIATE_SEND_THRESHOLD = 4;

/// 立即发送字节数阈值（6帧 × 13字节）
constexpr size_t IMMEDIATE_SEND_THRESHOLD_BYTES = IMMEDIATE_SEND_THRESHOLD * CAN_FRAME_SIZE;


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
 * - **高性能同步发送**: 支持同步批量发送，最大化总线利用率
 * - **批量处理优化**: 支持批量帧发送，减少系统调用开销
 * - **自动错误恢复**: 连接异常自动检测和重连机制
 * 
 * ## 性能优化特性
 * - **帧完整性保护**: 严格按完整帧管理，确保13字节帧边界不被破坏
 * - **批量帧处理**: 最多支持24帧批量发送(6电机×4PDO)，减少系统调用开销
 * - **零拷贝优化**: 直接操作内存缓冲区，避免不必要的数据复制
 * - **缓存行对齐**: 关键数据结构64字节对齐，避免伪共享
 * 
 * ## 编译时配置
 * 通过定义以下宏启用特定功能：
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
    
    // 预分配发送缓冲区（64字节对齐，1024字节大小）
    alignas(64) std::array<uint8_t, 1024> sendBuffer_;
    std::mutex bufferMutex_;                         ///< 缓冲区操作互斥锁
    
    
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
    
    
public:
    /**
     * @brief 默认构造函数
     */
    SerialPortManager() {
        // 初始化预分配缓冲区
        sendBuffer_.fill(0);
    }
    
    /**
     * @brief 析构函数
     * 
     * @details 安全地关闭串口连接并清理所有资源。
     */
    ~SerialPortManager() {
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
     * @details 发送单个CAN帧到串口设备。使用同步发送方式。
     * 阻塞当前线程直到数据完全写入。
     * 
     * @note 该方法使用同步发送方式，确保数据立即发送到串口。
     */
    bool sendFrame(const CanFrame& frame) {
        if (!isConnected()) {
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[WARN][SerialPortManager::sendFrame]: 串口未连接" << std::endl;
#endif
            return false;
        }
        
        const std::vector<uint8_t>& binaryData = frame.getBinaryFrame();
        
        // 同步发送模式
        
        
        try {
            // 帧完整性验证：每个帧必须是13字节
            if (binaryData.size() != CAN_FRAME_SIZE) {
                throw std::runtime_error("CAN帧长度错误：预期13字节，实际" + 
                                       std::to_string(binaryData.size()) + "字节");
            }
#ifdef ENABLE_SERIAL_DEBUG
            // 开始计时（纳秒精度）
            auto startTime = std::chrono::high_resolution_clock::now();
            
#endif

            {

                std::lock_guard<std::mutex> lock(sendMutex_);

                //发送数据
                boost::asio::write(*serialPort_, boost::asio::buffer(binaryData.data(), binaryData.size()));


            }

#ifdef ENABLE_SERIAL_DEBUG
            // 结束计时并计算耗时（纳秒转换为微秒）
            auto endTime = std::chrono::high_resolution_clock::now();
            auto durationNs = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime);
            double elapsedTimeUs = durationNs.count() / 1000.0; // 纳秒转换为微秒
            

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
     * @brief 同步批量发送环形缓冲区数据
     * 
     * @details 从环形缓冲区发送数据，以13字节为单位进行同步发送。
     * 该方法提供高性能的同步批量发送功能，适用于需要精确控制发送时机的场景。
     * 使用线程局部存储避免锁内内存分配，优化性能。
     * 
     * @param buffer 环形缓冲区引用，包含待发送的CAN帧数据
     * @param sendsize 指定发送字节数，-1表示发送所有可用完整帧
     * @param clearAfterSend 发送后是否清空缓冲区（默认false）
     * @return 成功发送的字节数（13的倍数）
     * 
     * @note 该方法确保只发送完整的CAN帧，维护严格的帧边界完整性。
     * @warning 如果指定sendsize不是13的倍数，会自动向下取整到最近的13字节倍数
     * @warning sendsize <= 0（除-1外）时不会发送任何数据
     */
    inline size_t sendBufferSync(CircularBuffer& buffer, int sendsize = -1, bool clearAfterSend = false) {
        if (!isConnected() || buffer.isEmpty()) {
            return 0;
        }
        
        // 第一阶段：无锁计算发送大小
        size_t sendSize;
        if (sendsize == -1) {
            // sendsize为-1时，发送所有可用完整帧，但不超过缓冲区大小
            sendSize = std::min(buffer.getAvailableFrames() * CAN_FRAME_SIZE, sendBuffer_.size());
        } else if (sendsize > 0) {
            // sendsize大于0时，按指定字节数发送，但确保是13的倍数且不超过缓冲区
            sendSize = std::min(static_cast<size_t>((sendsize / CAN_FRAME_SIZE) * CAN_FRAME_SIZE), 
                               sendBuffer_.size());
            // 限制发送大小不超过缓冲区可用空间
            sendSize = std::min(sendSize, static_cast<size_t>(buffer.getUsedSpace()));
        } else {
            // sendsize <= 0（除-1外）时，不发送任何数据
            return 0;
        }

        if (sendSize == 0) return 0;
        
        // 第二阶段：最小化锁范围 - 仅保护缓冲区填充操作
        size_t bytesRead;
        {
            std::lock_guard<std::mutex> lock(bufferMutex_);
            bytesRead = buffer.popBytes(sendBuffer_.data(), sendSize);
        }
        
        if (bytesRead == 0) {
            return 0;
        }
        
        // 确保只发送完整帧（13字节倍数）
        size_t bytesToSend = (bytesRead / CAN_FRAME_SIZE) * CAN_FRAME_SIZE;
        if (bytesToSend == 0) {
            return 0;
        }
        
        try {
            // 帧完整性验证：确保发送的数据是13的倍数
            if (bytesToSend % CAN_FRAME_SIZE != 0) {
                throw std::runtime_error("内部错误：发送数据总长度不是13字节的整数倍");
            }

#ifdef ENABLE_SERIAL_DEBUG


            // 开始计时（纳秒精度）
            auto startTime = std::chrono::high_resolution_clock::now();
#endif


            {
                std::lock_guard<std::mutex> lock(sendMutex_);
                // 同步发送所有数据
                boost::asio::write(*serialPort_, boost::asio::buffer(sendBuffer_.data(), bytesToSend));
            }
#ifdef ENABLE_SERIAL_DEBUG
            // 结束计时并计算耗时（纳秒转换为微秒）
            auto endTime = std::chrono::high_resolution_clock::now();
            auto durationNs = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime);
            double elapsedTimeUs = durationNs.count() / 1000.0; // 纳秒转换为微秒
            

            std::cout << "[DEBUG][SerialPortManager::sendBufferSync]: "
                      << "同步批量发送完成，请求字节数: " << sendsize
                      << "，实际发送字节数: " << bytesToSend
                      << "，帧数: " << (bytesToSend / CAN_FRAME_SIZE)
                      << "，耗时: " << std::fixed << std::setprecision(2) 
                      << elapsedTimeUs << " us" << std::endl;
            
            // 显示每帧的详细信息
            for (size_t frameStart = 0; frameStart < bytesToSend; frameStart += CAN_FRAME_SIZE) {
                std::cout << "[DEBUG][SerialPortManager::sendBufferSync]: "
                          << "帧 " << (frameStart / CAN_FRAME_SIZE) << ": ";
                for (size_t i = 0; i < CAN_FRAME_SIZE; ++i) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0')
                              << static_cast<int>(sendBuffer_[frameStart + i]) << " ";
                }
                std::cout << std::dec << std::endl;
            }
#endif
            
            // 如果需要清空缓冲区
            if (clearAfterSend) {
                buffer.clear();
            }
            
            return bytesToSend;
        }
        catch (const std::exception& e) {
            connected_.store(false, std::memory_order_release);
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[ERROR][SerialPortManager::sendBufferSync]: " << e.what() << std::endl;
#endif
            return 0;
        }
    }
    
    /**
     * @brief 自动同步循环发送环形缓冲区数据
     * 
     * @details 自动清空环形缓冲区中的所有完整CAN帧数据，采用两阶段策略：
     * 1. 一次性从环形缓冲区读取所有数据到发送缓冲区
     * 2. 循环方式每次发送13字节（1帧），直到发送缓冲区清空
     * 该方法专为"一键清空"场景设计，确保严格的帧边界完整性。
     * 
     * @param buffer 环形缓冲区引用，包含待发送的CAN帧数据
     * @param clearAfterSend 发送后是否清空缓冲区（默认true）
     * @return 成功发送的字节数（13的倍数）
     * 
     * @note 该方法采用两阶段策略，减少环形缓冲区锁定时间，提高并发性能。
     * @warning 只发送完整的CAN帧，确保严格的帧边界完整性。
     */
    inline size_t sendBufferSyncAuto(CircularBuffer& buffer, bool clearAfterSend = true) {
        if (!isConnected() || buffer.isEmpty()) {
            return 0;
        }
        
        // 第一阶段：一次性读取所有数据到发送缓冲区
        size_t totalBytesRead;
        {
            std::lock_guard<std::mutex> lock(bufferMutex_);
            totalBytesRead = buffer.popBytes(sendBuffer_.data(), sendBuffer_.size());
        }
        
        if (totalBytesRead == 0) {
            return 0;
        }
        
        // 确保只处理完整帧（13字节倍数）
        size_t totalBytesToSend = (totalBytesRead / CAN_FRAME_SIZE) * CAN_FRAME_SIZE;
        if (totalBytesToSend == 0) {
            return 0;
        }
        
        size_t totalBytesSent = 0;
        
        try {
#ifdef ENABLE_SERIAL_DEBUG
            // 开始计时（纳秒精度）
            auto startTime = std::chrono::high_resolution_clock::now();
            std::cout << "[DEBUG][SerialPortManager::sendBufferSyncAuto]: "
                      << "开始循环发送，总字节数: " << totalBytesToSend
                      << "，总帧数: " << (totalBytesToSend / CAN_FRAME_SIZE) << std::endl;
#endif

            // 第二阶段：循环发送每一帧
            for (size_t offset = 0; offset < totalBytesToSend; offset += CAN_FRAME_SIZE) {
                // 帧完整性验证
                if (offset + CAN_FRAME_SIZE > totalBytesToSend) {
                    break; // 确保不越界
                }
                
                // 发送单帧数据
                {
                    std::lock_guard<std::mutex> lock(sendMutex_);
                    boost::asio::write(*serialPort_, 
                        boost::asio::buffer(sendBuffer_.data() + offset, CAN_FRAME_SIZE));
                }
                
                totalBytesSent += CAN_FRAME_SIZE;
                
#ifdef ENABLE_SERIAL_DEBUG
                std::cout << "[DEBUG][SerialPortManager::sendBufferSyncAuto]: "
                          << "帧 " << (offset / CAN_FRAME_SIZE) << ": ";
                for (size_t i = 0; i < CAN_FRAME_SIZE; ++i) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0')
                              << static_cast<int>(sendBuffer_[offset + i]) << " ";
                }
                std::cout << std::dec << std::endl;
#endif
            }

#ifdef ENABLE_SERIAL_DEBUG
            // 结束计时并计算耗时（纳秒转换为微秒）
            auto endTime = std::chrono::high_resolution_clock::now();
            auto durationNs = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime);
            double elapsedTimeUs = durationNs.count() / 1000.0; // 纳秒转换为微秒
            
            std::cout << "[DEBUG][SerialPortManager::sendBufferSyncAuto]: "
                      << "循环发送完成，总发送字节数: " << totalBytesSent
                      << "，总帧数: " << (totalBytesSent / CAN_FRAME_SIZE)
                      << "，总耗时: " << std::fixed << std::setprecision(2) 
                      << elapsedTimeUs << " us" << std::endl;
            
            if (totalBytesSent > 0) {
                double avgTimePerFrame = elapsedTimeUs / (totalBytesSent / CAN_FRAME_SIZE);
                std::cout << "[PERF][SerialPortManager::sendBufferSyncAuto]: "
                          << "平均每帧耗时: " << std::fixed << std::setprecision(2) 
                          << avgTimePerFrame << " us/帧" << std::endl;
            }
#endif
            
            // 如果需要清空缓冲区（确保完全清空）
            if (clearAfterSend) {
                buffer.clear();
            }
            
            return totalBytesSent;
        }
        catch (const std::exception& e) {
            connected_.store(false, std::memory_order_release);
#ifdef ENABLE_SERIAL_DEBUG
            std::cerr << "[ERROR][SerialPortManager::sendBufferSyncAuto]: " << e.what() << std::endl;
#endif
            return totalBytesSent; // 返回已成功发送的字节数
        }
    }
    
    
};

#endif // SERIAL_MODULE_HPP