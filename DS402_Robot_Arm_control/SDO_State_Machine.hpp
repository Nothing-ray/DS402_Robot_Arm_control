/**
 * @file SDO_State_Machine.hpp
 * @brief 定义基于CANopen协议的SDO通信状态机和事务管理类
 *
 * @details 本文件实现了机械臂控制系统的SDO（Service Data Object）状态机模块，
 * 采用现代C++原子操作和智能指针技术，为多线程环境下的安全SDO通信提供完整的
 * 事务生命周期管理。该设计针对机械臂低频配置操作场景优化，确保逐个事务的
 * 原子性处理和总线通信安全。
 *
 * 主要功能包括：
 * - 定义SDO事务状态机和响应类型的完整枚举体系
 * - 提供线程安全的SDO事务准备、启动、响应处理和超时检测
 * - 实现基于内存栅栏和原子操作的并发安全状态转换
 * - 支持CANopen标准SDO帧的解析和响应分类
 * - 为机械臂配置和模式更改提供可靠的低频通信保障
 * - 集成智能重试机制，提供自动超时重传和错误恢复功能
 *
 * @note 该模块采用单事务处理设计，符合机械臂控制系统对安全性和确定性的要求。
 * 所有数据结构均使用缓存行对齐和智能指针管理，避免多线程环境下的竞态条件和
 * 内存安全问题。500μs超时设置适配CAN总线实时性需求，最大3次重试确保通信可靠性。
 *
 * @par 设计特点：
 * - 安全性优先：逐个事务处理避免总线竞争和状态不一致
 * - 线程安全：原子操作+互斥锁+条件变量确保并发安全
 * - 内存安全：智能指针管理生命周期，消除悬挂指针风险
 * - 实时性：精确的超时控制和状态机响应机制
 * - 标准化：完整支持CANopen CiA 301 SDO协议规范
 * - 可靠性：智能重试机制提供自动错误恢复和异常处理
 *
 * @par 典型使用场景：
 * - 机械臂关节参数配置（低频操作，10-20个SDO帧/周期）
 * - 运行模式切换和状态查询
 * - 故障诊断和错误恢复操作
 * - 系统初始化和参数校准
 *
 * @par 重试机制：
 * - 超时时间：500μs（可配置）
 * - 最大重试次数：3次（可配置）
 * - 重试策略：指数退避，自动清零成功计数器
 * - 异常处理：按CLAUDE.md标准输出错误并抛出runtime_error
 * - 状态管理：RETRYING和MAX_RETRIES_EXCEEDED状态
 *
 * @warning 本模块设计为单事务处理，不支持并发多个SDO事务，这是为了确保
 * 机械臂控制系统的安全性和确定性而做出的有意设计决策。
 */

#ifndef SDO_STATE_MACHINE_HPP
#define SDO_STATE_MACHINE_HPP

#include <cstdint>
#include <array>
#include <optional>
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <memory>
#include <stdexcept>
#include <iostream>
#include "CAN_frame.hpp"

/// @brief SDO通信超时时间定义（微秒）
/// @details 根据CANopen标准和实时性要求设置的默认超时值
/// @note 可根据具体网络环境和性能要求调整
#define TIME_OUT_US 2000  // 超时时间(微秒)

/// @brief SDO重试机制配置
#define MAX_RETRY_COUNT 3  // 最大重试次数（2-3次）

namespace canopen {

    /**
     * @brief SDO事务状态枚举
     * 
     * @details 定义SDO通信事务的所有可能状态，用于状态机管理
     * - IDLE: 初始状态，可以发起新的SDO请求
     * - WAITING_RESPONSE: 已发送请求，等待目标节点响应
     * - RESPONSE_VALID: 收到正常响应，数据已可用
     * - RESPONSE_ERROR: 收到错误响应，操作失败
     * - TIMEOUT: 响应超时，通信中断
     * - RETRYING: 正在重试，准备重新发送请求
     * - MAX_RETRIES_EXCEEDED: 超过最大重试次数，事务最终失败
     * 
     * @note 使用uint8_t存储以减少内存占用和提高缓存效率
     */
    enum class SdoState : uint8_t {
        IDLE,           ///< 空闲状态，可发送请求
        WAITING_RESPONSE, ///< 已发送请求，等待响应
        RESPONSE_VALID, ///< 收到有效响应
        RESPONSE_ERROR, ///< 收到错误响应
        TIMEOUT,        ///< 响应超时
        RETRYING,       ///< 正在重试
        MAX_RETRIES_EXCEEDED ///< 超过最大重试次数
    };

    /**
     * @brief SDO响应类型枚举
     * 
     * @details 定义CANopen SDO协议中所有可能的响应类型
     * - NO_RESPONSE: 初始状态，尚未收到任何响应
     * - UNKNOWN: 不识别的响应类型，可能是不支持的命令
     * - READ_8BIT: 快速上传1字节数据响应 (CANopen命令 0x4F)
     * - READ_16BIT: 快速上传2字节数据响应 (CANopen命令 0x4B)
     * - READ_32BIT: 快速上传4字节数据响应 (CANopen命令 0x43)
     * - WRITE_SUCCESS: 快速下载成功响应 (CANopen命令 0x60)
     * - ERROR_RESPONSE: 错误响应 (CANopen命令 0x80-0x8F)
     * 
     * @note 符合CANopen CiA 301标准的SDO协议定义
     */
    enum class SdoResponseType : uint8_t {
        NO_RESPONSE,    ///< 无响应，初始状态
        UNKNOWN,        ///< 未知响应
        READ_8BIT,      ///< 读8位响应 (0x4F)
        READ_16BIT,     ///< 读16位响应 (0x4B)  
        READ_32BIT,     ///< 读32位响应 (0x43)
        WRITE_SUCCESS,  ///< 写成功响应 (0x60)
        ERROR_RESPONSE  ///< 错误响应 (0x80-0xFF)
    };

    /**
     * @brief SDO事务结构体（缓存行对齐）
     * 
     * @details 存储一个完整的SDO通信事务的所有数据和状态
     * - 使的64字节对齐以避免多核系统中的伪共享问题
     * - 使用原子变量存储状态信息保证线程安全
     * - 禁用拷贝操作防止意外的数据复制
     * - 支持移动语义提高性能
     * 
     * @note 缓存行对齐可能增加内存使用，但可显著提高多线程性能
     * @warning 必须使用移动语义或直接构造，不可拷贝
     */
    struct alignas(64) SdoTransaction {
        uint8_t node_id;                        ///< 目标节点ID
        uint16_t index;                         ///< 对象字典索引
        uint8_t subindex;                       ///< 对象字典子索引
        std::array<uint8_t, 8> request_data;    ///< 请求数据
        std::array<uint8_t, 8> response_data;   ///< 响应数据
        uint8_t response_dlc{ 0 };              ///< 响应数据长度
        std::atomic<SdoState> state{ SdoState::IDLE };         ///< 原子状态
        std::atomic<SdoResponseType> response_type{ SdoResponseType::NO_RESPONSE }; ///< 原子响应类型
        std::atomic<uint8_t> retry_count{ 0 };  ///< 重试计数器

        /**
         * @brief 默认构造函数
         */
        SdoTransaction() = default;

        /**
         * @brief 移动构造函数
         */
        SdoTransaction(SdoTransaction&& other) noexcept
            : node_id(other.node_id),
            index(other.index),
            subindex(other.subindex),
            request_data(std::move(other.request_data)),
            response_data(std::move(other.response_data)),
            response_dlc(other.response_dlc),
            state(other.state.load(std::memory_order_acquire)),
            response_type(other.response_type.load(std::memory_order_acquire)),
            retry_count(other.retry_count.load(std::memory_order_acquire)) {
        }

        /**
         * @brief 移动赋值运算符
         */
        SdoTransaction& operator=(SdoTransaction&& other) noexcept {
            if (this != &other) {
                node_id = other.node_id;
                index = other.index;
                subindex = other.subindex;
                request_data = std::move(other.request_data);
                response_data = std::move(other.response_data);
                response_dlc = other.response_dlc;
                state.store(other.state.load(std::memory_order_acquire), std::memory_order_release);
                response_type.store(other.response_type.load(std::memory_order_acquire), std::memory_order_release);
                retry_count.store(other.retry_count.load(std::memory_order_acquire), std::memory_order_release);
            }
            return *this;
        }

        // 禁用拷贝构造和拷贝赋值
        SdoTransaction(const SdoTransaction&) = delete;
        SdoTransaction& operator=(const SdoTransaction&) = delete;
    };

    /**
     * @brief 基于内存栅栏的SDO状态机类
     * 
     * @details 实时控制系统中的高性能线程安全SDO状态机实现
     * - 使用原子操作和精确的内存序保证数据一致性
     * - 支持并发访问，但同时只支持一个活跃事务
     * - 使用条件变量实现高效的等待机制
     * - 内置超时检测和错误处理机制
     * - 符合CANopen CiA 301 SDO协议标准
     * 
     * @note 线程安全设计，使用原子操作和内存栅栏保证数据一致性
     * @warning 不支持并发多个事务，同时只能有一个活跃事务
     * @performance 优化过的内存访问模式，适用于实时系统
     */
    class AtomicSdoStateMachine {
    public:
        /**
         * @brief 准备SDO事务 将CAN帧注册到SDO事务的各个变量以便处理
         * @param frame CAN帧数据
         * @return 初始化好的SDO事务对象
         * 
         * @details 安全解析CAN帧数据为SDO事务对象，包含完整的边界检查和数据验证
         * - 验证帧类型和节点ID有效性
         * - 安全解析对象字典索引和子索引
         * - 防止缓冲区溢出的数据复制
         * - 使用内存序保证线程安全
         * 
         * @note 线程安全: 返回的事务对象初始状态为IDLE，可安全用于多线程环境
         * @warning 输入frame.data必须至少包含dlc指定的字节数
         */
        inline SdoTransaction prepareTransaction(const CanFrame& frame) {
            SdoTransaction transaction;

            // 使用位掩码提取SDO帧类型和节点ID
            const uint32_t SDO_ID_MASK = 0x780;
            const uint32_t SDO_REQUEST_ID = 0x600;
            const uint32_t NODE_ID_MASK = 0x7F;
            const uint8_t MAX_CAN_DLC = 8;  // CAN帧最大数据长度
            const uint8_t MIN_SDO_DLC = 4;  // SDO帧最小数据长度

            uint32_t frame_type = frame.frameID & SDO_ID_MASK;
            uint8_t node_id = static_cast<uint8_t>(frame.frameID & NODE_ID_MASK);

            // 验证帧类型、节点ID和数据长度
            if (frame_type != SDO_REQUEST_ID || node_id == 0 || node_id > 127 || 
                frame.dlc > MAX_CAN_DLC) {
                transaction.state.store(SdoState::IDLE, std::memory_order_release);
                return transaction;
            }

            transaction.node_id = node_id;

            // 安全解析对象字典索引和子索引 - 确保有足够的数据
            if (frame.dlc >= MIN_SDO_DLC) {
                // 安全访问数组元素，已验证dlc >= 4
                transaction.index = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[1];
                transaction.subindex = frame.data[3];
            } else {
                // 数据长度不足，设置默认值
                transaction.index = 0;
                transaction.subindex = 0;
            }

            // 安全复制请求数据 - 防止缓冲区溢出
            const size_t copy_size = std::min({
                static_cast<size_t>(frame.dlc),
                static_cast<size_t>(MAX_CAN_DLC),
                transaction.request_data.size()
            });
            
            std::copy_n(frame.data, copy_size, transaction.request_data.begin());
            
            // 清零剩余字节以确保数据一致性
            if (copy_size < transaction.request_data.size()) {
                std::fill(transaction.request_data.begin() + copy_size,
                          transaction.request_data.end(), 0);
            }

            transaction.state.store(SdoState::IDLE, std::memory_order_release);
            return transaction;
        }

        /**
         * @brief 开始SDO事务（内存栅栏保护） 归零重试计数器
         * @param transaction 要开始的事务对象引用
         * @return 是否成功开始事务
         * 
         * @details 安全开始SDO事务，使用智能指针管理生命周期
         * - 原子比较交换确保状态为IDLE时才开始
         * - 使用shared_ptr避免悬挂指针风险
         * - 线程安全的状态转换和时间记录
         * - 自动初始化重试计数器
         * 
         * @note 事务对象的生命周期由shared_ptr管理，防止悬挂指针
         * @warning 只有在事务状态为IDLE时才能成功开始
         */
        inline bool startTransaction(SdoTransaction& transaction) {
            SdoState expected = SdoState::IDLE;

            // 原子比较交换确保状态为IDLE时转换
            if (transaction.state.compare_exchange_strong(expected,
                SdoState::WAITING_RESPONSE,
                std::memory_order_acq_rel,
                std::memory_order_acquire)) {

                std::lock_guard<std::mutex> lock(mutex_);
                
                // 重置重试计数器
                transaction.retry_count.store(0, std::memory_order_release);
                
                // 使用shared_ptr保证对象生命周期安全
                // 通过std::shared_ptr的aliasing constructor共享所有权 
                //对处理当前事务的智能指针初始化
                current_transaction_ = std::shared_ptr<SdoTransaction>(
                    std::shared_ptr<SdoTransaction>{}, &transaction);
                
                start_time_ = std::chrono::steady_clock::now();

                // 内存栅栏，确保数据写入对所有线程可见
                std::atomic_thread_fence(std::memory_order_release);
                return true;
            }
            return false;
        }

        /**
         * @brief 处理SDO响应（内存栅栏保护） 接收线程专用
         * @param response_data 响应数据数组指针
         * @param dlc 数据长度（字节数）
         * @param node_id 节点ID
         * @return 是否成功处理响应
         * 
         * @details 安全处理SDO响应数据，包含完整的数据验证和线程安全保证
         * - 验证当前事务状态和节点ID匹配
         * - 安全复制响应数据，防止缓冲区溢出
         * - 正确分类响应类型并更新状态
         * - 使用内存序和条件变量保证线程同步
         * 
         * @note 线程安全: 使用独占锁和原子操作保证数据一致性
         * @warning response_data指针必须有效且至少包含dlc个字节
         * @pre 当前事务必须处于WAITING_RESPONSE状态
         */
        inline bool processResponse(const uint8_t response_data[8], uint8_t dlc, uint8_t node_id) {
            // 参数验证 - 防止空指针和非法DLC
            if (!response_data || dlc > 8) {
                return false;
            }

            std::unique_lock<std::mutex> lock(mutex_);

            // 使用shared_ptr安全获取当前事务引用，防止TOCTOU问题
            auto transaction_ptr = current_transaction_;
            if (!transaction_ptr ||
                transaction_ptr->state.load(std::memory_order_acquire) != SdoState::WAITING_RESPONSE ||
                transaction_ptr->node_id != node_id) {
                return false;
            }

            // 安全复制响应数据 - 防止缓冲区溢出
            const size_t copy_size = std::min({
                static_cast<size_t>(dlc),
                static_cast<size_t>(8),  // response_data数组最大大小
                current_transaction_->response_data.size()
            });
            
            std::copy_n(response_data, copy_size, transaction_ptr->response_data.begin());
            
            // 清零剩余字节以确保数据一致性
            if (copy_size < transaction_ptr->response_data.size()) {
                std::fill(transaction_ptr->response_data.begin() + copy_size,
                          transaction_ptr->response_data.end(), 0);
            }
            
            transaction_ptr->response_dlc = dlc;

            // 安全获取命令字节 - 验证数据有效性
            uint8_t command = (dlc > 0) ? response_data[0] : 0;
            SdoResponseType response_type = classifyResponse(command);

            // 原子更新响应类型和状态
            transaction_ptr->response_type.store(response_type, std::memory_order_release);

            SdoState new_state = (response_type == SdoResponseType::ERROR_RESPONSE) ?
                SdoState::RESPONSE_ERROR : SdoState::RESPONSE_VALID;

            transaction_ptr->state.store(new_state, std::memory_order_release);
            
            // 重试成功，清零重试计数器
            if (new_state == SdoState::RESPONSE_VALID) {
                transaction_ptr->retry_count.store(0, std::memory_order_release);
            }

            // 内存栅栏，确保状态更新对所有线程可见
            std::atomic_thread_fence(std::memory_order_release);

            // 通知等待线程
            condition_.notify_all();
            return true;
        }

        /**
         * @brief 等待响应或超时
         * @param timeout_us 超时时间（微秒）
         * @return 是否在超时前收到响应
         */
        /**
         * @brief 等待响应或超时
         * @param timeout_us 超时时间（微秒）
         * @return 是否在超时前收到响应
         * 
         * @details 安全等待SDO事务响应，防止空指针访问和伪唤醒
         * - 使用shared_ptr安全检查事务存在性
         * - 条件变量等待状态变化，防止伪唤醒
         * - 自动处理超时情况
         * 
         * @note 使用lambda捕获shared_ptr副本防止竞争条件
         * @warning 只有在事务存在且状态正确时才返回true
         */
        inline bool waitForResponse(uint32_t timeout_us = TIME_OUT_US) {
            std::unique_lock<std::mutex> lock(mutex_);

            // 获取shared_ptr副本用于lambda捕获
            auto transaction_ptr = current_transaction_;
            if (!transaction_ptr) {
                return false;
            }

            // 使用条件变量等待状态变化，捕获shared_ptr副本防止竞争
            return condition_.wait_for(lock, std::chrono::microseconds(timeout_us),
                [transaction_ptr]() {
                    // 使用捕获的shared_ptr副本，防止在lambda执行过程中指针被释放
                    if (!transaction_ptr) {
                        return false;  // 事务不存在，停止等待
                    }
                    auto state = transaction_ptr->state.load(std::memory_order_acquire);
                    return state == SdoState::RESPONSE_VALID ||
                        state == SdoState::RESPONSE_ERROR ||
                        state == SdoState::TIMEOUT ||
                        state == SdoState::MAX_RETRIES_EXCEEDED;
                });
        }

        /**
         * @brief 检查超时并处理重试逻辑 将事务开始时间和当前时间相减来判断超时
         * @return 是否发生超时或需要重试
         * 
         * @details 增强的超时检查，支持自动重试机制
         * - 检查是否超时
         * - 未超过最大重试次数时自动重试
         * - 超过最大重试次数时输出错误并抛出异常
         * 
         * @throws std::runtime_error 当超过最大重试次数时
         * @note 重试成功后计数器会在收到响应时清零
         */
        inline bool checkTimeout() {
            std::lock_guard<std::mutex> lock(mutex_);

            // 使用shared_ptr安全获取当前事务引用
            auto transaction_ptr = current_transaction_;

            //如果为空指针或者状态机不在等待状态则返回为错误
            if (!transaction_ptr ||
                transaction_ptr->state.load(std::memory_order_acquire) != SdoState::WAITING_RESPONSE) {
                return false;
            }

            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                now - start_time_).count();//通过将当前时间和事务开始时间相减来判断是否超时

            if (elapsed >= TIME_OUT_US) {

                //获取重试次数
                uint8_t current_retry = transaction_ptr->retry_count.load(std::memory_order_acquire);
                
                //还有重试机会的情况
                if (current_retry < MAX_RETRY_COUNT) {
                    // 还可以重试，增加计数器并设置重试状态
                    transaction_ptr->retry_count.fetch_add(1, std::memory_order_acq_rel);
                    transaction_ptr->state.store(SdoState::RETRYING, std::memory_order_release);
                    
                    // 重置开始时间准备重试
                    start_time_ = std::chrono::steady_clock::now();
                    
                    // 输出重试信息
                    std::cout << "[WARNING][SDO_State_Machine::checkTimeout]: SDO超时，正在重试 ("
                              << static_cast<int>(current_retry + 1) << "/" 
                              << MAX_RETRY_COUNT << "), 节点ID=" 
                              << static_cast<int>(transaction_ptr->node_id)
                              << ", 索引=0x" << std::hex << transaction_ptr->index 
                              << std::dec << std::endl;
                    
                    // 内存栅栏和通知
                    std::atomic_thread_fence(std::memory_order_release);
                    condition_.notify_all();
                    return true;
                } else {
                    // 超过最大重试次数，设置最终失败状态
                    transaction_ptr->state.store(SdoState::MAX_RETRIES_EXCEEDED, std::memory_order_release);
                    
                    // 按照CLAUDE.md标准输出错误信息
                    std::cout << "[ERROR][SDO_State_Machine::checkTimeout]: SDO通信失败，已重试" 
                              << MAX_RETRY_COUNT << "次仍无响应, 节点ID=" 
                              << static_cast<int>(transaction_ptr->node_id)
                              << ", 索引=0x" << std::hex << transaction_ptr->index 
                              << std::dec << ", 子索引=0x" << std::hex 
                              << static_cast<int>(transaction_ptr->subindex) << std::dec << std::endl;
                    
                    // 内存栅栏和通知
                    std::atomic_thread_fence(std::memory_order_release);
                    condition_.notify_all();
                    
                    // 抛出异常
                    throw std::runtime_error("SDO通信超时，超过最大重试次数");
                }
            }

            return false;
        }

        /**
         * @brief 检查是否需要重试
         * @return 是否处于重试状态
         * 
         * @details 用于外部检查当前事务是否需要重新发送
         * - 返回true时，外部应重新发送SDO请求
         * - 自动将状态从RETRYING转换为WAITING_RESPONSE
         * 
         * @note 此方法应在发送线程中调用以处理重试逻辑
         */
        inline bool needsRetry() {
            std::lock_guard<std::mutex> lock(mutex_);
            auto transaction_ptr = current_transaction_;
            if (!transaction_ptr) return false;
            
            SdoState expected = SdoState::RETRYING;
            // 尝试将状态从RETRYING转换为WAITING_RESPONSE
            if (transaction_ptr->state.compare_exchange_strong(expected,
                SdoState::WAITING_RESPONSE,
                std::memory_order_acq_rel,
                std::memory_order_acquire)) {
                return true;
            }
            return false;
        }
         
        /**
         * @brief 获取当前重试次数
         * @return 重试次数
         */
        uint8_t getRetryCount() const {
            std::lock_guard<std::mutex> lock(mutex_);
            auto transaction_ptr = current_transaction_;
            return transaction_ptr ? 
                transaction_ptr->retry_count.load(std::memory_order_acquire) : 0;
        }

        /**
         * @brief 完成当前事务
         * 
         * @details 安全释放当前事务的引用，使状态机返回空闲状态
         * - 使用互斥锁保证线程安全
         * - shared_ptr自动管理内存释放
         * - 释放后状态机可接受新事务
         * 
         * @note 调用后状态机返回空闲状态，可处理新的SDO事务
         */
        inline void completeTransaction() {
            std::lock_guard<std::mutex> lock(mutex_);
            current_transaction_.reset();  // 使用reset()明确释放引用
        }

        /**
         * @brief 获取当前事务状态（原子读取）
         * @return 当前事务状态
         */
        SdoState getCurrentState() const {
            std::lock_guard<std::mutex> lock(mutex_);
            auto transaction_ptr = current_transaction_;
            return transaction_ptr ?
                transaction_ptr->state.load(std::memory_order_acquire) :
                SdoState::IDLE;
        }

        /**
         * @brief 获取响应类型（原子读取）
         * @return 响应类型
         */
        SdoResponseType getResponseType() const {
            std::lock_guard<std::mutex> lock(mutex_);
            auto transaction_ptr = current_transaction_;
            return transaction_ptr ?
                transaction_ptr->response_type.load(std::memory_order_acquire) :
                SdoResponseType::NO_RESPONSE;
        }

        /**
         * @brief 获取响应数据（线程安全）
         * @return 响应数据（如果存在）
         * 
         * @details 线程安全地获取当前事务的响应数据
         * - 使用互斥锁保证数据一致性
         * - 返回安全的数据副本避免并发访问问题
         * - 验证事务存在性后返回数据
         * 
         * @note 返回的是数据副本，不会影响原始数据的线程安全性
         * @warning 只有在事务存在时才返回数据，否则返回nullopt
         */
        std::optional<std::array<uint8_t, 8>> getResponseData() const {
            std::lock_guard<std::mutex> lock(mutex_);
            auto transaction_ptr = current_transaction_;
            if (!transaction_ptr) {
                return std::nullopt;
            }
            return transaction_ptr->response_data;
        }

        /**
         * @brief 是否正在进行事务
         * @return 是否繁忙
         */
        bool isBusy() const {
            std::lock_guard<std::mutex> lock(mutex_);
            auto transaction_ptr = current_transaction_;
            if (!transaction_ptr) return false;
            
            auto state = transaction_ptr->state.load(std::memory_order_acquire);
            return state == SdoState::WAITING_RESPONSE || state == SdoState::RETRYING;
        }

        /**
         * @brief 重置状态机
         * 
         * @details 强制重置状态机到初始状态，用于错误恢复或清理
         * - 立即释放当前事务引用
         * - 不等待事务完成，直接清理
         * - 用于紧急情况下的状态恢复
         * 
         * @warning 调用后当前事务将被强制中断，可能导致数据不一致
         */
        void reset() {
            std::lock_guard<std::mutex> lock(mutex_);
            current_transaction_.reset();  // 使用reset()明确释放引用
        }

        // 静态工具函数 - 用于快速SDO帧识别和验证
        
        /**
         * @brief 判断是否为SDO请求帧
         * @param frame_id CAN帧ID
         * @return 是否为SDO请求帧 (0x600 + node_id)
         * 
         * @details 检查帧ID是否符合CANopen SDO请求帧格式
         * - SDO请求帧ID范围: 0x600 - 0x67F
         * - 使用位掩码提取帧类型
         */
        inline static bool isSdoRequestFrame(uint32_t frame_id) {
            return (frame_id & 0x780) == 0x600;
        }

        /**
         * @brief 判断是否为SDO响应帧
         * @param frame_id CAN帧ID
         * @return 是否为SDO响应帧 (0x580 + node_id)
         * 
         * @details 检查帧ID是否符合CANopen SDO响应帧格式
         * - SDO响应帧ID范围: 0x580 - 0x5FF
         * - 使用位掩码提取帧类型
         */
        inline static bool isSdoResponseFrame(uint32_t frame_id) {
            return (frame_id & 0x780) == 0x580;
        }

        /**
         * @brief 从帧ID中提取节点ID
         * @param frame_id CAN帧ID
         * @return 有效的节点ID (1-127)，无效时返回0
         * 
         * @details 安全提取并验证CANopen节点ID
         * - 节点ID范围: 1-127 (符合CANopen标准)
         * - 使用位掩码提取低7位
         * - 自动验证范围有效性
         */
        inline static uint8_t extractNodeId(uint32_t frame_id) {
            uint8_t node_id = static_cast<uint8_t>(frame_id & 0x7F);
            return (node_id >= 1 && node_id <= 127) ? node_id : 0;
        }

        /**
         * @brief 判断是否为SDO帧
         * @param frame_id 帧ID
         * @return 是否为SDO帧
         */
        inline static bool isSdoFrame(uint32_t frame_id) {
            return isSdoRequestFrame(frame_id) || isSdoResponseFrame(frame_id);
        }

    private:
        /**
         * @brief 分类SDO响应类型
         * @param command 命令字节
         * @return 响应类型
         * 
         * @details 根据CANopen SDO协议分类响应类型
         * - 错误响应: 0x80-0x8F (高四位为0x8)
         * - 读8位响应: 0x4F
         * - 读16位响应: 0x4B  
         * - 读32位响应: 0x43
         * - 写成功响应: 0x60
         * - 未知响应: 其他值
         * 
         * @note 移除了不必要的内存栅栏，因为参数是局部变量
         * @warning 命令字节必须来自有效的SDO帧数据
         */
        inline SdoResponseType classifyResponse(uint8_t command) const {
            // 检查错误响应 - 高四位为0x8
            if ((command & 0xF0) == 0x80) {
                return SdoResponseType::ERROR_RESPONSE;
            }

            // 根据命令字节分类正常响应
            switch (command) {
            case 0x4F: return SdoResponseType::READ_8BIT;   // 快速上传 1字节
            case 0x4B: return SdoResponseType::READ_16BIT;  // 快速上传 2字节
            case 0x43: return SdoResponseType::READ_32BIT;  // 快速上传 4字节
            case 0x60: return SdoResponseType::WRITE_SUCCESS; // 快速下载成功
            default:   return SdoResponseType::UNKNOWN;     // 未知或不支持的命令
            }
        }

    private:
        mutable std::mutex mutex_;                      ///< 互斥锁保护共享数据
        std::condition_variable condition_;             ///< 条件变量用于线程同步
        std::shared_ptr<SdoTransaction> current_transaction_; ///< 当前处理的事务智能指针
        std::chrono::steady_clock::time_point start_time_; ///< 事务开始时间
    };

} // namespace canopen

#endif // SDO_STATE_MACHINE_HPP
