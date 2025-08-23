/**
 * @file CAN_processing.hpp
 * @brief 实现CAN帧解析和电机状态更新的核心处理逻辑
 *
 * @details 本文件提供了机械臂驱动程序的CAN通信协议栈解析模块，
 * 负责对接收自串口转CAN芯片的特殊格式CAN帧进行识别、分类和处理。
 * 该模块支持完整的CANopen协议帧类型，包括NMT、SYNC、SDO和PDO，
 * 并通过PDO映射机制实现电机状态数据的高效更新。
 *
 * 主要功能包括：
 * - 解析特殊格式的CAN帧并识别协议类型
 * - 分发不同类型CAN帧到对应的处理函数
 * - 实现PDO数据帧的自动映射和电机状态更新
 * - 提供线程安全的电机数据写入机制
 * - 支持多电机系统的批量数据处理
 *
 * @note 该模块专为串口转CAN芯片的特殊帧格式设计，保持与硬件的
 * 紧密耦合。所有处理函数均采用内联优化，确保2ms实时同步周期内
 * 完成数据处理。电机数据访问通过互斥锁保护，PDO处理支持原子化
 * 标志位操作。
 *
 * @par 协议支持：
 * - NMT网络管理帧：设备控制和状态监控
 * - SYNC同步帧：系统时钟同步信号
 * - SDO服务数据对象：参数配置和诊断通信
 * - PDO过程数据对象：实时数据交换（RPDO/TPDO）
 *
 * @par 处理流程：
 * 1. CAN帧接收 → 2. 协议类型识别 → 3. 分发到专用处理器
 * 4. PDO数据映射 → 5. 电机状态更新 → 6. 触发数据刷新
 *
 * @par 线程安全性：
 * - 每个电机对象独立加锁，避免全局锁竞争
 * - PDO映射表只读访问，支持多线程并发查询
 * - 原子标志位操作确保刷新状态的一致性
 */










#ifndef CAN_PROCESSING_HPP
#define CAN_PROCESSING_HPP




#include <vector>
#include <array>
#include "CAN_frame.hpp"
#include "PDO_config.hpp"
#include "class_motor.hpp"




/* 先声明所有处理函数 */
inline void handleNmtCommand(const uint8_t* data, uint8_t dlc);
inline void handleSyncFrame();
inline void handleNodeStatus(uint8_t nodeId, uint8_t state);
inline void handleSdoCommand(uint8_t nodeId, const uint8_t* data, uint8_t dlc);
inline void handleSdoResponse(uint8_t nodeId, const uint8_t* data, uint8_t dlc);
inline void ProcessPDO(uint8_t nodeId, uint8_t pdoNum, bool isTxFrame,
    const uint8_t* data, uint8_t dlc,
    std::array<Motor, 6>& motors,
    const std::vector<PdoMappingEntry>& pdoTable);
inline void handleUnknownFrame(uint32_t id, const uint8_t* data, uint8_t dlc);



/**
 * @brief 解析CAN帧并更新电机状态
 *
 * @param frame 接收到的CAN帧(保持原有特殊格式)
 * @param motors 机械臂所有电机的引用(使用数组存储)
 * @param pdoTable PDO配置表的引用(使用const引用避免拷贝)
 *
 * @note 函数根据CAN ID自动识别帧类型并分发到对应的处理逻辑
 * @warning 保持原有CAN帧格式不变，专门适配串口转CAN芯片的特殊格式
 */
void parseCanFrame(const CanFrame& frame, std::array<Motor, 6>& motors,
    const std::vector<PdoMappingEntry>& pdoTable) {
    const uint32_t id = frame.frameID;
    const uint8_t* data = frame.data;
    const uint8_t dlc = frame.dlc;
    // 帧类型判断分支
    if (id == 0x0000) {
        // NMT设备控制报文
        handleNmtCommand(data, dlc);
    }
    else if (id == 0x0080) {
        // SYNC同步报文
        handleSyncFrame();
    }
    else if ((id >= 0x0701) && (id <= 0x077F)) {
        // NMT节点状态报文
        uint8_t nodeId = id - 0x0700;
        handleNodeStatus(nodeId, data[0]);
    }
    else if ((id >= 0x0601) && (id <= 0x067F)) {
        // SDO命令报文
        uint8_t nodeId = id - 0x0600;
        handleSdoCommand(nodeId, data, dlc);
    }
    else if ((id >= 0x0581) && (id <= 0x05FF)) {
        // SDO响应报文
        uint8_t nodeId = id - 0x0580;
        handleSdoResponse(nodeId, data, dlc);
    }
    else {
        // PDO帧处理逻辑合并
        if ((id >= 0x0201 && id <= 0x027F) ||  // RPDO1
            (id >= 0x0301 && id <= 0x037F) ||  // RPDO2
            (id >= 0x0181 && id <= 0x01FF) ||  // TPDO1
            (id >= 0x0281 && id <= 0x02FF)) {  // TPDO2
            uint8_t nodeId;
            uint8_t pdoNum;
            bool isTx;


            // 通用PDO帧类型识别
            if (id >= 0x0201 && id <= 0x027F) {
                nodeId = id - 0x0200;
                pdoNum = 1;
                isTx = false;
            }


            else if (id >= 0x0301 && id <= 0x037F) {
                nodeId = id - 0x0300;
                pdoNum = 2;
                isTx = false;
            }


            else if (id >= 0x0181 && id <= 0x01FF) {
                nodeId = id - 0x0180;
                pdoNum = 1;
                isTx = true;
            }


            else if (id >= 0x0281 && id <= 0x02FF) {
                nodeId = id - 0x0280;
                pdoNum = 2;
                isTx = true;
            }


            // 验证nodeId有效性
            if (nodeId == 0 || nodeId > motors.size()) {
                handleUnknownFrame(id, data, dlc);
                return;
            }

            //执行PDO的处理
            ProcessPDO(nodeId, pdoNum, isTx, data, dlc, motors, pdoTable);

        }


        else {
            // 其他未知帧
                handleUnknownFrame(id, data, dlc);
        }
    }
}

/* 以下是各类型帧的处理函数声明（具体实现留空） */

/**
 * @brief 处理NMT命令帧
 */
inline void handleNmtCommand(const uint8_t* data, uint8_t dlc) {

    (void)data;
    (void)dlc;
    // 实现留空
}

/**
 * @brief 处理SYNC同步帧
 */
inline void handleSyncFrame() {
    // 实现留空  
}

/**
 * @brief 处理节点状态帧
 */
inline void handleNodeStatus(uint8_t nodeId, uint8_t state) {
    // 实现留空
}

/**
 * @brief 处理SDO命令帧
 */
inline void handleSdoCommand(uint8_t nodeId, const uint8_t* data, uint8_t dlc) {
    // 实现留空
}

/**
 * @brief 处理SDO响应帧
 */
inline void handleSdoResponse(uint8_t nodeId, const uint8_t* data, uint8_t dlc) {
    // 实现留空
}

/**
 * @brief 处理PDO帧数据
 *
 * @param nodeId 电机节点ID (1-N)
 * @param pdoNum PDO编号(1-4)
 * @param data CAN数据区指针
 * @param isTxFrame  是否是TPDO（下位机→上位机）
 * @param dlc 数据长度
 * @param motors 电机数组引用
 * @param pdoTable PDO配置表引用
 *
 * @note 函数逻辑：
 * 1. 检查节点ID有效性
 * 2. 遍历PDO配置表查找匹配条目
 * 3. 根据配置将数据拷贝到电机对应字段
 * 4. 设置数据刷新标志位
 * 5. 立即执行数据刷新
 * 
 * TPDO下位机→上位机
 * RPDO上位机→下位机
 */
inline void ProcessPDO(uint8_t nodeId, //节点ID
    uint8_t pdoNum, //PDO编号
    bool isTxFrame,//是否是TPDO（下位机→上位机）
    const uint8_t* data, //CAN帧的数据区的指针
    uint8_t dlc,//数据长度
    std::array<Motor, 6>& motors,//电机数组的引用
    const std::vector<PdoMappingEntry>& pdoTable) {
    // 1. 检查节点ID有效性
    if (nodeId == 0 || nodeId > motors.size()) {
        return; // 无效节点ID
    }

    // 获取目标电机引用(节点ID-1转换为数组索引)
    Motor& targetMotor = motors[nodeId - 1];
    std::lock_guard<std::mutex> lock(targetMotor.mtx_);//上锁

    // 2. 遍历PDO配置表
    for (const auto& entry : pdoTable) {
        // 匹配当前节点和PDO编号
        if (entry.motorIndex == nodeId && entry.pdoIndex == pdoNum && entry.isTx == isTxFrame) {
            // 3. 检查数据边界
            if (entry.size > 8 || entry.offsetInPdo + entry.size > dlc) {
                continue; // 数据越界跳过
            }

            // 4. 计算目标字段地址
            uint8_t* motorField = reinterpret_cast<uint8_t*>(&targetMotor) + entry.motorFieldOffset;

            // 5. 拷贝数据到电机字段
            std::memcpy(motorField, &data[entry.offsetInPdo], entry.size);

            // 动态设置标志位（在锁保护下操作，可直接使用非原子写入）
            //首先分析是写入还是发送，再根据电机端匹配到的地址对应到上位机的电机类的变量，修改完毕后，改变刷新标志等待刷新
            if (entry.isTx) {//TPDO的情况（下位机→上位机）
                switch (entry.index) {

                case OD_STATUS_WORD: // 0x6041 状态字（接收）
                    targetMotor.stateAndMode.refresh = true;
                    targetMotor.stateAndMode.controlData.statusWordRaw[0] = data[entry.offsetInPdo];
                    targetMotor.stateAndMode.controlData.statusWordRaw[1] = data[entry.offsetInPdo + 1];
                    break;

                case OD_ACTUAL_CURRENT: // 0x6078 实际电流（接收）
                    targetMotor.current.raw_actual.bytes_[0] = data[entry.offsetInPdo];
                    targetMotor.current.raw_actual.bytes_[1] = data[entry.offsetInPdo + 1];
                    targetMotor.current.flags_.store(
                        targetMotor.current.flags_.load(std::memory_order_relaxed) |
                        MotorCurrent::RAW_DATA_RECEIVE_NEED_REFRESH,
                        std::memory_order_relaxed);
                    break;

                case OD_ACTUAL_POSITION: // 0x6064 实际位置（接收）
                    for (int i = 0; i < 4; ++i) {
                        targetMotor.position.raw_actual.bytes_[i] = data[entry.offsetInPdo + i];
                    }
                    targetMotor.position.flags_.store(
                        targetMotor.position.flags_.load(std::memory_order_relaxed) |
                        MotorPosition::RAW_DATA_RECEIVE_NEED_REFRESH,
                        std::memory_order_relaxed);
                    break;

                case OD_ACTUAL_VELOCITY: // 0x606C 实际速度（接收）
                    targetMotor.velocity.raw_actual.bytes_[0] = data[entry.offsetInPdo];
                    targetMotor.velocity.raw_actual.bytes_[1] = data[entry.offsetInPdo + 1];
                    targetMotor.velocity.flags_.store(
                        targetMotor.velocity.flags_.load(std::memory_order_relaxed) |
                        MotorVelocity::RAW_DATA_RECEIVE_NEED_REFRESH,
                        std::memory_order_relaxed);
                    break;

                case OD_MODES_OF_DISPLAY: // 0x6061 运行模式（接收）
                    targetMotor.stateAndMode.modeData.modeOfOperationRaw[0] = data[entry.offsetInPdo];
                    break;

                case OD_ERROR_CODE: // 0x603F 错误码（接收）
                    targetMotor.stateAndMode.modeData.errorCodeRaw[0] = data[entry.offsetInPdo];
                    targetMotor.stateAndMode.modeData.errorCodeRaw[1] = data[entry.offsetInPdo + 1];
                    break;
                
                }
            }
            else {
                switch (entry.index) {//RPDO的情况（上位机→下位机）
                



                case OD_CONTROL_WORD: // 0x6040 控制字（发送）
                    targetMotor.stateAndMode.controlData.controlWordRaw[0] = data[entry.offsetInPdo];
                    targetMotor.stateAndMode.controlData.controlWordRaw[1] = data[entry.offsetInPdo + 1];
                    targetMotor.stateAndMode.refresh = true;
                    break;

                case OD_TARGET_CURRENT: // 0x6071 目标电流（发送）
                    targetMotor.current.raw_target.bytes_[0] = data[entry.offsetInPdo];
                    targetMotor.current.raw_target.bytes_[1] = data[entry.offsetInPdo + 1];
                    targetMotor.current.flags_.store(
                        targetMotor.current.flags_.load(std::memory_order_relaxed) |
                        MotorCurrent::RAW_DATA_SEND_NEED_REFRESH,
                        std::memory_order_relaxed);
                    break;

                case OD_TARGET_POSITION: // 0x607A 目标位置（发送）
                    for (int i = 0; i < 4; ++i) {
                        targetMotor.position.raw_target.bytes_[i] = data[entry.offsetInPdo + i];
                    }
                    targetMotor.position.flags_.store(
                        targetMotor.position.flags_.load(std::memory_order_relaxed) |
                        MotorPosition::RAW_DATA_SEND_NEED_REFRESH,
                        std::memory_order_relaxed);
                    break;

                case OD_TARGET_VELOCITY_VELOCITY_MODE: // 0x60FF 目标速度（速度模式发送）
                    targetMotor.velocity.raw_target_velocity_mode.bytes_[0] = data[entry.offsetInPdo];
                    targetMotor.velocity.raw_target_velocity_mode.bytes_[1] = data[entry.offsetInPdo + 1];
                    targetMotor.velocity.flags_.store(
                        targetMotor.velocity.flags_.load(std::memory_order_relaxed) |
                        MotorVelocity::RAW_DATA_SEND_NEED_REFRESH_VELOCITY_MODE,
                        std::memory_order_relaxed);
                    break;


                case OD_TARGET_VELOCITY_POSITION_MODE: // 0x6081 目标速度（位置模式发送）
                    targetMotor.velocity.raw_target_position_mode.bytes_[0] = data[entry.offsetInPdo];
                    targetMotor.velocity.raw_target_position_mode.bytes_[1] = data[entry.offsetInPdo + 1];
                    targetMotor.velocity.flags_.store(
                        targetMotor.velocity.flags_.load(std::memory_order_relaxed) |
                        MotorVelocity::RAW_DATA_SEND_NEED_REFRESH_POSITION_MODE,
                        std::memory_order_relaxed);
                    break;

                case OD_MODES_OF_OPERATION: // 0x6060 运行模式（发送）
                    targetMotor.stateAndMode.modeData.modeOfOperationRaw[0] = data[entry.offsetInPdo];
                    break;

                case OD_ACCELERATION: // 0x6083 加速度（发送）
                    targetMotor.accelDecel.raw_accel.bytes_[0] = data[entry.offsetInPdo];
                    targetMotor.accelDecel.raw_accel.bytes_[1] = data[entry.offsetInPdo + 1];
                    break;

                case OD_DECELERATION: // 0x6084 减速度（发送）
                    targetMotor.accelDecel.raw_decel.bytes_[0] = data[entry.offsetInPdo];
                    targetMotor.accelDecel.raw_decel.bytes_[1] = data[entry.offsetInPdo + 1];
                    break;
                }
            }
        }
    }

    // 7. 立即刷新所有数据
    //targetMotor.refreshMotorData(targetMotor.stateAndMode);
    targetMotor.refreshMotorData(targetMotor.current);
    targetMotor.refreshMotorData(targetMotor.position);
    targetMotor.refreshMotorData(targetMotor.velocity);
}





/**
 * @brief 处理未知帧类型
 */
inline void handleUnknownFrame(uint32_t id, const uint8_t* data, uint8_t dlc) {
    // 实现留空
}



#endif // !CAN_PROCESSING_HPP