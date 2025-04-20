#pragma once
#ifndef PDO_CONFIG_HPP
#define PDO_CONFIG_HPP

#include <cstdint>
#include <cstddef>
#include <vector>
#include <stdexcept>
#include <string>
#include "class_motor.hpp"

/**
 * @brief 将逻辑电机编号转换为 CANopen SDO 从站 ID
 *
 * 在 CANopen 协议中，每个电机设备通常分配一个唯一的 SDO 从站 ID，
 * 本函数用于将逻辑编号 (`motorIndex = 1..N`) 映射到 SDO 通信 ID (`0x601..0x60N`)。
 *
 * @param motorIndex 逻辑电机编号（通常从 1 开始编号）
 * @return uint16_t 对应的 SDO 从站 ID，例如 `motorIndex=1` 返回 `0x601`
 * @throws std::out_of_range 若 `motorIndex` 不在允许范围内（1..12）
 */
inline uint16_t toSdoMotorId(uint8_t motorIndex)
{
    // 检查输入值是否在合理范围 (1..12)
    // 在 CANopen 规范中，电机 ID 通常有最大值限制，避免地址冲突
    if (motorIndex == 0 || motorIndex > 12) {
        throw std::out_of_range("motorIndex must be 1..12"); // 超出范围则抛出异常
    }

    // 计算并返回对应的 SDO 从站 ID
    // 按照规则：motorIndex=1 => 0x601, motorIndex=2 => 0x602 ...
    return uint16_t(0x600 + motorIndex);
}


/**
 * @brief 根据逻辑电机编号与 PDO 通道编号，计算 RPDO 的 COB-ID
 *
 * 兼容 1~4 通道的 RPDO:
 *  - RPDO1 => 0x200 + motorIndex
 *  - RPDO2 => 0x300 + motorIndex
 *  - RPDO3 => 0x400 + motorIndex
 *  - RPDO4 => 0x500 + motorIndex
 *
 * @param motorIndex 逻辑电机编号 (通常为 1..N)，用于区分同一条 CAN 总线下的不同电机
 * @param pdoIndex   PDO 通道编号 (1..4)，对应 RPDO1..RPDO4
 * @return 计算出的 RPDO COB-ID。例如 motorIndex=1, pdoIndex=1 => 0x201
 * @throws std::out_of_range 若 pdoIndex 不在 1..4 范围
 */
inline uint32_t toRpdoCobId(uint8_t motorIndex, uint8_t pdoIndex)
{
    switch (pdoIndex)
    {
    case 1: return 0x200 + motorIndex; // RPDO1
    case 2: return 0x300 + motorIndex; // RPDO2
    case 3: return 0x400 + motorIndex; // RPDO3
    case 4: return 0x500 + motorIndex; // RPDO4
    default:
        throw std::out_of_range("Unsupported RPDO pdoIndex (valid 1..4)");
    }
}

/**
 * @brief 根据逻辑电机编号与 PDO 通道编号，计算 TPDO 的 COB-ID
 *
 * 兼容 1~4 通道的 TPDO:
 *  - TPDO1 => 0x180 + motorIndex
 *  - TPDO2 => 0x280 + motorIndex
 *  - TPDO3 => 0x380 + motorIndex
 *  - TPDO4 => 0x480 + motorIndex
 *
 * @param motorIndex 逻辑电机编号 (通常为 1..N)，用于区分同一条 CAN 总线下的不同电机
 * @param pdoIndex   PDO 通道编号 (1..4)，对应 TPDO1..TPDO4
 * @return 计算出的 TPDO COB-ID。例如 motorIndex=2, pdoIndex=3 => 0x382
 * @throws std::out_of_range 若 pdoIndex 不在 1..4 范围
 */
inline uint32_t toTpdoCobId(uint8_t motorIndex, uint8_t pdoIndex)
{
    switch (pdoIndex)
    {
    case 1: return 0x180 + motorIndex; // TPDO1
    case 2: return 0x280 + motorIndex; // TPDO2
    case 3: return 0x380 + motorIndex; // TPDO3
    case 4: return 0x480 + motorIndex; // TPDO4
    default:
        throw std::out_of_range("Unsupported TPDO pdoIndex (valid 1..4)");
    }
}


/*****************************************************************************
 模板结构：表示“单个电机”常见的固定映射
 ****************************************************************************/

 /**
  * @brief 单电机 PDO 映射模板 (PdoMappingTemplate)
  *
  * 该结构体描述了 _单个电机_ 在某个 PDO 通道上的对象字典映射关系，作为 _模板_ 存储：
  * - _不包含 motorIndex_，因为它适用于所有电机
  * - 仅描述 _某个 PDO 通道_ 内部的数据映射，如：
  *   - RPDO1 (pdoIndex=1) 映射  目标位置 (OD_TARGET_POSITION)
  *   - TPDO1 (pdoIndex=1) 映射  实际位置 (OD_ACTUAL_POSITION)
  * - 在  初始化期 ，会基于该模板  复制并补充 motorIndex ，生成最终的 `PdoMappingEntry`
  *
  * @brief PdoMappingTemplate::isTx              PDO 方向 (true: TPDO, false: RPDO)
  * @brief PdoMappingTemplate::pdoIndex          PDO 通道编号 (1..4)
  * @brief PdoMappingTemplate::index             映射的对象字典索引 (16 位)
  * @brief PdoMappingTemplate::subIndex          映射的对象字典子索引 (通常为 0)
  * @brief PdoMappingTemplate::offsetInPdo       在 PDO 数据帧中的偏移量 (字节)
  * @brief PdoMappingTemplate::size              映射数据长度 (字节)
  * @brief PdoMappingTemplate::motorFieldOffset  在 Motor 结构体中的字段偏移量
  */
struct PdoMappingTemplate
{
    bool     isTx;         ///< PDO 方向 (true: TPDO, false: RPDO)
    uint8_t  pdoIndex;     ///< PDO 通道编号 (1..4)
    uint16_t index;        ///< 映射的对象字典索引 (16 位)
    uint8_t  subIndex;     ///< 映射的对象字典子索引 (通常为 0)
    uint8_t  offsetInPdo;  ///< 在 PDO 数据帧中的起始字节
    uint8_t  size;         ///< 映射数据长度 (字节)
    size_t   motorFieldOffset; ///< 在 Motor 结构体中的字段偏移量
};


/*****************************************************************************
 最终的映射条目：带上 motorIndex, 用于实际读写
 ****************************************************************************/
 
 /**
  * @brief 最终的 PDO 映射条目 (PdoMappingEntry)
  * @brief PdoMappingEntry::  motorIndex        逻辑电机编号 (通常 1..N)
  * @brief PdoMappingEntry::  isTx              PDO 方向 (true: TPDO, false: RPDO)
  * @brief PdoMappingEntry::  pdoIndex          PDO 通道编号 (1..4)
  * @brief PdoMappingEntry::  index             映射的对象字典索引 (16 位)
  * @brief PdoMappingEntry::  subIndex          映射的对象字典子索引 (通常为 0)
  * @brief PdoMappingEntry::  offsetInPdo       在 PDO 数据帧中的偏移量 (字节)
  * @brief PdoMappingEntry::  size              映射数据长度 (字节)
  * @brief PdoMappingEntry::motorFieldOffset  在 Motor 结构体中的字段偏移量
  */
struct PdoMappingEntry
{
    uint8_t  motorIndex;       ///< 逻辑电机编号 (通常 1..N)
    bool     isTx;             ///< PDO 方向 (true: TPDO, false: RPDO)
    uint8_t  pdoIndex;         ///< PDO 通道编号 (1..4)
    uint16_t index;            ///< 映射的对象字典索引 (16 位)
    uint8_t  subIndex;         ///< 映射的对象字典子索引 (通常为 0)
    uint8_t  offsetInPdo;      ///< 在 PDO 数据帧中的偏移量 (字节)
    uint8_t  size;             ///< 映射数据长度 (字节)
    size_t   motorFieldOffset; ///< 在 Motor 结构体中的字段偏移量
};


/*****************************************************************************
 * 4) 准备一个“单电机模板”数组 (编译期常量),
 *    这表示：每个电机都要映射什么对象, 放在哪个PDO, 偏移多少
 ****************************************************************************/
inline const std::vector<PdoMappingTemplate>& getDefaultMotorTemplate()
{
    // 例如：RPDO1 => 目标位置 (4字节, offset=0), 控制字(2字节, offset=4)
    //       TPDO1 => 实际位置 (4字节, offset=0)
    static const std::vector<PdoMappingTemplate> singleMotorTemplate = {
        // RPDO1
        { false, 1, OD_TARGET_POSITION, 0x00, 0, 4, offsetof(Motor, position.targetPositionRaw) },
        { false, 1, OD_CONTROL_WORD,    0x00, 4, 2, offsetof(Motor, stateAndMode.controlWordRaw) },
        
        // TPDO1
        { true,  1, OD_ACTUAL_POSITION, 0x00, 0, 4, offsetof(Motor, position.actualPositionRaw) },
    };
    return singleMotorTemplate;
}

/*****************************************************************************
  在程序启动/机械臂初始化时: 根据 “机械臂实际有几个电机”
  复制“单电机模板” 生成 “最终映射表” (PdoMappingEntry[]).
 ****************************************************************************/



 /**
  * @brief 构建“机械臂”上所有电机的 PDO 映射表 (buildArmMappingTable)
  *
  * 该函数在_程序初始化阶段_被调用，用于为_当前机械臂上的所有电机_
  * 生成一张完整的 PDO 映射表。映射表中的条目来源于_单电机模板_，并在
  * 复制过程中填充_motorIndex_，以适配不同编号的电机。
  *
  * - _编译期_：
  *   - `getDefaultMotorTemplate()` 定义了_单个电机_的标准 PDO 映射规则。
  *   - 其中包含 哪些对象字典被映射 ，存在哪个 PDO，占多少字节 等信息。
  *
  * - _初始化期_：
  *   - `motorCount` 指定 本机械臂上实际有几个电机（例如 6、7、12...）。
  *   - 遍历 motorIndex=1..motorCount，为每个电机 拷贝一份模板数据，
  *     并填充 `motorIndex` 以形成最终映射表 `std::vector<PdoMappingEntry>`。
  *
  * - _运行期_：
  *   - 该映射表保持不可变，用于 PDO 读写时自动查找 Motor 结构体对应变量，
  *     例如：解析 TPDO，将 偏移 0 的 4 字节数据填充到 `Motor.position.actualPositionRaw`。
  *
  * @param motorCount 机械臂上电机个数 (通常为 6, 7, 12...)，决定最终映射表的大小
  * @return std::vector<PdoMappingEntry> 生成的 _最终 PDO 映射表_
  *         其中 `motorIndex=1..motorCount`，其余字段来自 `getDefaultMotorTemplate()`
  */
inline std::vector<PdoMappingEntry> buildArmMappingTable(uint8_t motorCount)
{
    std::vector<PdoMappingEntry> finalTable;
    finalTable.reserve(motorCount * getDefaultMotorTemplate().size()); // 预分配空间，提高性能

    for (uint8_t m = 1; m <= motorCount; m++)
    {
        // 遍历单电机模板，并复制到最终表中
        for (auto& tmpl : getDefaultMotorTemplate())
        {
            PdoMappingEntry entry;
            entry.motorIndex = m;               ///< 赋值 motorIndex，区分不同电机
            entry.isTx = tmpl.isTx;             ///< 继承 TPDO / RPDO 方向
            entry.pdoIndex = tmpl.pdoIndex;     ///< 继承 PDO 通道编号 (1~4)
            entry.index = tmpl.index;           ///< 继承对象字典索引
            entry.subIndex = tmpl.subIndex;     ///< 继承对象字典子索引
            entry.offsetInPdo = tmpl.offsetInPdo; ///< 继承 PDO 数据帧内偏移
            entry.size = tmpl.size;             ///< 继承数据字节大小
            entry.motorFieldOffset = tmpl.motorFieldOffset; ///< 继承 `Motor` 结构体字段偏移

            finalTable.push_back(entry); // 存入最终映射表
        }
    }

    return finalTable;
}


#endif // PDO_CONFIG_HPP
