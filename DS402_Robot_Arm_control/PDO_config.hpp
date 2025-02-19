#pragma once
#ifndef PDO_CONFIG_HPP
#define PDO_CONFIG_HPP

#include <cstdint>
#include <cstddef>
#include <vector>
#include <stdexcept>
#include <string>
#include "class_motor.hpp"

/*****************************************************************************
 * 1) 可选: ID 换算函数
 *    如果您想将 "逻辑电机编号" -> "SDO/PDO ID",
 *    可以在此定义，也可以在别处定义
 ****************************************************************************/
inline uint16_t toSdoMotorId(uint8_t motorIndex)
{
    // 例如： motorIndex=1 => 0x601, 2=>0x602 ...
    // 如果需要更灵活的映射，可自行修改
    if (motorIndex == 0 || motorIndex > 12) {
        throw std::out_of_range("motorIndex must be 1..12");
    }
    return uint16_t(0x600 + motorIndex);
}

// RPDO cobId 计算示例
inline uint32_t toRpdoCobId(uint8_t motorIndex, uint8_t pdoIndex)
{
    if (pdoIndex == 1) {
        return 0x200 + motorIndex;
    }
    else if (pdoIndex == 2) {
        return 0x300 + motorIndex;
    }
    // ...
    throw std::out_of_range("Unsupported RPDO pdoIndex");
}

// TPDO cobId 计算示例
inline uint32_t toTpdoCobId(uint8_t motorIndex, uint8_t pdoIndex)
{
    if (pdoIndex == 1) {
        return 0x180 + motorIndex;
    }
    else if (pdoIndex == 2) {
        return 0x280 + motorIndex;
    }
    // ...
    throw std::out_of_range("Unsupported TPDO pdoIndex");
}

/*****************************************************************************
 * 2) 模板结构：表示“单个电机”常见的固定映射
 ****************************************************************************/

 /**
  * @brief 一条“单电机模板”中，描述一个要映射的 DS402 对象
  *
  * 这不直接存 motorIndex，而是说“对于RPDO1, 偏移=0, 映射 OD_TARGET_POSITION”。
  * 稍后在初始化期，会基于这些模板复制并补充“motorIndex”等实际信息。
  */
struct PdoMappingTemplate
{
    bool     isTx;         ///< true=TPDO, false=RPDO
    uint8_t  pdoIndex;     ///< PDO编号(1..4)
    uint16_t index;        ///< 对象字典Index(如0x607A)
    uint8_t  subIndex;     ///< 对象字典subIndex(一般0)
    uint8_t  offsetInPdo;  ///< 在PDO帧中的起始字节
    uint8_t  size;         ///< 映射的字节数
    size_t   motorFieldOffset; ///< Motor类的字段偏移
};

/*****************************************************************************
 * 3) 最终的映射条目：带上 motorIndex, 用于实际读写
 ****************************************************************************/
struct PdoMappingEntry
{
    // 在初始化期生成
    uint8_t  motorIndex;
    bool     isTx;
    uint8_t  pdoIndex;
    uint16_t index;
    uint8_t  subIndex;
    uint8_t  offsetInPdo;
    uint8_t  size;
    size_t   motorFieldOffset;

    // 额外可记录 cobId, 传输类型, ...
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
 * 5) 在程序启动/机械臂初始化时: 根据 “机械臂实际有几个电机”
 *    复制“单电机模板” 生成 “最终映射表” (PdoMappingEntry[]).
 ****************************************************************************/

 /**
  * @brief 构建此“机械臂”上所有电机的PDO映射表
  *
  * @param motorCount   机械臂上电机个数(可能是 6, 7, 12...)
  * @return 一个拼好的“最终映射表”，其中 motorIndex=1..motorCount
  *         其余字段来自 getDefaultMotorTemplate()
  *
  * 注意：本函数仅示例, 若您每个电机都需要不同映射, 也可传入更多参数或写更复杂逻辑
  */
inline std::vector<PdoMappingEntry> buildArmMappingTable(uint8_t motorCount)
{
    std::vector<PdoMappingEntry> finalTable;
    finalTable.reserve(motorCount * getDefaultMotorTemplate().size());

    for (uint8_t m = 1; m <= motorCount; m++)
    {
        // 对第m个电机, 遍历模板
        for (auto& tmpl : getDefaultMotorTemplate())
        {
            PdoMappingEntry entry;
            entry.motorIndex = m;
            entry.isTx = tmpl.isTx;
            entry.pdoIndex = tmpl.pdoIndex;
            entry.index = tmpl.index;
            entry.subIndex = tmpl.subIndex;
            entry.offsetInPdo = tmpl.offsetInPdo;
            entry.size = tmpl.size;
            entry.motorFieldOffset = tmpl.motorFieldOffset;
            finalTable.push_back(entry);
        }
    }

    return finalTable;
}

#endif // PDO_CONFIG_HPP
