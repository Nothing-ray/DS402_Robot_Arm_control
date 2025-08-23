/**
 * @file PDO_config.hpp
 * @brief 定义CANopen PDO映射配置和COB-ID转换工具
 *
 * @details 本文件实现了机械臂驱动程序的PDO（Process Data Object）配置管理模块，
 * 提供运行时PDO映射表的构建机制和CANopen通信对象标识符的转换功能。该设计支持
 * 多电机系统的批量配置，通过模板化映射条目实现配置的灵活性和可扩展性。
 *
 * 主要功能包括：
 * - 定义PDO映射条目模板和运行时映射表结构
 * - 提供基于电机数量的批量PDO映射表构建函数
 * - 实现SDO和PDO通信对象标识符的标准化转换
 * - 支持RPDO（接收PDO）和TPDO（发送PDO）的双向映射配置
 * - 为多机械臂系统提供可扩展的配置管理接口
 *
 * @note 该模块采用编译期模板定义与运行时动态构建相结合的方式，
 * 既保证了配置的灵活性，又提供了良好的性能表现。COB-ID转换遵循
 * CANopen标准规范，支持最多12个电机节点的配置。
 *
 * @par 设计特点：
 * - 使用宏定义简化PDO映射条目的声明
 * - 通过offsetof运算符实现Motor类成员的高效访问
 * - 采用预分配内存策略优化映射表构建性能
 * - 支持1-based电机编号规范，符合工业设备惯例
 * - 提供异常安全的COB-ID转换接口
 *
 * @par 扩展性说明：
 * - 易于添加新的PDO映射条目和配置模板
 * - 支持不同机械臂配置的独立映射表管理
 * - 可适配不同厂商电机的PDO映射需求
 */







#pragma once
#ifndef PDO_CONFIG_HPP
#define PDO_CONFIG_HPP

#include <cstdint>
#include <vector>
#include <stdexcept>
#include "class_motor.hpp"



#define TPDO true
#define RPDO false

// ====================== 编译期配置宏 ====================== //
// 通过定义这些宏可以集中配置PDO映射关系


/** 
 * @brief PDO映射条目模板（不包含具体电机ID） MAP_ENTRY 定义
 * @param isTx          方向(true=TPDO, false=RPDO)
 * @param pdoIndex      PDO通道号(1-4)
 * @param index         对象字典索引  目标要写入的地址
 * @param subIndex      对象字典子索引 写入地址的子索引
 * @param offsetInPdo   在PDO数据帧中的字节偏移
 * @param size          数据长度(字节)
 * @param motorFieldOffset Motor类中对应字段的偏移量 用于进行线性查找
 */

/*  RPDO: 上位机→下位机   TPDO:下位机→上位机  */

// RPDO1 配置 (0x200 + NodeID)
// 0-4 目标位置  4-6 控制字
#define RPDO1_MAPPINGS \
    MAP_ENTRY(RPDO, 1, OD_TARGET_POSITION, 0x00, 0, 4, offsetof(Motor, position.raw_actual)) \
    MAP_ENTRY(RPDO, 1, OD_CONTROL_WORD,    0x00, 4, 2, offsetof(Motor, stateAndMode.controlData.statusWordRaw))

// RPDO2 配置 (0x300 + NodeID)
// 0-2 目标速度 2-4 目标电流
#define RPDO2_MAPPINGS \
    MAP_ENTRY(RPDO, 2, OD_TARGET_VELOCITY_VELOCITY_MODE, 0x00, 0, 2, offsetof(Motor, velocity.raw_target_position_mode)) \
    MAP_ENTRY(RPDO, 2, OD_TARGET_CURRENT,  0x00, 2, 2, offsetof(Motor, current.raw_target))

// TPDO1 配置 (0x180 + NodeID)
// 0-4 实际位置  4-6状态字
#define TPDO1_MAPPINGS \
    MAP_ENTRY(TPDO,  1, OD_ACTUAL_POSITION, 0x00, 0, 4, offsetof(Motor, position.raw_actual)) \
    MAP_ENTRY(TPDO,  1, OD_STATUS_WORD,     0x00, 4, 2, offsetof(Motor, stateAndMode.controlData.statusWordRaw))

// TPDO2 配置 (0x280 + NodeID)
// 0-2 实际速度 2-4实际电流
#define TPDO2_MAPPINGS \
    MAP_ENTRY(TPDO,  2, OD_ACTUAL_VELOCITY, 0x00, 0, 2, offsetof(Motor, velocity.raw_actual)) \
    MAP_ENTRY(TPDO,  2, OD_ACTUAL_CURRENT,  0x00, 2, 2, offsetof(Motor, current.raw_actual))

// ====================== 核心数据结构 ====================== //

/**
 * @brief PDO映射条目模板（不包含具体电机ID）
 * @param isTx          方向(true=TPDO, false=RPDO)
 * @param pdoIndex      PDO通道号(1-4)
 * @param index         对象字典索引  目标要写入的地址
 * @param subIndex      对象字典子索引 写入地址的子索引
 * @param offsetInPdo   在PDO数据帧中的字节偏移
 * @param size          数据长度(字节)
 * @param motorFieldOffset Motor类中对应字段的偏移量 用于进行线性查找
 */
#define MAP_ENTRY(isTx, pdoIndex, index, subIndex, offsetInPdo, size, motorFieldOffset) \
    {isTx, pdoIndex, index, subIndex, offsetInPdo, size, motorFieldOffset},


//将编译宏转化为结构体
struct PdoMappingTemplate {
    bool     isTx;              ///< 方向(true: TPDO, false: RPDO)
    uint8_t  pdoIndex;          ///< PDO通道号(1-4)
    uint16_t index;             ///< 对象字典索引
    uint8_t  subIndex;          ///< 对象字典子索引
    uint8_t  offsetInPdo;       ///< 在PDO数据帧中的偏移(字节)
    uint8_t  size;              ///< 数据长度(字节)
    size_t   motorFieldOffset;  ///< Motor类字段偏移
};

/**
 * @brief 完整PDO映射条目（包含电机ID）
 */
struct PdoMappingEntry {
    uint8_t  motorIndex;        ///< 电机逻辑编号(1-N)
    bool     isTx;              ///< 方向(true: TPDO, false: RPDO)
    uint8_t  pdoIndex;          ///< PDO通道号(1-4)
    uint16_t index;             ///< 对象字典索引
    uint8_t  subIndex;          ///< 对象字典子索引
    uint8_t  offsetInPdo;       ///< 在PDO数据帧中的偏移(字节)
    uint8_t  size;              ///< 数据长度(字节)
    size_t   motorFieldOffset;  ///< Motor类字段偏移
};

// ====================== 硬编码配置 ====================== //

/**
 * @brief 获取默认的单电机PDO映射模板
 * @return 包含所有预定义映射的const vector
 * @note 将编译宏整理成向量便于后面批处理 
 */
inline const std::vector<PdoMappingTemplate>& getDefaultMotorTemplate() {
    static const std::vector<PdoMappingTemplate> singleMotorTemplate = {
        // 展开所有预定义的映射
        RPDO1_MAPPINGS
        RPDO2_MAPPINGS
        TPDO1_MAPPINGS
        TPDO2_MAPPINGS
    };
    return singleMotorTemplate;
}

// ====================== 运行时构建 ====================== //

/**
 * @brief 构建完整的机械臂PDO映射表
 * @param motorCount 机械臂上的电机数量（范围：1~12，受限于CANopen节点地址规范）
 * @return 包含所有电机完整PDO映射的列表，每个电机的映射基于预先定义的模板扩展
 *
 * @note 此函数核心设计思想：基于单个电机的标准PDO映射模板，通过复制扩展实现机械臂多电机的批量映射。
 *       这种设计提供良好的扩展性，新增电机时只需简单倍增模板条目。
 */
inline std::vector<PdoMappingEntry> buildArmMappingTable(uint8_t motorCount) {
    // 最终生成的完整映射表容器
    std::vector<PdoMappingEntry> finalTable;

    // 步骤1：获取单个电机的标准PDO映射模板
    // 此处使用单例模式的const引用避免重复构造模板数据
    const auto& templateTable = getDefaultMotorTemplate();

    // 步骤2：预分配最终容器内存空间（重要性能优化）
    // 计算公式：总电机数 × 单电机模板条目数 → 预分配准确内存避免动态扩容开销
    // 例如：3个电机 × 每条PDO通道2个条目 × 4个PDO → 3×8=24条目
    finalTable.reserve(motorCount * templateTable.size());

    // 步骤3：电机循环 - 遍历每个需要配置的电机
    // motorIndex从1开始，遵循工业设备常见1-based编号规范（如CANopen节点1~12）
    // 注意：motorIndex=0是保留地址，部分硬件使用0作为广播地址
    for (uint8_t motorIndex = 1; motorIndex <= motorCount; ++motorIndex) {
        // 步骤3.1：模板循环 - 为当前电机复制所有模板条目
        for (const auto& tmpl : templateTable) {
            // 步骤3.1.1：构造完整映射条目
            // 核心操作：将模板条目与具体的电机ID结合，生成CAN网络中的真实映射项
            finalTable.push_back({
                motorIndex,             // 填充实际电机ID，此值将影响最终COB-ID生成
                tmpl.isTx,              // 继承模板方向属性（true=TPDO，false=RPDO）
                tmpl.pdoIndex,          // PDO通道编号（如RPDO1=1, TPDO2=2等）
                tmpl.index,             // 对象字典索引值（如0x607A=目标位置）
                tmpl.subIndex,          // 对象字典子索引（通常为0x00）
                tmpl.offsetInPdo,       // 数据在PDO报文中的字节偏移（需对齐内存布局）
                tmpl.size,              // 数据长度（字节）必需与OD定义一致
                tmpl.motorFieldOffset   // Motor类成员偏移，用于与PDO数据内存拷贝
                });
            // 示例：当motorIndex=3时，模板条目会被实例化为：
            // {3, false, 1, 0x607A, 0x00, 0, 4, offsetof(Motor, position.raw_target)}
        }
    }

    // 步骤4：返回生成的完整映射表
    // 此表可被上层用于：
    // 1. 初始化CANopen栈的PDO映射配置
    // 2. 构造PDO数据包时快速查找映射关系
    // 3. 将收到/发送的PDO数据与具体电机的内存区域关联
    return finalTable;
}


// ====================== COB-ID转换工具 ====================== //

inline uint16_t toSdoMotorId(uint8_t motorIndex) {
    if (motorIndex == 0 || motorIndex > 12) {
        throw std::out_of_range("motorIndex must be 1..12");
    }
    return uint16_t(0x600 + motorIndex);
}

inline uint32_t toRpdoCobId(uint8_t motorIndex, uint8_t pdoIndex) {
    switch (pdoIndex) {
    case 1: return 0x200 + motorIndex;
    case 2: return 0x300 + motorIndex;
    case 3: return 0x400 + motorIndex;
    case 4: return 0x500 + motorIndex;
    default: throw std::out_of_range("Invalid RPDO index (1-4)");
    }
}

inline uint32_t toTpdoCobId(uint8_t motorIndex, uint8_t pdoIndex) {
    switch (pdoIndex) {
    case 1: return 0x180 + motorIndex;
    case 2: return 0x280 + motorIndex;
    case 3: return 0x380 + motorIndex;
    case 4: return 0x480 + motorIndex;
    default: throw std::out_of_range("Invalid TPDO index (1-4)");
    }
}

#endif // PDO_CONFIG_HPP
