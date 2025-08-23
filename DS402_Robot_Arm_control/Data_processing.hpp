/**
 * @file Data_processing.hpp
 * @brief 提供数据转换、格式化和调试辅助函数的工具模块
 *
 * @details 本文件实现了机械臂驱动程序中常用的数据处理功能，
 * 包括编码器与角度值的双向转换、字节序处理、CAN帧调试打印等
 * 核心工具函数。该模块采用模板化设计和内联优化，确保高性能
 * 的数据处理能力，适用于实时控制系统中的各种数值转换需求。
 *
 * 主要功能包括：
 * - 编码器值与物理角度的精确双向转换
 * - 多字节整数与字节数组的高效互转
 * - 支持大小端字节序的灵活配置
 * - CAN帧二进制数据的格式化调试输出
 * - 数值归一化和边界检查机制
 *
 * @note 该模块处于开发阶段，部分功能仍在完善中。所有函数均
 * 采用内联实现以优化性能，支持8/16/32/64位整型数据处理。
 * 角度转换函数内置[-180,180]范围归一化，确保数据一致性。
 *
 * @par 设计特点：
 * - 使用C++17 constexpr和if constexpr提升编译期优化
 * - 模板特化处理常见数据类型以提高运行效率
 * - 内置类型安全检查防止非法数据转换
 * - 调试函数采用RAII风格保护流状态
 * - 支持可配置的小数位数和舍入策略
 *
 * @par 性能优化：
 * - 16/32位整型转换使用无分支特化处理
 * - 角度归一化采用高效模运算算法
 * - 字节序转换避免不必要的循环操作
 * - 调试打印函数最小化状态切换开销
 *
 * @par 待完善功能：
 * - 增加更多传感器数据类型的转换支持
 * - 扩展调试工具函数集
 * - 优化大数据量处理性能
 * - 增加单元测试覆盖
 *
 * @warning 部分调试功能仅在DEBUG_MODE下启用，
 * 生产环境中应关闭相关输出以提升性能。
 *
 * @see CLASS_Motor.hpp 电机数据结构中使用角度转换功能
 * @see CAN_frame.hpp   CAN帧处理中使用字节序转换功能
 */




#ifndef DATA_PROCESSING_HPP
#define DATA_PROCESSING_HPP

#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <chrono>
#include <stdint.h>
#include <iostream>
#include <cmath>
#include <type_traits>
#include <cstdint>
#include <atomic>
#include <iomanip>
#include <string>

/*************************************/
/*

数据处理
主要用于各种数值转换，打印，调试，拼装

*/

#define SENSOR_RANGE 32768   //传感器的范围


/**
 * @brief 编码器值与角度值的双向转换函数（支持对称范围）
 *
 * @param value 输入值
 * @param toAngle true表示编码器→角度，false表示角度→编码器
 * @param sensorRange 编码器量程（默认32768）
 * @param decimalPlaces 保留小数位数（默认2位，仅toAngle=true有效）
 * @param round 是否四舍五入（默认true）
 * @return double 转换结果
 *
 * @note 1. 编码器范围: -32767到32768对应-180度到180度
 *       2. 角度→编码器转换时，返回值会截断为整数
 *       3. 角度输入会自动归一化到[-180,180]范围
 */
inline double convertSensorAngle(
    double value,
    bool toAngle = true,
    int32_t sensorRange = SENSOR_RANGE,
    int decimalPlaces = 2,
    bool round = true
) {
    const double scale = 180.0 / sensorRange;
    if (toAngle) {
        // 编码器→角度转换
        double result = value * scale;
        // 处理小数位数
        if (decimalPlaces >= 0) {
            const double factor = std::pow(10.0, decimalPlaces);
            return round ?
                std::round(result * factor) / factor :
                std::floor(result * factor) / factor;
        }
        return result;
    }
    else {
        // 角度→编码器转换
        // 归一化角度到[-180,180]范围
        double normalizedAngle = std::fmod(value, 360.0);
        if (normalizedAngle > 180.0) {
            normalizedAngle -= 360.0;
        }
        else if (normalizedAngle < -180.0) {
            normalizedAngle += 360.0;
        }
        // 计算编码器值
        double sensorValue = normalizedAngle / scale;

        // 确保不超出编码器范围
        sensorValue = std::max(sensorValue, static_cast<double>(-sensorRange));
        sensorValue = std::min(sensorValue, static_cast<double>(sensorRange));

        return round ?
            std::round(sensorValue) :
            std::floor(sensorValue);
    }
}








/**
 * @file Data_processing.hpp
 * @brief 数据转换相关函数集
 */

 /**
  * @brief 将字节数组转换为指定类型的整数值
  * @tparam T 目标整数类型（uint8_t/int16_t/uint32_t等）
  * @param[in] bytes 源字节数组指针（必须至少包含sizeof(T)字节）
  * @param[out] value 转换结果输出
  * @param[in] big_endian 是否使用大端字节序（默认小端）
  *
  * @note 功能特性：
  * - 支持8/16/32/64位整型（通过static_assert限制）
  * - 自动处理字节序转换
  * - 对16/32位类型使用无分支优化
  * - 类型安全检查（仅允许整型）
  */
template <typename T>
inline void bytesToValue(const uint8_t* bytes, T& value, bool big_endian = false)
{
    // 编译期类型检查：目标类型必须是整数且不超过64位
    static_assert(std::is_integral_v<T>, "Target type must be integral");
    static_assert(sizeof(T) <= 8, "Max 64-bit supported");

    // 16位类型特化处理（无分支优化）
    if constexpr (sizeof(T) == 2) {

        value = big_endian
            ? (static_cast<T>(bytes[0]) << 8) | bytes[1]
            : (static_cast<T>(bytes[1]) << 8) | bytes[0];
    }
    // 32位类型特化处理（无分支优化）  
    else if constexpr (sizeof(T) == 4) {
        value = big_endian
            ? (static_cast<T>(bytes[0]) << 24) | (static_cast<T>(bytes[1]) << 16)
            | (static_cast<T>(bytes[2]) << 8) | bytes[3]
            : (static_cast<T>(bytes[3]) << 24) | (static_cast<T>(bytes[2]) << 16)
            | (static_cast<T>(bytes[1]) << 8) | bytes[0];
    }
    // 其他类型（8/64位）通用处理
    else {
        // 初始化输出值
        value = 0;

        for (size_t i = 0; i < sizeof(T); ++i) {
            // 计算当前字节的数组索引（考虑字节序）
            const size_t idx = big_endian ? sizeof(T) - 1 - i : i;

            // 字节移位并合并（注意移位次数为8*i而不是8*idx）
            value |= static_cast<T>(bytes[idx]) << (8 * i);
        }
    }
}


/**
 * @brief 将整数值转换为字节数组
 * @tparam T 源整数类型（uint8_t/int16_t/uint32_t等）
 * @param[in] value 待转换的整数值
 * @param[out] bytes 输出缓冲区（必须至少能容纳sizeof(T)字节）
 * @param[in] big_endian 是否使用大端字节序（默认小端）  True 大端 False 小端
 *
 * @note 功能特性：
 * - 支持8/16/32/64位整型
 * - 自动处理字节序转换
 * - 16/32位类型使用无分支优化
 * - 高位自动截断（安全处理整数降级）
 *
 * */
template <typename T>
inline void valueToBytes(T value, uint8_t* bytes, bool big_endian = false)
{
    // 编译期类型检查
    static_assert(std::is_integral_v<T>, "Source type must be integral");
    static_assert(sizeof(T) <= 8, "Max 64-bit supported");

     // 16位类型特化处理
    if constexpr (sizeof(T) == 2) {
        /*
         * 内存布局控制：
         * 大端序：高字节 → bytes[0], 低字节 → bytes[1]
         * 小端序：低字节 → bytes[0], 高字节 → bytes[1]
         */
        bytes[big_endian ? 0 : 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
        bytes[big_endian ? 1 : 0] = static_cast<uint8_t>(value & 0xFF);
    }
    // 32位类型特化处理
    else if constexpr (sizeof(T) == 4) {
        bytes[big_endian ? 0 : 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
        bytes[big_endian ? 1 : 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
        bytes[big_endian ? 2 : 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
        bytes[big_endian ? 3 : 0] = static_cast<uint8_t>(value & 0xFF);
    }
    // 通用处理（支持8/64位）
    else {
        for (size_t i = 0; i < sizeof(T); ++i) {
            const size_t shift = big_endian ?
                8 * (sizeof(T) - 1 - i) :
                8 * i;
            bytes[i] = static_cast<uint8_t>((value >> shift) & 0xFF);
        }
    }
}




/**
 * @brief 打印CAN帧二进制数据的调试信息
 *
 * @param binaryData 要打印的二进制数据向量，格式应符合CAN帧规范：
 *                   [0]   - 帧信息字节（包含DLC和标志位）
 *                   [1-4] - 帧ID（大端序）
 *                   [5-12]- 数据域（实际长度由DLC决定）
 *
 * @note 输出格式示例：
 *       [DEBUG] Frame BinaryData: 02 00 00 01 23 aa bb 00 00 00 00 00 00
 *       其中：
 *       - 02      : 帧信息（DLC=2）
 *       - 00000123: 帧ID=0x123
 *       - aabb    : 数据字节
 *       - 00...   : 填充字节
 *
 * @warning 此函数仅用于调试目的，会修改cout的格式状态（hex/dec）
 */
inline void PrintCANbinaryData(const std::vector<uint8_t>& binaryData)
{
    // 保存当前流状态以便恢复（RAII风格）
    std::ios oldState(nullptr);
    oldState.copyfmt(std::cout);

    // 输出调试头
    std::cout << "[DEBUG] Frame BinaryData: ";

    // 遍历并打印每个字节
    for (uint8_t byte : binaryData) {
        std::cout << std::hex << std::uppercase           // 16进制大写输出
            << std::setw(2)          // 固定2字符宽度
            << std::setfill('0')     // 前导零填充
            << static_cast<int>(byte)// 避免char类型特殊处理
            << " ";
    }

    // 恢复十进制输出并换行
    std::cout << std::nouppercase << std::dec << std::endl;  // 恢复小写状态
    // 恢复原始流格式
    std::cout.copyfmt(oldState);
    return;
}








#endif // DATA_PROCESSING_HPP

