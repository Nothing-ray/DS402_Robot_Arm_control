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

/*************************************/
/*

数据处理
主要用于各种数值转换，打印，调试，拼装

*/

#define SENSOR_RANGE 32768   //传感器的范围


/**
 * @brief 编码器值与角度值的双向转换函数
 *
 * @param value 输入值
 * @param sensorRange 编码器量程（默认32768）
 * @param toAngle true表示编码器→角度，false表示角度→编码器
 * @param decimalPlaces 保留小数位数（默认2位，仅toAngle=true有效）
 * @param round 是否四舍五入（默认true）
 * @return double 转换结果
 *
 * @note 角度→编码器转换时，返回值会截断为整数
 * @warning 角度输入超过360度时会自动取模
 */
inline double convertSensorAngle(
    double value,
    bool toAngle = true,
    int32_t sensorRange = SENSOR_RANGE,
    int decimalPlaces = 2,
    bool round = true
) {
    const double scale = 360.0 / sensorRange;

    if (toAngle) {
        // 编码器→角度转换
        double result = value * scale;

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
        double normalizedAngle = std::fmod(value, 360.0);
        if (normalizedAngle < 0) normalizedAngle += 360.0;

        double sensorValue = normalizedAngle / scale;
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





 





#endif // DATA_PROCESSING_HPP

