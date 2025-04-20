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





/**
 * @brief 将输入值转换为目标范围的值，并返回至少两位小数的结果
 * @tparam T 输入值类型（可以是整数或浮点数）
 * @param value 输入值
 * @param lowerBound 下限
 * @param upperBound 上限
 * @return double 类型的转换后值，至少保留两位小数
 */
template <typename T>
double convertValueWithBounds(T value, double lowerBound, double upperBound) {
    static_assert(std::is_arithmetic<T>::value, "输入值必须是数值类型");

    if (upperBound == lowerBound) {
        throw std::invalid_argument("上限和下限不能相同");
    }

    // 计算比例因子
    double scaleFactor = (upperBound - lowerBound) / 2.0;

    // 计算转换后的值
    double result = lowerBound + (value * scaleFactor);

    // 保留至少两位小数
    return std::floor(result * 100.0) / 100.0;
}





/**
 * @file Data_processing.hpp
 * @brief 数据转换相关函数集
 */





 /**
  * @brief 通用数据转换模板类
  * @tparam T 目标类型（必须为整型）
  */
template <typename T, typename = std::enable_if_t<std::is_integral_v<T>>>
class DataConverter {
public:
    // 将原始字节数组转换为目标类型（考虑字节序）
    static T bytesToValue(const uint8_t* bytes, bool big_endian = true) {
        T value = 0;
        if (big_endian) {
            for (size_t i = 0; i < sizeof(T); ++i) {
                value |= static_cast<T>(bytes[i]) << (8 * (sizeof(T) - 1 - i));
            }
        }
        else {
            for (size_t i = 0; i < sizeof(T); ++i) {
                value |= static_cast<T>(bytes[i]) << (8 * i);
            }
        }
        return value;
    }
     
    // 将目标类型转回字节数组（默认小端格式）
    static void valueToBytes(T value, uint8_t* output, bool big_endian = true) {
        if (big_endian) {
            for (size_t i = 0; i < sizeof(T); ++i) {
                output[i] = static_cast<uint8_t>((value >> (8 * (sizeof(T) - 1 - i))) & 0xFF);
            }
        }
        else {
            for (size_t i = 0; i < sizeof(T); ++i) {
                output[i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFF);
            }
        }
    }
};

/**
 * @brief 电流转换特化（q15格式）
 */
template <>
class DataConverter<float> {
public:
    static constexpr float Q15_SCALE = 32767.0f;

    static float bytesToCurrent(const uint8_t* bytes) {
        int16_t raw = DataConverter<int16_t>::bytesToValue(bytes);
        return static_cast<float>(raw) / Q15_SCALE;
    }

    static void currentToBytes(float value, uint8_t* output) {
        int16_t raw = static_cast<int16_t>(value * Q15_SCALE);
        DataConverter<int16_t>::valueToBytes(raw, output);
    }
};

/**
 * @brief




 





#endif // DATA_PROCESSING_HPP

