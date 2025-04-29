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
  * @brief 从字节数组读取整数值
  * @tparam T 目标整数类型
  * @param bytes 字节数组指针
  * @param[out] value 输出结果
  * @param big_endian 是否使用大端序(默认小端)
  *
  * @note 适用于所有标准整数类型(uint8_t,int16_t等)
  * @warning 需确保bytes缓冲区足够大(sizeof(T))
  */
template <typename T>
inline void bytesToValue(const uint8_t* bytes, T& value, bool big_endian = false)
{
    static_assert(std::is_integral_v<T>, "Target type must be integral");

    value = 0; // 初始化输出值

    // 按字节处理，根据字节序调整位移量
    for (size_t i = 0; i < sizeof(T); ++i)
    {
        const size_t shift = big_endian ?
            8 * (sizeof(T) - 1 - i) : // 大端序：高位在前
            8 * i;                   // 小端序：低位在前

        value |= static_cast<T>(bytes[i]) << shift;
    }
}

/**
 * @brief 将整数值写入字节数组
 * @tparam T 源整数类型
 * @param value 输入值
 * @param[out] bytes 输出缓冲区
 * @param big_endian 是否使用大端序(默认小端)
 *
 * @note 会自动截断超出字节范围的高位数据
 */
template <typename T>
inline void valueToBytes(T value, uint8_t* bytes, bool big_endian = false)
{
    static_assert(std::is_integral_v<T>, "Source type must be integral");

    // 逐个字节提取并存储
    for (size_t i = 0; i < sizeof(T); ++i)
    {
        const size_t shift = big_endian ?
            8 * (sizeof(T) - 1 - i) : // 大端序：高位在前
            8 * i;                   // 小端序：低位在前

        bytes[i] = static_cast<uint8_t>((value >> shift) & 0xFF);
    }
}


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





 





#endif // DATA_PROCESSING_HPP

