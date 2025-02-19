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





 





#endif // DATA_PROCESSING_HPP

