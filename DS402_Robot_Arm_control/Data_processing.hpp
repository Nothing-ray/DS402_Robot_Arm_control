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

���ݴ���
��Ҫ���ڸ�����ֵת������ӡ�����ԣ�ƴװ

*/





/**
 * @brief ������ֵת��ΪĿ�귶Χ��ֵ��������������λС���Ľ��
 * @tparam T ����ֵ���ͣ������������򸡵�����
 * @param value ����ֵ
 * @param lowerBound ����
 * @param upperBound ����
 * @return double ���͵�ת����ֵ�����ٱ�����λС��
 */
template <typename T>
double convertValueWithBounds(T value, double lowerBound, double upperBound) {
    static_assert(std::is_arithmetic<T>::value, "����ֵ��������ֵ����");

    if (upperBound == lowerBound) {
        throw std::invalid_argument("���޺����޲�����ͬ");
    }

    // �����������
    double scaleFactor = (upperBound - lowerBound) / 2.0;

    // ����ת�����ֵ
    double result = lowerBound + (value * scaleFactor);

    // ����������λС��
    return std::floor(result * 100.0) / 100.0;
}











#endif // DATA_PROCESSING_HPP

