/**
 * @file test_cob_id_calculation.cpp
 * @brief 简单的COB-ID计算验证程序
 */

#include <iostream>
#include <iomanip>
#include "PDO_config.hpp"

int main() {
    std::cout << "=== COB-ID计算验证 ===" << std::endl;

    for (uint8_t motorIndex = 1; motorIndex <= 6; ++motorIndex) {
        for (uint8_t pdoIndex = 1; pdoIndex <= 2; ++pdoIndex) {
            uint32_t cobId = toRpdoCobId(motorIndex, pdoIndex);
            std::cout << "电机" << static_cast<int>(motorIndex)
                      << " PDO" << static_cast<int>(pdoIndex)
                      << ": 0x" << std::hex << cobId << std::dec << std::endl;
        }
    }

    std::cout << "=== 验证完成 ===" << std::endl;
    return 0;
}