#include <cstdint>
#include <array>
#include <string>
#include <cmath>
#include <mutex>
#include <atomic>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <chrono>
#include <stdint.h>
#include <type_traits>


#include "CLASS_Motor.hpp"
#include "Test_module.hpp"
#include "Data_processing.hpp"






int main(){


    std::cout << 1 << std::endl;
    /**/
    std::array<Motor, 6> motors = {
        Motor(1),  // ���ID 1
        Motor(2),  // ���ID 2
        Motor(3),  // ���ID 3
        Motor(4),  // ���ID 4
        Motor(5),  // ���ID 5
        Motor(6)   // ���ID 6
    };
    // 2. �������б�־��ȷ��֮ǰ��״̬�������
    running = true;
    // 3. ���ò��Ժ���������5��
    std::cout << "=== Starting Motor Class Test ===\n";
    testMotorClass(motors, 0.5);  // �ڶ��������ǲ��Գ���ʱ��(��)






	return 0;
}