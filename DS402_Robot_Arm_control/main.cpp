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
        Motor(1),  // 电机ID 1
        Motor(2),  // 电机ID 2
        Motor(3),  // 电机ID 3
        Motor(4),  // 电机ID 4
        Motor(5),  // 电机ID 5
        Motor(6)   // 电机ID 6
    };
    // 2. 重置运行标志（确保之前的状态被清除）
    running = true;
    // 3. 调用测试函数，测试5秒
    std::cout << "=== Starting Motor Class Test ===\n";
    testMotorClass(motors, 0.5);  // 第二个参数是测试持续时间(秒)






	return 0;
}