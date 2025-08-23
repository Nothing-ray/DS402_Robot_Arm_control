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
#include <locale>
#include <codecvt>

#include "CLASS_Motor.hpp"
#include "Test_module.hpp"
#include "Data_processing.hpp"
#include "CAN_processing.hpp"






int main(){

	std::locale::global(std::locale(""));  // 使用系统默认locale
	std::wcout.imbue(std::locale());


	/*  
         //电机类测试
        // 创建电机实例数组   
    std::array<Motor, 6> motors = {
        Motor(1), Motor(2), Motor(3),
        Motor(4), Motor(5), Motor(6)
    };

    // 执行测试
    testMotorClass(motors);
    */

    testTPDOProcessing();


	return 0;
}