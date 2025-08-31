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
#include "Data_processing.hpp"
#include "CAN_processing.hpp"

#include "test/test_CLASS_Motor.hpp"
#include "test/test_PDO_config.hpp"
#include "test/test_PDO_processing.hpp"
#include "test/test_SDO_State_Machine.hpp"




int main(){

	std::locale::global(std::locale(""));  // 使用系统默认locale
	std::wcout.imbue(std::locale());


	 
         //电机类测试
        // 创建电机实例数组   
    std::array<Motor, 6> motors = {
        Motor(1), Motor(2), Motor(3),
        Motor(4), Motor(5), Motor(6) };
    

    // 执行测试
    //testMotorClass(motors);
    

    // 测试注释说明：
    // testMotorClass(motors) - 电机类全面测试
    // testPDOConfiguration() - PDO配置测试  
    // testTPDOProcessing() - TPDO处理测试
    // testSdoStateMachinePerformance() - SDO状态机单线程性能测试
    // testSDOStateMachineMultiThreadPerformance() - SDO状态机多线程性能测试
    // testSdoTimeoutRetryMechanism() - SDO超时重传机制测试
    
    //testPDOConfiguration();
    //testTPDOProcessing();
    //testSdoStateMachinePerformance();
    //testSDOStateMachineMultiThreadPerformance();
    //testSdoTimeoutRetryMechanism();

    testSdoTimeoutRetryMechanism();



	return 0;
}