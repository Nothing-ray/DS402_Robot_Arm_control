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
	testCANProcessing();


	return 0;
}