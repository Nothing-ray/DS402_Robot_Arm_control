#ifndef TEST_MODULE
#define TEST_MODULE

#include <array>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>


#include "CLASS_Motor.hpp"
#include "PDO_config.hpp"
#include "Data_processing.hpp"
#include "CAN_frame.hpp"
#include "CAN_processing.hpp"

using namespace std::chrono_literals;




/******************** Motor类测试 ********************/



// 全局控制标志
std::atomic<bool> running{ true };

// 计时工具宏
#define TIME_IT(operation, description) \
    do { \
        auto start = std::chrono::high_resolution_clock::now(); \
        operation; \
        auto end = std::chrono::high_resolution_clock::now(); \
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start); \
        std::cout << description << " took " << duration.count() << " us\n"; \
    } while(0)

/**
 * @brief 电机类测试函数
 * @param motors 电机实例数组
 * @param test_duration 测试持续时间(秒)
 */
void testMotorClass(std::array<Motor, 6>& motors, size_t test_duration = 5) {
    std::cout << "=== Starting Motor Class Test ===\n";
    std::cout << "Creating " << motors.size() << " motor instances...\n";

    // 初始化所有电机
    for (size_t i = 0; i < motors.size(); ++i) {
        TIME_IT(
            motors[i].init(),
            "Motor " + std::to_string(i + 1) + " initialization"
        );
    }

    // 创建线程容器
    std::vector<std::thread> threads;
    threads.reserve(motors.size() * 2); // 每个电机一个读线程一个写线程

    // 为每个电机创建读写线程
    for (auto& motor : motors) {
        // 写入线程
        threads.emplace_back([&motor]() {
            int counter = 0;
            while (running) {
                std::cout << "Motor " << static_cast<int>(motor.motor_id_)
                    << " write cycle " << counter << ":\n";
                  
                // 写入目标值并计时
                TIME_IT({
                    motor.current.target_current.store(100.0f + counter);
                    motor.position.target_degree.store(45.0f + counter);
                    motor.velocity.target_rpm.store(500.0f + counter);
                    }, "  Target value assignment");

                // 设置刷新标志
                TIME_IT({
                    motor.current.flags_.store(
                        MotorCurrent::Flags::TARGET_DATA_SEND_NEED_REFRESH);
                    motor.position.flags_.store(
                        MotorPosition::Flags::TARGET_DATA_SEND_NEED_REFRESH);
                    motor.velocity.flags_.store(
                        MotorVelocity::Flags::TARGET_DATA_SEND_NEED_REFRESH);
                    }, "  Flag setting");

                // 执行刷新并计时
                TIME_IT({
                    std::lock_guard<std::mutex> lock(motor.mtx_);
                    motor.refreshMotorData(motor.current);
                    motor.refreshMotorData(motor.position);
                    motor.refreshMotorData(motor.velocity);
                    }, "  Data refresh");

                counter++;
                std::this_thread::sleep_for(1ms);
            }
            });

        // 读取线程
        threads.emplace_back([&motor]() {
            int counter = 0;
            while (running) {
                std::cout << "Motor " << static_cast<int>(motor.motor_id_)
                    << " read cycle " << counter << ":\n";

                // 模拟接收数据
                TIME_IT({
                    std::lock_guard<std::mutex> lock(motor.mtx_);

                    int16_t current_val = 100 + counter;
                    motor.current.raw_actual.atomicWrite(&current_val);

                    int32_t position_val = 10000 + counter;
                    motor.position.raw_actual.atomicWrite(&position_val);

                    int16_t velocity_val = 500 + counter;
                    motor.velocity.raw_actual.atomicWrite(&velocity_val);

                    // 设置刷新标志
                    motor.current.flags_.store(
                        MotorCurrent::Flags::RAW_DATA_RECEIVE_NEED_REFRESH);
                    motor.position.flags_.store(
                        MotorPosition::Flags::RAW_DATA_RECEIVE_NEED_REFRESH);
                    motor.velocity.flags_.store(
                        MotorVelocity::Flags::RAW_DATA_RECEIVE_NEED_REFRESH);

                    // 执行刷新
                    motor.refreshMotorData(motor.current);
                    motor.refreshMotorData(motor.position);
                    motor.refreshMotorData(motor.velocity);
                    }, "  Data reception simulation");

                // 读取并验证数据
                float actual_current, actual_degree, actual_rpm;
                TIME_IT({
                    actual_current = motor.current.actual_current.load();
                    actual_degree = motor.position.actual_degree.load();
                    actual_rpm = motor.velocity.actual_rpm.load();
                    }, "  Data reading");

                std::cout << "  Read values - Current: " << actual_current
                    << " mA, Position: " << actual_degree
                    << "°, Velocity: " << actual_rpm << " RPM\n";

                counter++;
                std::this_thread::sleep_for(1ms);
            }
            });
    }

    // 运行测试指定时间
    std::cout << "\n=== Running test for " << test_duration << " seconds ===\n";
    auto test_start = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(std::chrono::seconds(test_duration));
    running = false;

    // 等待所有线程结束
    for (auto& t : threads) {
        if (t.joinable()) t.join();
    }

    // 测试总结
    auto test_end = std::chrono::high_resolution_clock::now();
    auto test_duration_us = std::chrono::duration_cast<std::chrono::microseconds>(test_end - test_start);

    std::cout << "\n=== Test Summary ===" << std::endl;
    std::cout << "Total test duration: " << test_duration_us.count() / 1000.0 << " ms" << std::endl;
    std::cout << "Number of motors tested: " << motors.size() << std::endl;

    // 验证最终数据
    for (auto& motor : motors) {
        std::cout << "\nMotor " << static_cast<int>(motor.motor_id_) << " final values:" << std::endl;

        // 读取转换后的值
        std::cout << "  Current: " << motor.current.actual_current.load() << " mA" << std::endl;
        std::cout << "  Position: " << motor.position.actual_degree.load() << "°" << std::endl;
        std::cout << "  Velocity: " << motor.velocity.actual_rpm.load() << " RPM" << std::endl;

        // 读取原始值并验证
        int16_t raw_current;
        motor.current.raw_actual.atomicRead(&raw_current);
        std::cout << "  Raw current: " << raw_current << std::endl;

        int32_t raw_position;
        motor.position.raw_actual.atomicRead(&raw_position);
        std::cout << "  Raw position: " << raw_position << std::endl;

        int16_t raw_velocity;
        motor.velocity.raw_actual.atomicRead(&raw_velocity);
        std::cout << "  Raw velocity: " << raw_velocity << std::endl;
    }
}




/******************** PDO配置测试 ********************/



/**
 * @brief 测试PDO配置系统的完整功能
 *
 * 本测试函数执行以下验证：
 * 1. 测试映射表初始化功能，包括构建时间和内容完整性
 * 2. 测试COB-ID转换工具的正确性和异常处理
 * 3. 使用高精度计时器测量关键操作性能
 *
 * 测试包含以下场景：
 * - 正常电机ID(1-12)的转换
 * - 边界值测试(最小/最大有效ID)
 * - 无效输入测试(0/13等非法ID)
 *
 * @note 测试结果将输出到标准输出，错误信息输出到标准错误
 * @warning 测试中将故意触发异常，相关错误信息属于预期行为
 */
void testPDOConfiguration() {
    std::cout << "=== PDO Configuration Test Begin ===\n\n";

    /* 第一阶段：映射表初始化测试 */
    std::vector<PdoMappingEntry> mappingTable;

    // 使用计时宏测量6电机映射表构建时间
    TIME_IT(
        mappingTable = buildArmMappingTable(6),
        "Build mapping table for 6 motors"
    );

    /* 打印映射表详细内容 */
    std::cout << "\n=== Mapping Table Content ===\n";
    for (const auto& entry : mappingTable) {
        // 格式化输出每个映射条目信息：
        // 1. 电机编号 | 2. PDO类型及通道 | 3. 对象字典地址
        // 4. 数据帧偏移 | 5. 数据大小 | 6. Motor类成员偏移
        std::cout << "Motor " << (int)entry.motorIndex
            << " | " << (entry.isTx ? "TPDO" : "RPDO") << (int)entry.pdoIndex
            << " | OD: 0x" << std::hex << entry.index << std::dec << "/" << (int)entry.subIndex
            << " | Frame offset: " << (int)entry.offsetInPdo
            << " | Size: " << (int)entry.size << " bytes"
            << " | Motor offset: " << entry.motorFieldOffset
            << "\n";
    }
    std::cout << "Total entries: " << mappingTable.size() << "\n\n";

    /* 第二阶段：COB-ID转换工具测试 */
    std::cout << "=== COB-ID Conversion Test ===\n";

    // 定义COB-ID测试lambda函数，封装重复测试逻辑
    auto testCobIdConversion = [](uint8_t motorId) {
        std::cout << "For Motor " << (int)motorId << ":\n";

        try {
            // 测试SDO ID转换
            std::cout << "  SDO ID: 0x" << std::hex << toSdoMotorId(motorId) << std::dec << "\n";

            // 测试所有PDO通道(1-4)的RPDO/TPDO转换
            for (uint8_t pdo = 1; pdo <= 4; ++pdo) {
                std::cout << "  RPDO" << (int)pdo << ": 0x" << std::hex
                    << toRpdoCobId(motorId, pdo) << std::dec << "\n";
                std::cout << "  TPDO" << (int)pdo << ": 0x" << std::hex
                    << toTpdoCobId(motorId, pdo) << std::dec << "\n";
            }
        }
        catch (const std::exception& e) {
            // 捕获并报告转换异常
            std::cerr << "  Error: " << e.what() << "\n";
        }
        };

    /* 测试用例1：正常值测试（电机1） */
    TIME_IT(
        testCobIdConversion(1),
        "COB-ID conversion for motor 1"
    );

    /* 测试用例2：边界值测试（电机12，最大值） */
    std::cout << "\nTesting boundary values:\n";
    TIME_IT(
        testCobIdConversion(12),
        "COB-ID conversion for motor 12 (max)"
    );

    /* 测试用例3：异常值测试 */
    std::cout << "\nTesting invalid values:\n";

    // 子用例3.1：电机ID为0（非法值）
    try {
        TIME_IT(
            toSdoMotorId(0),
            "Invalid motor ID 0"
        );
    }
    catch (const std::exception& e) {
        std::cerr << "Expected error: " << e.what() << "\n";
    }

    // 子用例3.2：电机ID为13（超出上限）
    try {
        TIME_IT(
            toRpdoCobId(13, 1),
            "Invalid motor ID 13"
        );
    }
    catch (const std::exception& e) {
        std::cerr << "Expected error: " << e.what() << "\n";
    }

    // 子用例3.3：PDO索引为5（非法值）
    try {
        TIME_IT(
            toTpdoCobId(1, 5),
            "Invalid PDO index 5"
        );
    }
    catch (const std::exception& e) {
        std::cerr << "Expected error: " << e.what() << "\n";
    }

    std::cout << "\n=== PDO Configuration Test Complete ===\n";
}




/*********************** PDO读写测试区 **********************/



// ===================== RPDO1测试帧 (0x200 + NodeID) =====================
// 映射：目标位置(4B) + 控制字(2B)
CanFrame rpdo1_frame1(0x201, new uint8_t[8]{ 0x00, 0x40, 0x00, 0x00, 0x9A, 0xBC }, 6); // 位置=0x00004000, 控制字=0xBC9A
CanFrame rpdo1_frame2(0x202, new uint8_t[8]{ 0xFF, 0xEE, 0xDD, 0xCC, 0x55, 0x66 }, 6); // 位置=0xCCDDEEFF, 控制字=0x6655
CanFrame rpdo1_frame3(0x203, new uint8_t[8]{ 0x00, 0x11, 0x22, 0x33, 0x80, 0x01 }, 6); // 位置=0x33221100, 控制字=0x0180

// ===================== RPDO2测试帧 (0x300 + NodeID) =====================
// 映射：目标速度(2B) + 目标电流(2B) 
CanFrame rpdo2_frame1(0x301, new uint8_t[8]{ 0xAA, 0xBB, 0xCC, 0xDD }, 4); // 速度=0xBBAA, 电流=0xDDCC
CanFrame rpdo2_frame2(0x302, new uint8_t[8]{ 0x11, 0x22, 0x33, 0x44 }, 4); // 速度=0x2211, 电流=0x4433
CanFrame rpdo2_frame3(0x303, new uint8_t[8]{ 0x7F, 0xFF, 0x80, 0x00 }, 4); // 速度=0xFF7F, 电流=0x0080

// ===================== TPDO1测试帧 (0x180 + NodeID) =====================
// 映射：实际位置(4B) + 状态字(2B)
CanFrame tpdo1_frame1(0x181, new uint8_t[8]{ 0x01, 0x23, 0x45, 0x67, 0x04, 0x37 }, 6); // 位置=0x67452301, 状态=0x3704
CanFrame tpdo1_frame2(0x182, new uint8_t[8]{ 0xFE, 0xDC, 0xBA, 0x98, 0x27, 0x10 }, 6); // 位置=0x98BADCFE, 状态=0x1027
CanFrame tpdo1_frame3(0x183, new uint8_t[8]{ 0x7F, 0xFF, 0xFF, 0xFF, 0x00, 0x00 }, 6); // 位置=0xFFFFFF7F, 状态=0x0000

// ===================== TPDO2测试帧 (0x280 + NodeID) =====================
// 映射：实际速度(2B) + 实际电流(2B)
CanFrame tpdo2_frame1(0x281, new uint8_t[8]{ 0x12, 0x34, 0x56, 0x78 }, 4); // 速度=0x3412, 电流=0x7856
CanFrame tpdo2_frame2(0x282, new uint8_t[8]{ 0xFF, 0xFF, 0x80, 0x01 }, 4); // 速度=0xFFFF, 电流=0x0180
CanFrame tpdo2_frame3(0x283, new uint8_t[8]{ 0x00, 0x00, 0x7F, 0xFF }, 4); // 速度=0x0000, 电流=0xFF7F




void testCANProcessing() {




    std::cout << "============= 开始CAN帧处理测试 =============" << std::endl;

    // 1. 初始化6个电机
    std::cout << "\n[阶段1] 初始化6个电机..." << std::endl;
    std::array<Motor, 6> motors = {
        Motor(1), Motor(2), Motor(3),
        Motor(4), Motor(5), Motor(6)
    };
    std::cout << "电机初始化完成" << std::endl;
    // 2. 构建PDO配置表
    std::cout << "\n[阶段2] 构建PDO配置表..." << std::endl;
    std::vector<PdoMappingEntry> pdoTable;
    TIME_IT(
        pdoTable = buildArmMappingTable(6),
        "构建6个电机的PDO配置表"
    );
    std::cout << "生成的PDO配置表条目数: " << pdoTable.size() << std::endl;
    // 3. 准备测试CAN帧
    std::cout << "\n[阶段3] 准备测试CAN帧..." << std::endl;
    std::vector<CanFrame> testFrames = {
        // RPDO1测试帧 (0x200 + NodeID)
        CanFrame(0x201, new uint8_t[8]{ 0x00, 0x40, 0x00, 0x00, 0x9A, 0xBC }, 6), // 位置=0x00004000, 控制字=0xBC9A
        CanFrame(0x202, new uint8_t[8]{0xFF, 0xEE, 0xDD, 0xCC, 0x55, 0x66}, 6), // 电机2: 位置=0xCCDDEEFF, 控制字=0x6655

        // RPDO2测试帧 (0x300 + NodeID)
        CanFrame(0x301, new uint8_t[8]{0xAA, 0xBB, 0xCC, 0xDD}, 4), // 电机1: 速度=0xBBAA, 电流=0xDDCC
        CanFrame(0x302, new uint8_t[8]{0x11, 0x22, 0x33, 0x44}, 4), // 电机2: 速度=0x2211, 电流=0x4433

        // TPDO1测试帧 (0x180 + NodeID)
        CanFrame(0x181, new uint8_t[8]{0x01, 0x23, 0x45, 0x67, 0x04, 0x37}, 6), // 电机1: 位置=0x67452301, 状态=0x3704
        CanFrame(0x182, new uint8_t[8]{0xFE, 0xDC, 0xBA, 0x98, 0x27, 0x10}, 6), // 电机2: 位置=0x98BADCFE, 状态=0x1027

        // TPDO2测试帧 (0x280 + NodeID)
        CanFrame(0x281, new uint8_t[8]{0x12, 0x34, 0x56, 0x78}, 4), // 电机1: 速度=0x3412, 电流=0x7856
        CanFrame(0x282, new uint8_t[8]{0xFF, 0xFF, 0x80, 0x01}, 4)  // 电机2: 速度=0xFFFF, 电流=0x0180
    };
    std::cout << "已准备 " << testFrames.size() << " 个测试CAN帧" << std::endl;
    // 4. 处理所有测试帧
    std::cout << "\n[阶段4] 处理测试CAN帧..." << std::endl;
    for (const auto& frame : testFrames) {
        std::cout << "\n处理CAN帧 ID: 0x" << std::hex << frame.frameID << std::dec
            << ", DLC: " << (int)frame.dlc << std::endl;

        TIME_IT(
            parseCanFrame(frame, motors, pdoTable),
            "解析处理CAN帧"
        );
        const std::vector<uint8_t>& binaryData = frame.getBinaryFrame();

        PrintCANbinaryData(binaryData);

        // 打印处理后的电机状态
        uint8_t nodeId = 0;
        if (frame.frameID >= 0x201 && frame.frameID <= 0x202) nodeId = frame.frameID - 0x200;
        else if (frame.frameID >= 0x301 && frame.frameID <= 0x302) nodeId = frame.frameID - 0x300;
        else if (frame.frameID >= 0x181 && frame.frameID <= 0x182) nodeId = frame.frameID - 0x180;
        else if (frame.frameID >= 0x281 && frame.frameID <= 0x282) nodeId = frame.frameID - 0x280;
        if (nodeId > 0 && nodeId <= motors.size()) {
            auto& motor = motors[nodeId - 1];
            std::lock_guard<std::mutex> lock(motor.mtx_);

            std::cout << "电机" << (int)nodeId << "状态更新:" << std::endl;
            if (frame.frameID == 0x201 || frame.frameID == 0x202) {
                std::cout << "  目标位置: " << motor.position.target_degree.load() << "°" << std::endl;
                std::cout << "  控制字: 0x" << std::hex
                    << (int)motor.stateAndMode.controlData.controlWordRaw[0]
                    << (int)motor.stateAndMode.controlData.controlWordRaw[1] << std::dec << std::endl;
            }
            else if (frame.frameID == 0x301 || frame.frameID == 0x302) {
                std::cout << "  目标速度: " << motor.velocity.target_rpm.load() << " RPM" << std::endl;
                std::cout << "  目标电流: " << motor.current.target_current.load() << " mA" << std::endl;
            }
            else if (frame.frameID == 0x181 || frame.frameID == 0x182) {
                std::cout << "  实际位置: " << motor.position.actual_degree.load() << "°" << std::endl;
                std::cout << "  状态字: 0x" << std::hex
                    << (int)motor.stateAndMode.controlData.statusWordRaw[0]
                    << (int)motor.stateAndMode.controlData.statusWordRaw[1] << std::dec << std::endl;
            }
            else if (frame.frameID == 0x281 || frame.frameID == 0x282) {
                std::cout << "  实际速度: " << motor.velocity.actual_rpm.load() << " RPM" << std::endl;
                std::cout << "  实际电流: " << motor.current.actual_current.load() << " mA" << std::endl;
            }
        }
    }
    std::cout << "\n============= CAN帧处理测试完成 =============" << std::endl;
}






#undef TIME_IT














#endif // !TEST_MODULE
