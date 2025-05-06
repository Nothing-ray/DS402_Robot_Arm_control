#ifndef TEST_MODULE
#define TEST_MODULE

#include <array>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include "CLASS_Motor.hpp"

using namespace std::chrono_literals;

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

















#undef TIME_IT














#endif // !TEST_MODULE
