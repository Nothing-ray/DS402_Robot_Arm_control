#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <chrono>
#include <stdint.h>
#include <iostream>

#include "CAN_frame.hpp"
#include "send_frame.hpp"
#include "read_frame.hpp"
#include "CircularBuffer.hpp"


// 调试模式宏定义（1 为开启，0 为关闭）
#define DEBUG_MODE 0

//#define Send_and_receive_DEBUG
//#define Send_and_receive_DEBUG_1


// 毫秒级延时宏定义
#define COMMAND_DELAY_MS 2    // 每条指令之间的延时     目前得到的情况是可以延迟1ms，也可以为了保险选择2ms          为了调试多线程的异步运行问题，现在调多一点
#define INIT_WAIT_MS 5       // 全部电机初始化后等待的时间    延迟5ms应该没问题

#define SENSOR_RANGE 32768   //传感器的范围

/*******************************************       操作数据帧定义       **********************************************/
// 数据帧定义


#define TRUE 1
#define FALSE 0


/*

* @TODO  需要适应更多的初始化模式，  需要考虑心跳包的情况         目前到20
* 也可以考虑读取模式，读取模式下电机使能会导致不能拖动，读取状态只执行到松开抱闸

*/


const uint8_t INIT_CMD_1[8] = { 0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00 }; // 准备好启动
const uint8_t INIT_CMD_2[8] = { 0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00 }; // 松开抱闸




const uint8_t INIT_CMD_3[8] = { 0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00 }; // 电机使能激活

const uint8_t INIT_CMD_17[8] = { 0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00 };// 电机失能

const uint8_t INIT_CMD_4[8] = { 0x2F, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00 }; // 设置位置模式

const uint8_t INIT_CMD_5[8] = { 0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00 }; // 设置速度模式

const uint8_t INIT_CMD_6[8] = { 0x2F, 0x60, 0x60, 0x00, 0x0A, 0x00, 0x00, 0x00 }; // 设置电流模式



const uint8_t INIT_CMD_7[8] = { 0x2B, 0x40, 0x60, 0x00, 0x4F, 0x00, 0x00, 0x00 }; // 设置电机开始绝对运动  注意！只能用于位置模式

const uint8_t INIT_CMD_8[8] = { 0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00 }; // 设置电机停止运动     注意！只能用于位置模式

const uint8_t INIT_CMD_9[8] = { 0x23, 0x83, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 }; // 设置电机的加速度，需要参数

const uint8_t INIT_CMD_10[8] = { 0x23, 0x84, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 }; //设置电机的减速度，需要参数


const uint8_t INIT_CMD_11[8] = { 0x23, 0x81, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 }; // 设置电机的速度，需要参数

const uint8_t INIT_CMD_12[8] = { 0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 }; // 设置电机的位置，需要参数    注意！只能用于位置模式

const uint8_t INIT_CMD_13[8] = { 0x23, 0x71, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };//  设置电机的电流，需要参数

const uint8_t INIT_CMD_20[8] = { 0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00 };//  设置电机清除错误状态

//2B 40 60 00 80 00 00 00


/*******************************************       读取数据帧定义       **********************************************/


const uint8_t INIT_CMD_14[8] = { 0x40, 0x78, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };//  读取电机实际电流，读取出来的是实际的mA，范围是-32767 ~32768

const uint8_t INIT_CMD_15[8] = { 0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };//  读取电机实际位置，-32767 ~32768（对应角度-180~180）

const uint8_t INIT_CMD_16[8] = { 0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };//  读取电机实际速度,读取出来的是实际的速度，单位为RPM/分，范围是-32767 ~32768

const uint8_t INIT_CMD_18[8] = { 0x40, 0x3F, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };//  读取电机的错误状态      

//0x0001:过压 0x0002：欠压 0x0008：堵转 0x0010：过载

const uint8_t INIT_CMD_19[8] = { 0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };//  读取电机的运动状态

//0x0021:准备好启动 0x0023：松开抱闸 0x0027：使能激活







/*******************************************       函数定义       **********************************************/




/**
 * @brief CAN通信超时异常（简洁版）
 *
 * 当CAN指令在指定重试次数后仍未收到响应时抛出
 */
class CANTimeoutException : public std::runtime_error {
public:
    /**
     * @brief 构造函数
     * @param expected_id 期待的响应帧ID
     * @param attempts 已尝试的次数
     */
    explicit CANTimeoutException(uint32_t expected_id, int attempts)
        : std::runtime_error("ERROR  CAN response timeout"),
        expectedID(expected_id),
        retryAttempts(attempts) {
    }

    /// @brief 获取期待的CAN帧ID
    uint32_t expected_id() const { return expectedID; }

    /// @brief 获取已重试次数
    int attempts() const { return retryAttempts; }

private: 
    uint32_t expectedID;   // 预期响应ID
    int retryAttempts;     // 已执行的重试次数
};



/// @brief 解析 uint8_t[8] 的后四个字节为十进制数据（支持 S16 和 S32）
/// @param data 输入的 uint8_t[8] 数据
/// @param mode int类型  0 表示数据为S16  1 表示数据为S32  2 表示数据为U16
/// @return 解析得到的十进制数
int32_t parseDataFromLastFourBytes(const uint8_t data[8], int mode) {
    int32_t value = 0;

    // 解析后四个字节的实际数据，低字节在前，高字节在后
    for (int i = 4; i < 8; ++i) {
        value |= (static_cast<int32_t>(data[i]) << (8 * (i - 4)));
    }

    // 根据 mode 参数决定如何处理数据
    switch (mode) {
    case 0: // S16
        value &= 0xFFFF;  // 只保留低 16 位
        if (value & 0x8000) { // 如果符号位为 1，进行符号扩展
            value |= 0xFFFF0000; // 扩展符号位为负数
        }
        break;

    case 1: // S32
        // 不做任何符号扩展或截断，直接返回 32 位整数
        break;

    case 2: // U16
        value &= 0xFFFF; // 仅保留低 16 位的无符号值
        break;

    default:
        break;
    }

    return value;
}









/// @brief 将十进制数转换为 uint8_t[8] 格式（支持 S16 和 S32），前四字节填充为 0
/// @param value 要转换的十进制数
/// @param mode int类型，0表示数据为S16,1表示数据为S32,2表示数据为U16
/// @return 准备好的 uint8_t[8] 数组
uint8_t* generateDataWithZeroPrefix(int32_t value, int mode) {
    static uint8_t outputData[8] = { 0 };

    // 根据 mode 参数决定如何处理数据
    switch (mode) {
    case 0: // S16
        if (value < -SENSOR_RANGE) value = -SENSOR_RANGE;   //防止发送超出量程的数据
        if (value > SENSOR_RANGE-1) value = SENSOR_RANGE-1;
        value &= 0xFFFF; // 保留低 16 位
        break;

    case 1: // S32
        // 无需做额外处理，直接使用 32 位值
        break;

    case 2: // U16
        value &= 0xFFFF; // 只保留低 16 位
        break;

    default:
        break;
    }

    // 后四字节填充为数据的字节形式（低位在前）
    for (int i = 0; i < 4; ++i) {
        outputData[i + 4] = (value >> (8 * i)) & 0xFF; // 小端序
    }

    return outputData;
}






/// @brief 拼装新的 8 字节数据
/// @param CommandData 原始的 8 字节数据（如 INIT_CMD_3）,用于取指令
/// @param ParamData 后四字节的新数据，用于控制的参数
/// @return 拼装好的新的 8 字节数据
uint8_t* assembleNewData(const uint8_t CommandData[8], const uint8_t ParamData[8]) {
    static uint8_t outputData[8];//创建一个输出用的参数

    // 前四字节从原始数据复制
    for (int i = 0; i < 4; ++i) {
        outputData[i] = CommandData[i];
    }

    // 后四字节从新的数据复制
    for (int i = 4; i < 8; ++i) {
        outputData[i] = ParamData[i];
    }

    return outputData;
}




/// @brief 传感器值与角度值之间的换算
/// @param value 输入的十进制数
/// @param isSensorToAngle 布尔值：1 表示传感器值转换为角度，0 表示角度转换为传感器值
/// @return 换算后的值（传感器到角度返回带两位小数的浮点数，角度到传感器值返回整数）
/// @TODO   输入不接受小数值，有可能影响关节角到传感器值的精度，有可能需要考虑重载或者模板
double convertSensorAndAngle(double value, bool isSensorToAngle) {                 //输入参数为整数，无法接受小数，待修改
    const double sensorRange = SENSOR_RANGE; // 传感器范围   32768
    const double scaleFactor = 360.0 / (2 * sensorRange); // 换算比例因子 (360°对应传感器全量程)

    if (isSensorToAngle) {
        // 传感器值转换为角度值，带两位小数（截断多余小数）
        double angle = value * scaleFactor;
        return std::round(angle * 100.0) / 100.0; // 保留两位小数，四舍五入
    }
    else {
        // 角度值转换为传感器值（整数）
        return static_cast<int32_t>(std::round(value / scaleFactor));
    }
}







/*

/// @brief 发送指令并等待目标响应帧
/// @param frame 要发送的 CAN 帧
/// @param serial 串口对象
/// @param buffer 缓冲区指针
/// @param returnResponse 布尔值：为 true 时返回接收到的目标帧；为 false 时返回空帧（ID 为 0，数据为 0）
/// @return 返回接收到的目标帧或空帧
CanFrame sendAndReceive(CanFrame frame, boost::asio::serial_port& serial, CircularBuffer* buffer, bool returnResponse) {
    // 动态计算目标接收帧的 ID
    uint32_t expectedResponseID = frame.frameID - 0x080;

#ifdef Send_and_receive_DEBUG

    std::cout << "[DEBUG] Entering sendAndReceive" << std::endl;

#endif // Send_and_receive_DEBUG

    // 发送数据
    sendDataToSerial(serial, frame);

    // 等待并解析目标帧
    while (true) {


        //std::cout << "E0" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(COMMAND_DELAY_MS)); // 等待一段时间，这个等待时间可以视为轮询的频率

        if (buffer->isbuffer_busy == true)
        {//当缓冲区忙的时候停止读取，等待下一个读取循环，避免线程问题
            //std::this_thread::yield();
            continue;
        }
        CanFrame receivedFrame = parseCanFrameFromBuffer(buffer); // 从缓冲区中解析帧


#ifdef Send_and_receive_DEBUG_1

        std::cout << "[DEBUG] Exiting sendAndReceive, response: " << receivedFrame.frameID << std::endl;

#endif // Send_and_receive_DEBUG_1

        // 检查是否匹配目标响应 ID
        if (receivedFrame.frameID == expectedResponseID) {
            
#ifdef Send_and_receive_DEBUG_1
            std::cout << "接收到响应帧 ID: 0x" << std::hex << receivedFrame.frameID << std::endl;
#endif // Send_and_receive_DEBUG_1

            // 如果布尔值为 true，返回接收到的目标帧
            if (returnResponse) {
                return receivedFrame;
            }
            break; // 如果不需要返回响应帧，退出循环

        }
#ifdef Send_and_receive_DEBUG_1
        else {
            std::cout << "接收到错误帧或无效帧，继续等待..." << std::endl;
        }
#endif // Send_and_receive_DEBUG_1
    }
    
    // 如果布尔值为 false，返回一个空帧（ID 为 0，数据全为 0）
    return CanFrame(0, nullptr, 0);
}

*/




/**
 * @brief 通过CAN总线发送指令并等待目标响应帧（具备超时控制）
 *
 * @param[in] frame 要发送的CAN帧，包含帧ID、数据负载等信息
 * @param[in,out] serial 串口对象引用，用于数据发送，需保证在函数调用期间有效
 * @param[in] buffer 循环缓冲区指针，用于存储接收的原始数据（需确保线程安全）
 * @param[in] returnResponse 响应控制标志：
 *              - true  : 返回实际接收到的响应帧
 *              - false : 返回空帧（ID为0，数据为空）
 *
 * @return CanFrame 返回接收到的有效响应帧或空帧
 *
 * @throw CANTimeoutException 当达到最大重试次数仍未收到有效响应时抛出
 *
 * @note 函数执行流程：
 *  1. 计算预期响应帧ID（当前帧ID - 0x080）
 *  2. 发送请求帧到串口
 *  3. 进入轮询循环：
 *     a. 等待固定时间间隔（COMMAND_DELAY_MS）
 *     b. 检查缓冲区状态
 *     c. 解析CAN帧数据
 *     d. 验证响应帧ID匹配性
 *  4. 达到最大重试次数抛出超时异常
 *
 * @warning 重要约束条件：
 *  - 缓冲区必须实现线程安全访问机制
 *  - COMMAND_DELAY_MS应大于单次CAN帧传输时间
 *  - 最大重试次数(MAX_RETRY)需根据实际总线延迟调整
 *
 * @par 示例代码：
 * @code
 * try {
 *     CanFrame response = sendAndReceive(commandFrame, serial, buffer, true);
 *     processResponse(response);
 * } catch (const CANTimeoutException& e) {
 *     handleTimeout(e.expected_id());
 * }
 * @endcode
 *
 * @see CANTimeoutException
 */
CanFrame sendAndReceive(CanFrame frame,
    boost::asio::serial_port& serial,
    CircularBuffer* buffer,
    bool returnResponse)
{
    const uint32_t expectedResponseID = frame.frameID - 0x080;
    constexpr int MAX_RETRY = 80;      // 最大重试次数
    int retryCount = 0;                // 当前重试计数

#ifdef Send_and_receive_DEBUG
    std::cout << "[DEBUG] 期待响应ID: 0x" << std::hex << expectedResponseID << std::endl;
#endif

    // 发送请求帧
    sendDataToSerial(serial, frame);

    // 接收响应循环
    while (retryCount < MAX_RETRY) {
        std::this_thread::sleep_for(std::chrono::milliseconds(COMMAND_DELAY_MS));

        // 检查缓冲区状态
        if (buffer->isbuffer_busy) {
            ++retryCount;  // 递增重试计数器
            continue;
        }

        // 尝试解析CAN帧
        CanFrame receivedFrame = parseCanFrameFromBuffer(buffer);

#ifdef Send_and_receive_DEBUG_1
        std::cout << "[DEBUG] 收到帧ID: 0x" << std::hex << receivedFrame.frameID
            << " 数据长度: " << std::dec << receivedFrame.dataLength << std::endl;
#endif

        // 验证响应ID
        if (receivedFrame.frameID == expectedResponseID) {
            return returnResponse ? receivedFrame : CanFrame(0, nullptr, 0);
        }

        ++retryCount;  // 递增重试计数器

        // 最后一次重试仍未收到响应
        if (retryCount >= MAX_RETRY) {
            throw CANTimeoutException(expectedResponseID, MAX_RETRY);
        }
    }

    // 此处不会执行到（因循环内已抛出异常）
    return CanFrame(0, nullptr, 0);
}






/// @brief 打印向量数据，以十六进制格式输出
/// @param data 要打印的 uint8_t 类型的向量数据
/// @param description 描述信息，默认为 "Data"
/// @details 此函数接收一个 uint8_t 类型的向量数据，并将每个字节以十六进制格式打印到标准输出流。
///          每个字节输出为两位的 16 进制数，前缀为 "0x"，并以空格分隔。打印完成后，恢复为 10 进制输出。
///          描述信息作为数据的标题打印在输出的首行，便于区分不同的数据集合。
/// @example
/// std::vector<uint8_t> exampleData = {0x12, 0xAB, 0x7F};
/// PrintVectorData(exampleData, "Example Vector");
/// // 输出:
/// // Example Vector: 0x12 0xAB 0x7F 
/// @note 使用 std::hex 和 std::setfill 确保每个字节都显示两位，无论其数值大小。
///       如果需要以 10 进制查看数据，可直接打印原始向量或移除格式化输出。
void PrintVectorData(const std::vector<uint8_t>& data, const std::string& description = "Data") {
    std::cout << description << ": ";
    for (size_t i = 0; i < data.size(); ++i) {
        // 输出为16进制格式，保证每个字节都显示两位
        //std::cout << "0x";   //可以选择要不要加上0x
        std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
            << static_cast<int>(data[i]) << " ";
    }
    std::cout << std::dec << std::endl; // 恢复为10进制输出
}





/// @brief 电机初始化函数
/// @brief 按照顺序给一个电机发送两条初始化指令
/// @brief 只启动和松开抱闸
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机ID
/// @param buffer 指向接收反馈帧的缓冲区的指针
void Motor_Init(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送

    CanFrame frame_init_1(motorID, INIT_CMD_1, 8);
    CanFrame frame_init_2(motorID, INIT_CMD_2, 8);

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame1 = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值
    CanFrame receiveFrame2 = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值

    receiveFrame1 = sendAndReceive(frame_init_1, serial, buffer, 0);
    receiveFrame2 = sendAndReceive(frame_init_2, serial, buffer, 0);

    //设置电机的ID

}



/// @brief 位置模式
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向接收反馈帧的缓冲区的指针
void MotorAngle_Mode(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    
    
    //构筑数据帧，取指令，然后发送

    CanFrame frame_init(motorID, INIT_CMD_4, 8);

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值

    receiveFrame = sendAndReceive(frame_init, serial, buffer, 0);
    
    //设置角度模式

}


/// @brief 速度模式
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向接收反馈帧的缓冲区的指针
void MotorSpeed_Mode(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送

    CanFrame frame_init(motorID, INIT_CMD_5, 8);

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值

    receiveFrame = sendAndReceive(frame_init, serial, buffer, 0);

    //设置速度模式

}


/// @brief 电流模式
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向接收反馈帧的缓冲区的指针
void MotorCurrent_Mode(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送

    CanFrame frame_init(motorID, INIT_CMD_6, 8);

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值

    receiveFrame = sendAndReceive(frame_init, serial, buffer, 0);

    //设置速度模式

}




/// @brief 电机使能
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向接收反馈帧的缓冲区的指针
void MotorEnable_Mode(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送

    CanFrame frame_init(motorID, INIT_CMD_3, 8);

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值

    receiveFrame = sendAndReceive(frame_init, serial, buffer, 0);

    std::this_thread::sleep_for(std::chrono::milliseconds(3));//这个指令需要额外的3毫秒延时
    //设置使能模式

}


/// @brief 电机失能
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向接收反馈帧的缓冲区的指针
void MotorDisable_Mode(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送

    CanFrame frame_init(motorID, INIT_CMD_17, 8);

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值

    receiveFrame = sendAndReceive(frame_init, serial, buffer, 0);

    //设置失能模式

}




/// @brief 电机 “开始” 绝对运动，仅用于位置模式
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向接收反馈帧的缓冲区的指针
void Motor_Start(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送

    CanFrame frame_init(motorID, INIT_CMD_7, 8);

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值

    receiveFrame = sendAndReceive(frame_init, serial, buffer, 0);

    //设置电机开启绝对运动

}



/// @brief 电机 “停止” 绝对运动，仅用于位置模式
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向接收反馈帧的缓冲区的指针
void Motor_Stop(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送

    CanFrame frame_init(motorID, INIT_CMD_8, 8);

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值

    receiveFrame = sendAndReceive(frame_init, serial, buffer, 0);

    //设置电机开启绝对运动

}


/// @brief 电机的错误状态清除
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向接收反馈帧的缓冲区的指针
void Motor_Error_Clear(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送

    CanFrame frame_init(motorID, INIT_CMD_20, 8);

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值

    receiveFrame = sendAndReceive(frame_init, serial, buffer, 0);

    //设置电机的错误状态清除

}






/// @brief 设置电机的“加速度”
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向缓冲区的指针
/// @param data 需要写入的数据
void Set_Motor_Acceleration(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer,int data) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送

    data = data * 100; //需要先乘以100的电机减速比，如果是其他的电机则不用管

    //将数据，组装到后四个字节
    auto ParamData = generateDataWithZeroPrefix(data,2);    // 加减速度是U16
    auto SendData = assembleNewData(INIT_CMD_9, ParamData);

    CanFrame frame_init(motorID, SendData, 8);

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值

    receiveFrame = sendAndReceive(frame_init, serial, buffer, 0);

    //设置电机的加速度

}



/// @brief 设置电机的“减速度”
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向缓冲区的指针
/// @param data 需要写入的数据
void Set_Motor_Deceleration(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer, int data) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送

    data = data * 100; //需要先乘以100的电机减速比，如果是其他的电机则不用管

    //将数据，组装到后四个字节
    auto ParamData = generateDataWithZeroPrefix(data,2);  //加减速度是U16
    auto SendData = assembleNewData(INIT_CMD_10, ParamData);

    CanFrame frame_init(motorID, SendData, 8);

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值

    receiveFrame = sendAndReceive(frame_init, serial, buffer, 0);

    //设置电机的减速度

}


/// @brief 设置电机的“速度”
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向缓冲区的指针
/// @param data 需要写入的数据
void Set_Motor_Speed(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer, int data) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送

    //速度的设置不需要*100

    //将数据，组装到后四个字节
    auto ParamData = generateDataWithZeroPrefix(data,0);//目标速度的设置应该为S16
    //将指令头和指令参数进行组装
    auto SendData = assembleNewData(INIT_CMD_11, ParamData);
    
    CanFrame frame_init(motorID, SendData, 8);

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值

    receiveFrame = sendAndReceive(frame_init, serial, buffer, 0);

    //设置电机的速度

}

/// @brief 设置电机的“位置”
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向缓冲区的指针
/// @param data 需要写入的数据
void Set_Motor_Angle(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer, float data) {


    /*
    const int max_limit = 180;
    const int min_limit = -180;

    if (data > max_limit )
    {
        std::cerr << "检测到过大的电机参数:" << data << std::endl;
        throw std::runtime_error("ERROR:  Queue  Empty");//抛出异常 
    }

    */


    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送
    //先把输入的数据转换为传感器参数，再把传感器参数转换为数据帧
    int32_t NEWdata = static_cast<int32_t>(data);//先从int进行显式形式转换
    //把形式转换好的角度值转换为传感器值
    //data = (int)convertSensorAndAngle(data, 0);

    // 将角度值转换为传感器值
    int32_t sensorValue = static_cast<int32_t>(convertSensorAndAngle(data, false));


    //将数据，组装到后四个字节
    auto ParamData = generateDataWithZeroPrefix(sensorValue,1);//目标位置的设置应该是S32
    //将指令头和指令参数进行组装
    auto SendData = assembleNewData(INIT_CMD_12, ParamData);

    CanFrame frame_init(motorID, SendData, 8);

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值

    receiveFrame = sendAndReceive(frame_init, serial, buffer, 0);

    //设置电机的角度

}




/*

/// @brief 【高精度插值模式】 设置电机的“位置”   
/// @brief 首先读取当前位置和目标位置，然后高精度读取当前编码器的值，计算目标的编码值，然后进行插值
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向缓冲区的指针
/// @param data 需要写入的数据
/// @TODO  需要改为闭环模式
void Set_Motor_Angle_frame(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer, int data, int frame_num = 50) {
    const uint32_t BASE_ID = 0x600; // 电机的初始ID
    uint32_t motorID = BASE_ID + (uint32_t)ID; // 计算电机的ID

    // 读取当前编码器的值
    int Angle_now = Read_Motor_Angle(serial, ID, buffer);

    // 计算目标编码器值
    int Angle_target = static_cast<int>(convertSensorAndAngle(data, 0));

    // 计算插值步长
    int Angle_step = (Angle_target - Angle_now) / frame_num;

    // 进入插值过程
    for (int i = 0; i < frame_num; ++i) {
        int intermediate_angle = Angle_now + (i + 1) * Angle_step;

        // 生成数据帧
        auto ParamData = generateDataWithZeroPrefix(intermediate_angle, 1); // S32格式
        auto SendData = assembleNewData(INIT_CMD_12, ParamData);
        CanFrame frame(motorID, SendData, 8);

        // 发送数据帧
        sendAndReceive(frame, serial, buffer, 0);

        // 轮询等待目标角度到达
        while (!Target_Motor_Angle(serial, ID, buffer, intermediate_angle)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // 确保最终位置正确
    while (!Target_Motor_Angle(serial, ID, buffer, Angle_target)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

}


*/









/// @brief 设置电机的“电流”
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向缓冲区的指针
/// @param data 需要写入的数据
void Set_Motor_Current(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer, int data) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送

    //将数据，组装到后四个字节
    auto ParamData = generateDataWithZeroPrefix(data,0);//目标电流的设置应该是S16
    //将指令头和指令参数进行组装
    auto SendData = assembleNewData(INIT_CMD_13, ParamData);

    CanFrame frame_init(motorID, SendData, 8);

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值

    receiveFrame = sendAndReceive(frame_init, serial, buffer, 0);

    //设置电机的电流

}










/// @brief 读取当前的“电流”，单位mA，范围是-32767 ~32768
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向缓冲区的指针
/// @return 目标电机的电流
float Read_Motor_Current(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送


    CanFrame frame_init(motorID, INIT_CMD_14, 8);//读取电流的数据帧

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值


    //发送指令，然后接收返回值，处理以后进行输出
    receiveFrame = sendAndReceive(frame_init, serial, buffer, 1);

    //输出读取到的电流
    auto ReceiveData = parseDataFromLastFourBytes(receiveFrame.data,0);//电流应该是S16

    return ReceiveData;

    //读取电机的电流

}



/// @brief 读取当前的“位置”，-32767 ~32768（对应角度-180~180）
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向缓冲区的指针
/// @return 目标电机的角度（-180 —— +180）
/// @TODO  读取位置和角度转换部分存在精度损失，待修改
float Read_Motor_Angle(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送


    CanFrame frame_init(motorID, INIT_CMD_15, 8);//读取角度的数据帧

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值


    //发送指令，然后接收返回值，处理以后进行输出
    receiveFrame = sendAndReceive(frame_init, serial, buffer, 1);

    //输出读取到的位置
    auto ReceiveData = parseDataFromLastFourBytes(receiveFrame.data, 1);//读取到的位置，位置应该是S32

    //读取到的位置还需要转换成角度
    //ReceiveData = convertSensorAndAngle(ReceiveData,TRUE);
    
    //选择TRUE，传感器值转换为角度
    return static_cast<float>(convertSensorAndAngle(ReceiveData, true));

    //读取电机的位置

}






/// @brief 【高精度模式】 读取当前的“位置”，-32767 ~32768，返回的是编码器的问题 
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向缓冲区的指针
/// @return 目标电机的编码器
int Read_Motor_Angle_Encoder(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送


    CanFrame frame_init(motorID, INIT_CMD_15, 8);//读取角度的数据帧

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值


    //发送指令，然后接收返回值，处理以后进行输出
    receiveFrame = sendAndReceive(frame_init, serial, buffer, 1);

    //输出读取到的位置
    auto ReceiveData = parseDataFromLastFourBytes(receiveFrame.data, 1);//读取到的位置，位置应该是S32

    return ReceiveData;

    //读取电机的位置

}




/// @brief 读取当前的“速度”，读取出来的是实际的速度，单位为RPM/分，范围是-32767 ~32768
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向缓冲区的指针
/// @return 目标电机的速度，单位RPM/min
float Read_Motor_Speed(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送


    CanFrame frame_init(motorID, INIT_CMD_15, 8);//读取速度的数据帧

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值


    //发送指令，然后接收返回值，处理以后进行输出
    receiveFrame = sendAndReceive(frame_init, serial, buffer, 1);

    //输出读取到的速度
    auto ReceiveData = parseDataFromLastFourBytes(receiveFrame.data, 0);//读取到的速度，速度是S16

    return ReceiveData;

    //读取电机的速度

}



/// @brief 读取电机的错误状态，没有的标志就是没事                       
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向缓冲区的指针
/// @return 目标电机的错误状态，    
///         0x0001:过压 0x0002：欠压 0x0008：堵转 0x0010：过载
void Read_Motor_State_Error(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送


    CanFrame frame_init(motorID, INIT_CMD_18, 8);//读取电机错误状态的数据帧

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值


    //发送指令，然后接收返回值，处理以后进行输出
    receiveFrame = sendAndReceive(frame_init, serial, buffer, 1);

    //先读取输出的数据
    auto  data = receiveFrame.data;//

    //将读取到的数据转化为向量类型，输出函数只接受向量
    std::vector<uint8_t> Error_code(data, data + 8);

    std::cout << "0x0001:过压   0x0002：欠压   0x0008：堵转   0x0010：过载 " << std::endl;

    //输出读取到的错误代码
    PrintVectorData(Error_code, "Error Code: ");
    

    //读取电机的错误状态 

}



/// @brief 读取电机的运动状态，没有的标志就是没事                       
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向缓冲区的指针
/// @return 目标电机的运动状态，    
///         0x0021:准备好启动 0x0023：松开抱闸 0x0027：使能激活
void Read_Motor_State_Run(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送


    CanFrame frame_init(motorID, INIT_CMD_19, 8);//读取电机运动状态的数据帧

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值


    //发送指令，然后接收返回值，处理以后进行输出
    receiveFrame = sendAndReceive(frame_init, serial, buffer, 1);
     
    //先读取输出的数据
    auto  data = receiveFrame.data;//

    //将读取到的数据转化为向量类型，输出函数只接受向量
    std::vector<uint8_t> State_code(data, data + 8);

    std::cout << "0x0021:准备好启动   0x0023：松开抱闸   0x0027：使能激活 " << std::endl;

    //输出读取到的错误代码
    PrintVectorData(State_code, "State Code: ");


    //读取电机的运动状态 

}






/// @brief 简易的安全检查模块，通过读取当前关节角判断是否是正常值，如果读取到的值不安全，并且会输出出问题的数据帧 
/// @brief 先读取角度，输出关节目前的角度，没有问题则退出，有问题则输出数据帧
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向缓冲区的指针
/// @return  True : 电机角度没有检查出问题    
///          False: 电机的角度超过+-180
bool Read_Motor_Safe_Angle(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer) {

    const uint32_t BASE_ID = 0x600; // 电机的初始ID
    const int32_t max_angle = 180;
    const int32_t min_angle = -180;

    uint32_t motorID = BASE_ID + (uint32_t)ID;//先计算出电机的ID
    //构筑数据帧，取指令，然后发送

    CanFrame frame_init(motorID, INIT_CMD_15, 8);//读取角度的数据帧

    // 依次发送初始化指令并等待响应
    CanFrame receiveFrame = CanFrame(0, nullptr, 0);//创建一个空的用于接受的帧，目前必须接受返回值

    //发送指令，然后接收返回值，处理以后进行输出
    receiveFrame = sendAndReceive(frame_init, serial, buffer, 1);

    //输出读取到的位置
    auto ReceiveData = parseDataFromLastFourBytes(receiveFrame.data, 1);//读取到的位置，位置应该是S32

    //读取到的位置还需要转换成角度
    auto ReceiveAngle = convertSensorAndAngle(ReceiveData, TRUE);//选择TRUE，传感器值转换为角度

    //输出本关节电机的角度
    //std::cout << "Motor: " << ID << " Start Angle: " << ReceiveAngle << std::endl;


    if (ReceiveAngle > max_angle || ReceiveAngle < min_angle)
    {//读取到异常数据的情况
        
        //读取原始的数据帧
        std::vector<uint8_t> receivedata = receiveFrame.getBinaryFrame();
        //输出在异常时读取到的数据帧，保留错误信息
        PrintVectorData(receivedata, "[DEBUG]  [ E R R O R ]");


        //输出错误的信息以后失能并且刷写所有的数据为0

        Set_Motor_Acceleration(serial, ID, buffer, 0);
        Set_Motor_Speed(serial, ID, buffer, 0);
        Motor_Stop(serial, ID, buffer);
        MotorDisable_Mode(serial, ID, buffer);

        return false;
    }
    
    return true;


    //正常的情况下返回True

}












/*


/// @brief 检测机械臂位置，调用一次就检测一次，位置模式或者速度模式都可以用            还在施工中，请勿使用
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向缓冲区的指针
/// @return True，到达指定位置。False，未到达指定位置。
bool Target_Motor_Angle(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer, int data) {


    float ReceiveData = Read_Motor_Angle(serial, ID, buffer);//读取目标电机的角度，注意这里不需要再取地址了

    float closedata = 0.01;

    if (  (ReceiveData - (float)data) < closedata)
    {
        return TRUE;//小于校准值的情况，也就是到达目标位置
    }
    else
    {
        return FALSE;//没到达目标的情况
    }

}

*/


/// @brief 检测电机是否达到目标角度
/// @param serial boost::asio::serial_port 串口对象
/// @param ID 电机的编号，从肩关节为1号一直到6号
/// @param buffer 指向缓冲区的指针
/// @param target_angle 目标角度（单位：度）
/// @param tolerance 允许的误差范围（默认0.5度）
/// @return 返回 true 表示到达目标角度，false 表示未到达
bool Target_Motor_Angle(boost::asio::serial_port& serial, int ID, CircularBuffer* buffer, int target_angle, float tolerance = 0.5) {
    // 读取当前电机的角度
    float current_angle = Read_Motor_Angle(serial, ID, buffer);

    // 计算当前角度和目标角度的差值
    float difference = std::abs(current_angle - target_angle);

    // 如果角度误差小于设定的误差范围，则认为到达目标
    return difference <= tolerance;
}

#undef DEBUG_MODE