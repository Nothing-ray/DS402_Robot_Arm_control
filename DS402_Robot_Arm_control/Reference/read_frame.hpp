#ifndef READ_FRAME_HPP
#define READ_FRAME_HPP

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>

#include "CAN_frame.hpp"
#include "CircularBuffer.hpp"
#include  "send_frame.hpp"


/*******************      调试标志区      *******************/

#define DEBUG_MODE 0

//#define DEBUG_MODE_2_ARM

//#define handleRead_DEBUG

//#define Listener_DEBUG

//#define handleRead_DEBUG_Time

//#define distribute_DEBUG_1

//#define distribute_DEBUG_2

//#define distribute_DEBUG_Frame

//#define Listener_DEBUG

//#define DEBUG_MODE_1






/*******************      代码区      *******************/




/// @brief 监听串口并将接收到的消息写入环形缓冲区
/// @param serialPort boost::asio::serial_port 串口对象，用于接收数据
/// @param buffer 指向环形缓冲区的指针，用于存储接收到的数据
/// @details 从串口中读取数据并存储到缓冲区中。若缓冲区写入失败，打印错误信息。
void listenSerialPort(boost::asio::serial_port& serialPort, CircularBuffer* buffer) {
    uint8_t data[256];  // 临时接收数据缓冲区
    boost::asio::read(serialPort, boost::asio::buffer(data, sizeof(data)));

    // 将接收到的数据写入环形缓冲区
    if (!buffer->write(data, sizeof(data))) {
        std::cerr << "Error writing to buffer!" << std::endl;
    }
}



/// @brief 从环形缓冲区中解析消息并返回 CAN 帧
/// @param buffer 指向环形缓冲区的指针，用于读取和解析数据
/// @return 解析后的 CAN 帧对象，包含帧 ID、数据内容和数据长度
/// @details 检查缓冲区是否有足够的数据可供解析，若无数据或格式无效，记录错误信息。
///          解析帧的 ID、数据长度 (DLC) 和数据内容，不改变缓冲区中的其他内容。
CanFrame parseCanFrameFromBuffer(CircularBuffer* buffer) {
    uint8_t header[13];

    // 先检查缓冲区是否有足够的数据
    if (!buffer->read(header, 13)) {

        //std::cout << "C1 " << std::endl;

#if DEBUG_MODE

        std::cerr << "Not enough data to read frame!" << std::endl;
        //return CanFrame();  // 返回一个空的 CanFrame
#endif
    }

    // 解析数据
    uint8_t dlc = header[0];  // 数据长度
    
    if (dlc > 8) {
#if DEBUG_MODE
        std::cerr << "Invalid DLC (Data Length Code) value: " << (int)dlc << std::endl;
        //return CanFrame();  // 返回空帧
#endif
    }
    

    uint32_t frameID = (header[1] << 24) | (header[2] << 16) | (header[3] << 8) | header[4];

    // 创建数据缓冲区，并拷贝数据
    uint8_t data[8] = { 0 };  // 默认数据为0
    memcpy(data, &header[5], dlc);  // 将数据从 header 中复制到 data 数组

    // 创建并返回 CanFrame，这里似乎没有设置相关的标志位，不过作为返回帧这一点不是很重要
    return CanFrame(frameID, data, dlc);
}


// 编译宏定义电机数量
#define MOTOR_NUM 6
#define RECEIVE_MOTOR_ID_START 0x580

/// @brief 解析缓冲区中的消息并根据帧ID分配到对应的机械臂缓冲区
/// @param buffer 指向主环形缓冲区的指针，用于读取和解析数据
/// @param armBuffers 用于存储各机械臂数据的缓冲区数组
/// @details 检查主缓冲区是否有足够数据，并根据帧ID解析出目标机械臂编号。若数据合法，
///          将其分配到对应的机械臂缓冲区。写入失败时打印错误信息，确保调试输出清晰。
void distributeCanFrame(CircularBuffer* buffer, const std::vector<CircularBuffer*>& armBuffers) {
    uint8_t header[13];


#ifdef distribute_DEBUG_1

    // 调试：检查缓冲区状态
    std::cout << "[DEBUG] distributeCanFrame: Checking main buffer size: " << buffer->getSize() << std::endl;

#endif // distribute_DEBUG_1

    if (!buffer->read(header, 13)) {//当读取失败的情况直接退出

#ifdef distribute_DEBUG_2
        std::cerr << "[DEBUG] Error: Not enough data to read frame!" << std::endl;
#endif // distribute_DEBUG_2
        return;
    }

#ifdef distribute_DEBUG_1

    // 调试：读取到的帧头数据
    std::cout << "[DEBUG] Frame header data: ";
    for (int i = 0; i < 13; i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(header[i]) << " ";
    }
    std::cout << std::dec << std::endl;

#endif // distribute_DEBUG_1
    
   
    uint8_t dlc = header[0];
    if (dlc > 8) {//当数据长度超过合法长度时

#ifdef distribute_DEBUG_2
        std::cerr << "[DEBUG] Error: Invalid DLC value: " << static_cast<int>(dlc) << std::endl;
#endif // distribute_DEBUG_2

        return;
    }

    uint32_t frameID = (header[1] << 24) | (header[2] << 16) | (header[3] << 8) | header[4];
    uint32_t nodeID = frameID - RECEIVE_MOTOR_ID_START;
    uint32_t armIndex = (nodeID - 1) / MOTOR_NUM;

#ifdef distribute_DEBUG_Frame

    // 调试：帧解析结果
    std::cout << "[DEBUG] Parsed frameID: 0x" << std::hex << frameID
        << ", nodeID: " << std::dec << nodeID
        << ", armIndex: " << armIndex << std::endl;

#endif // distribute_DEBUG_Frame


    if (armIndex >= armBuffers.size() || armBuffers[armIndex] == nullptr) {
#ifdef distribute_DEBUG_2
        std::cerr << "[DEBUG] Error: armIndex out of range or buffer uninitialized: " << armIndex << std::endl;
#endif // distribute_DEBUG_2
        return;
    }

    /*
    while (armBuffers[armIndex]->isbuffer_busy) {
        std::this_thread::yield();               //让出CPU时间片
    }
    */

    if (!armBuffers[armIndex]->write(header, 13)) {
#ifdef distribute_DEBUG_2

        //写入失败的情况
        std::cerr << "[DEBUG] Error: Write to buffer failed at index: " << armIndex << std::endl;

#endif // distribute_DEBUG_2
    }
    else {

#ifdef distribute_DEBUG_2

        // 调试：确认数据写入成功
        std::cout << "[DEBUG] Successfully wrote data to armBuffer[" << armIndex << "]" << std::endl;

#endif // distribute_DEBUG_2

    }
}







class SerialListener {
public:


    /// @brief SerialListener类的构造函数
    /// @param serialPort boost::asio::serial_port 串口对象，用于接收数据
    /// @param buffer 指向主环形缓冲区的指针，用于存储接收到的数据
    /// @param buffers 用于分配数据的机械臂缓冲区数组
    /// @param bufferCount 机械臂缓冲区的数量
    /// @details 初始化串口监听器，记录机械臂缓冲区信息，并启动异步监听以实现数据接收。
    SerialListener(boost::asio::serial_port& serialPort, CircularBuffer* buffer, CircularBuffer* buffers[], size_t bufferCount)
        : serialPort(serialPort), buffer(buffer) {

        // 将传递的数组内容存储到 std::vector 中
        for (size_t i = 0; i < bufferCount; ++i) {
            armBuffers.push_back(buffers[i]);
        }
#ifdef Listener_DEBUG

        // 调试输出：打印所有缓冲区指针
        for (size_t i = 0; i < armBuffers.size(); ++i) {
            std::cout << "[DEBUG] armBuffers[" << i << "]: " << armBuffers[i] << std::endl;
        }

#endif // Listener_DEBUG

        startListening();  // 启动监听
    }

    /// @brief 从主缓冲区分配数据帧到目标机械臂缓冲区
    /// @param buffer 指向主环形缓冲区的指针，用于读取数据帧
    /// @details 调用 distributeCanFrame 函数，确保数据正确分配到对应的机械臂缓冲区。
    void distributeFrame(CircularBuffer* buffer) {
        distributeCanFrame(buffer, armBuffers);
    }


    /// @brief 启动串口的异步监听
    /// @details 初始化接收缓冲区并开始异步读取。数据到达后自动调用 handleRead 进行处理。
    void startListening() {
        size_t size = 1024;
        dataBuffer.resize(size);  // 数据缓冲区大小，可以根据需求调整
        asyncRead();  // 启动异步读取
    }

    // @brief 异步读取处理函数
    /// @param error boost::system::error_code 对象，用于检查读取是否成功
    /// @param bytesTransferred 实际传输的字节数
    /// @details 处理串口接收到的数据，将其写入主缓冲区。若写入成功，调用分配函数将数据帧分配到机械臂缓冲区。
    ///          若发生错误，则输出相关错误信息以供调试。最后继续下一轮异步读取。
    void handleRead(const boost::system::error_code& error, std::size_t bytesTransferred) {
        if (!error) {

            
            
#ifdef handleRead_DEBUG_Time

            //std::cout << "[DEBUG] Entering handleRead, bytesTransferred: " << bytesTransferred << std::endl;

            //计算开始的时间
            auto start = std::chrono::high_resolution_clock::now();

#endif // handleRead_DEBUG_Time
            
            // 将接收到的数据,从临时缓冲区，写入环形缓冲区
            if (!buffer->write(&dataBuffer[0], bytesTransferred)) {
                //写入错误的情况

#ifdef handleRead_DEBUG
                std::cerr << "[DEBUG] Error: Failed to write data to buffer. Buffer might be full." << std::endl;
#endif // handleRead_DEBUG

            }//下面这一段是新的数据分配模块
            else {
                
                // 从环形缓冲区内，分配数据帧到目标缓冲区
                if (bytesTransferred != 0)
                {

#ifdef handleRead_DEBUG

                    std::cout << "[DEBUG] Data successfully written to buffer. Transferred bytes: " << bytesTransferred << std::endl;
                    std::cout << "[DEBUG] Distributing data to armBuffers..." << std::endl;

                    std::cout << "[DEBUG] SerialListener this: " << this << std::endl;

                    std::cout << "[DEBUG] armBuffers address: " << &this->armBuffers << std::endl;
                    std::cout << "[DEBUG] armBuffers[0]: " << this->armBuffers[0] << std::endl;
                    std::cout << "[DEBUG] armBuffers[1]: " << this->armBuffers[1] << std::endl;

#endif // handleRead_DEBUG

                    distributeFrame(buffer);
                }
                
            }

#ifdef handleRead_DEBUG_Time

            //计算结束的时间
            auto end = std::chrono::high_resolution_clock::now();

            // 计算并输出时间差
            std::chrono::duration<double> duration = end - start;
            std::cout << "[DEBUG] Time taken to process " << bytesTransferred
                << " bytes: " << duration.count() * 1000 << " ms" << std::endl;//输出运行的毫秒

#endif // handleRead_DEBUG_Time
            
            // 继续异步读取
            asyncRead();
        }
        else {
#ifdef handleRead_DEBUG

            std::cerr << "[DEBUG] Error during read: " << error.message() << std::endl;

#endif // handleRead_DEBUG
        }
    }

private:
    boost::asio::serial_port& serialPort;  // 已初始化的串口对象
    CircularBuffer* buffer;                // 环形缓冲区指针
    std::vector<uint8_t> dataBuffer;       // 临时数据缓冲区，用于接收数据
    //CircularBuffer* armBuffers[];          // 分配用的环形缓冲区数组，这个数组每个元素代表了一个机械臂的专用缓冲区

    std::vector<CircularBuffer*> armBuffers;  // 用于保存缓冲区的动态数组，把读取到的地址复制到这里，避免跨线程的时候丢失数据


    /// @brief 异步读取串口数据
    /// @details 指定读取的字节数并启动异步读取。当读取完成或失败时，调用 handleRead 进行回调处理。
    void asyncRead() {

        const size_t packetSize = 13;//需要读取的字节数，如果

        // 异步读取串口数据
        //在这里指定需要读取的字节数，如果没有packetSize，会一直把缓冲区读满为止再写入环形缓冲区
        boost::asio::async_read(serialPort, boost::asio::buffer(dataBuffer, packetSize),
            boost::bind(&SerialListener::handleRead, this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
    }
};










#endif // READ_FRAME_HPP
