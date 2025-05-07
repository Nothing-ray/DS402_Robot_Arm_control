#ifndef CAN_FRAME_HPP
#define CAN_FRAME_HPP

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstring>
#include <cstdint>

// 如果需要在其他地方统一控制调试开关，可以把这里的宏注释掉，
// 并在编译命令或其他头文件中自行定义/取消定义 DEBUG_MODE。
#ifndef DEBUG_MODE
#define DEBUG_MODE 1
#endif

/*******************************************************************
 *
 *  CAN_frame.hpp
 *  负责 CAN 帧结构体 (CanFrame) 及相关逻辑
 *
 ******************************************************************/

 /**
  * @brief CAN 帧结构说明 (13字节二进制格式)  本文件定义的CAN帧是适用于串口转CAN芯片的特殊CAN帧
  *
  * | 字节位置 | 字段名称   | 长度 | 说明                                                                       |
  * |----------|------------|------|----------------------------------------------------------------------------|
  * | 0        | frameInfo  | 1    | [7]:FF(帧格式) [6]:PTR(远程帧) [5]:EDL(FD扩展) [4]:BRS(速率切换) [3:0]:DLC |
  * | 1-4      | frameID    | 4    | 帧ID (大端序: 字节1为最高有效位)                                           |
  * | 5-12     | data       | 8    | 数据区 (实际长度由DLC决定，未用部分补0)                                    |
  * 
  * */






 /**
  * @brief 计算数据的 CRC 校验值，使用 CRC-15 多项式
  * @param data 输入数据指针
  * @param len  数据长度
  * @return     计算得到的 CRC 校验值（15 位）
  */
uint16_t calculate_crc(const uint8_t* data, size_t len);

/**
 * @brief 表示一个标准的 CAN 帧结构，包含帧的 ID、数据、数据长度等信息
 *
 * 备注：支持标准帧、扩展帧，以及 CAN FD 帧格式(FF, PTR, EDL, BRS).
 *       其中每个标志位默认为0。
 */
struct CanFrame {
    uint32_t frameID;              ///< 帧ID，4字节
    uint8_t  data[8];              ///< 数据区，最大8字节
    uint8_t  dlc;                  ///< 数据长度 (0-8)
    uint8_t  frameInfo;            ///< 帧信息字节
    std::vector<uint8_t> binaryFrame; ///< 拼装后用于实际发送的13字节二进制帧

    /**
     * @brief 构造函数，初始化 CAN 帧
     *
     * @param id          帧ID
     * @param input_data  输入数据指针
     * @param input_dlc   数据长度（最大 8）
     * @param FF          帧格式标志 (0=标准帧,1=扩展帧)
     * @param PTR         优先级标志 (0=数据帧,1=远程帧)
     * @param EDL         扩展帧长度标志 (CAN=0,CANFD=1)
     * @param BRS         位速率切换标志 (0=不切换,1=切换)
     */
    CanFrame(uint32_t id, const uint8_t* input_data, uint8_t input_dlc,
        bool FF = false, bool PTR = false, bool EDL = false, bool BRS = false);

    /**
     * @brief 组装 13 字节的串行帧
     * 根据帧信息、ID 和数据，组装出完整的二进制 CAN 帧
     */
    void assembleFrame();

    /**
     * @brief 获取最终的串行帧
     * @return 组装好的 13 字节 CAN 帧数据
     */
    const std::vector<uint8_t>& getBinaryFrame() const;
};

/*******************************************************************
 *                    以下为实现部分 (同头文件内联)
 ******************************************************************/

inline uint16_t calculate_crc(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF; // 初始值
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i] << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                // CRC-15 多项式示例(0x4599)，可根据需要调整
                crc = static_cast<uint16_t>((crc << 1) ^ 0x4599);
            }
            else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return static_cast<uint16_t>(crc & 0x7FFF); // 返回 15 位结果
}

/**
 * @brief CAN 帧构造函数
 *
 * 该构造函数初始化 CAN 帧对象，存储帧 ID、数据、DLC（数据长度代码）和帧信息。
 * 它还会调用 `assembleFrame()` 方法，生成完整的二进制帧。
 *
 * @param id  帧 ID
 * @param input_data  输入数据指针（最多 8 字节）
 * @param input_dlc  数据长度代码（最大 8）
 * @param FF  帧格式标志 (0: 标准帧, 1: 扩展帧)
 * @param PTR  远程请求标志 (0: 数据帧, 1: 远程帧)
 * @param EDL  扩展数据长度 (0: CAN 帧, 1: CAN FD 帧)
 * @param BRS  位速率切换标志 (0: 不切换, 1: 切换)
 */
inline CanFrame::CanFrame(uint32_t id,
    const uint8_t* input_data,
    uint8_t input_dlc,
    bool FF,
    bool PTR,
    bool EDL,
    bool BRS) {
    frameID = id;
    dlc = (input_dlc > 8) ? 8 : input_dlc;  // 限定最大 8 字节
    std::memset(data, 0, sizeof(data));     // 初始化 data 为 0

    if (input_data) {
        std::memcpy(data, input_data, dlc); // 拷贝输入数据
    }

    // 组装帧信息字节 (低 4 位存 DLC, 高 4 位存 4 个标志)
    frameInfo = static_cast<uint8_t>(
        ((FF & 0x01) << 7) |
        ((PTR & 0x01) << 6) |
        ((EDL & 0x01) << 5) |
        ((BRS & 0x01) << 4) |
        (dlc & 0x0F)
        );

    assembleFrame(); // 组装完整的二进制 CAN 帧
}

/**
 * @brief 组装二进制 CAN 帧
 *
 * 该方法将 `frameInfo`、`frameID` 和 `data` 组装成完整的 CAN 帧，并存储到 `binaryFrame` 向量中。
 */
inline void CanFrame::assembleFrame() {
    binaryFrame.clear();

    // 1. 帧信息 (1 字节)
    binaryFrame.push_back(frameInfo);

    // 2. 帧 ID (4 字节: 高位 -> 低位)
    binaryFrame.push_back(static_cast<uint8_t>((frameID >> 24) & 0xFF));
    binaryFrame.push_back(static_cast<uint8_t>((frameID >> 16) & 0xFF));
    binaryFrame.push_back(static_cast<uint8_t>((frameID >> 8) & 0xFF));
    binaryFrame.push_back(static_cast<uint8_t>(frameID & 0xFF));

    // 3. 数据 (8 字节，不足则补 0)
    for (int i = 0; i < 8; ++i) {
        binaryFrame.push_back(data[i]);
    }
}

/**
 * @brief 获取 CAN 帧的二进制数据
 *
 * 该方法返回组装好的 CAN 帧二进制数据，用于发送或存储。
 *
 * @return const std::vector<uint8_t>& 二进制数据帧
 */
inline const std::vector<uint8_t>& CanFrame::getBinaryFrame() const {
    return binaryFrame;
}


#endif // CAN_FRAME_HPP
