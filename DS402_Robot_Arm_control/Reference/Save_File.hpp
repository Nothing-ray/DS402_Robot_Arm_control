#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>



/**
 * @brief 保存关键点数据到文件（增强版）
 * @param keyPoints1 1号机械臂关键点数据
 * @param keyPoints2 2号机械臂关键点数据
 * @param filename 输出文件名（可选前缀）
 * @param defaultSpeed 默认速度值（RPM）
 * @param defaultTime 默认时间段（毫秒）
 *
 * @details 每行格式：{角度1,角度2,...}{速度}{时间}
 */
void SaveKeyPoints(
    const std::vector<std::vector<float>>& keyPoints1,
    const std::vector<std::vector<float>>& keyPoints2,
    const std::string& filename = "",
    int defaultSpeed = 2,
    int defaultTime = 5000)
{
    // 生成时间戳（安全版本）
    time_t now = time(nullptr);
    struct tm tm;
#if defined(_WIN32) || defined(_WIN64)
    localtime_s(&tm, &now);  // Windows版本
#else
    localtime_r(&now, &tm);  // Linux/Mac版本
#endif

    // 构建文件名
    char timeStr[20];
    strftime(timeStr, sizeof(timeStr), "%H_%M", &tm);

    std::string fullFilename = filename.empty() ?
        std::string(timeStr) + ".txt" :
        filename + "_" + timeStr + ".txt";

    // 创建并打开文件
    std::ofstream outFile(fullFilename);
    if (!outFile.is_open()) {
        std::cerr << "无法创建文件: " << fullFilename << std::endl;
        return;
    }

    // 保存1号机械臂数据
    outFile << "#1" << std::endl;
    for (const auto& point : keyPoints1) {
        outFile << "{";
        for (size_t i = 0; i < point.size(); ++i) {
            outFile << point[i];
            if (i != point.size() - 1) outFile << ", ";
        }
        outFile << "}{" << defaultSpeed << "}{" << defaultTime << "}" << std::endl;
    }
    outFile << std::endl; // 数据段间空行分隔

    // 保存2号机械臂数据
    outFile << "#2" << std::endl;
    for (const auto& point : keyPoints2) {
        outFile << "{";
        for (size_t i = 0; i < point.size(); ++i) {
            outFile << point[i];
            if (i != point.size() - 1) outFile << ", ";
        }
        outFile << "}{" << defaultSpeed << "}{" << defaultTime << "}" << std::endl;
    }

    std::cout << "数据已保存至: " << fullFilename << std::endl;
}




/**
 * @brief 从文件读取机械臂关键点数据
 * @param filename 要读取的文件名
 * @param outDataQueue1 输出1号机械臂角度队列
 * @param outSpeedQueue1 输出1号机械臂速度队列
 * @param outTimeQueue1 输出1号机械臂时间队列
 * @param outDataQueue2 输出2号机械臂角度队列
 * @param outSpeedQueue2 输出2号机械臂速度队列
 * @param outTimeQueue2 输出2号机械臂时间队列
 * @return 是否成功读取
 *
 * @details 文件格式示例：
 * #1
 * {1,2,3,4,5,6}{2}{5000} // 注释
 * {7,8,9,10,11,12}{3}{4000}
 *
 * #2
 * {13,14,15,16,17,18}{5}{3000}
 */
bool LoadKeyPointsFromFile(
    const std::string& filename,
    std::queue<std::vector<float>>& outDataQueue1,
    std::queue<int>& outSpeedQueue1,
    std::queue<int>& outTimeQueue1,
    std::queue<std::vector<float>>& outDataQueue2,
    std::queue<int>& outSpeedQueue2,
    std::queue<int>& outTimeQueue2)
{
    std::ifstream inFile(filename);
    if (!inFile.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }

    // 清除队列中原有数据
    while (!outDataQueue1.empty()) outDataQueue1.pop();
    while (!outSpeedQueue1.empty()) outSpeedQueue1.pop();
    while (!outTimeQueue1.empty()) outTimeQueue1.pop();
    while (!outDataQueue2.empty()) outDataQueue2.pop();
    while (!outSpeedQueue2.empty()) outSpeedQueue2.pop();
    while (!outTimeQueue2.empty()) outTimeQueue2.pop();

    std::string line;
    int currentArm = 0; // 0=未知, 1=arm1, 2=arm2

    while (std::getline(inFile, line)) {
        // 移除注释部分
        size_t commentPos = line.find("//");
        if (commentPos != std::string::npos) {
            line = line.substr(0, commentPos);
        }

        // 去除首尾空白字符
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);

        // 跳过空行
        if (line.empty()) continue;

        // 检查机械臂标记
        if (line == "#1") {
            currentArm = 1;
            continue;
        }
        else if (line == "#2") {
            currentArm = 2;
            continue;
        }

        if (currentArm == 0) continue; // 跳过未指定机械臂的数据

        // 解析数据行：{角度}{速度}{时间}
        std::vector<std::string> segments;
        size_t start = 0, end = 0;

        while ((end = line.find("}{", start)) != std::string::npos) {
            segments.push_back(line.substr(start + 1, end - start - 1));
            start = end + 1;
        }
        segments.push_back(line.substr(start + 1, line.size() - start - 2));

        if (segments.size() != 3) {
            std::cerr << "格式错误: " << line << std::endl;
            continue;
        }

        try {
            // 解析角度数据
            std::vector<float> angles;
            std::stringstream angleStream(segments[0]);
            std::string angleStr;
            while (std::getline(angleStream, angleStr, ',')) {
                angles.push_back(std::stof(angleStr));
            }

            // 解析速度
            int speed = std::stoi(segments[1]);

            // 解析时间
            int time = std::stoi(segments[2]);

            // 存入对应队列
            if (currentArm == 1) {
                outDataQueue1.push(angles);
                outSpeedQueue1.push(speed);
                outTimeQueue1.push(time);
            }
            else if (currentArm == 2) {
                outDataQueue2.push(angles);
                outSpeedQueue2.push(speed);
                outTimeQueue2.push(time);
            }
        }
        catch (const std::exception& e) {
            std::cerr << "解析错误: " << e.what() << " in line: " << line << std::endl;
            continue;
        }
    }

    return true;
}
