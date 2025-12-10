#ifndef ONEMOTOR_DJIMOTORFRAME_HPP
#define ONEMOTOR_DJIMOTORFRAME_HPP

#include <cstdint>
#ifdef ONE_MOTOR_LINUX
#include <format>
#else
#include <sstream>
#include <iomanip>
#endif
#include <string>

#include "OneMotor/Can/CanFrame.hpp"

namespace OneMotor::Motor::DJI
{
    /**
     * @struct RawStatusFrame
     * @brief DJI电机原始状态反馈的CAN帧的直接映射。
     * @details
     * 此结构体用于解析从DJI电机接收到的CAN报文，或构建要发送到电机的CAN报文。
     * 它提供了与 `Can::CanFrame` 相互转换的功能。
     */
    struct RawStatusFrame
    {
        /**
         * @brief 将此结构体转换为通用的 `Can::CanFrame`。
         */
        explicit operator Can::CanFrame() const
        {
            Can::CanFrame frame{};
            frame.id = canId;
            frame.dlc = 8;
            frame.data[0] = static_cast<uint8_t>(ecd >> 8);
            frame.data[1] = static_cast<uint8_t>(ecd & 0xFF);
            frame.data[2] = static_cast<uint8_t>(rpm >> 8);
            frame.data[3] = static_cast<uint8_t>(rpm & 0xFF);
            frame.data[4] = static_cast<uint8_t>(current >> 8);
            frame.data[5] = static_cast<uint8_t>(current & 0xFF);
            frame.data[6] = temperature;
            return frame;
        };

        /**
         * @brief 从通用的 `Can::CanFrame` 构造此结构体。
         * @param frame 从CAN总线接收到的原始帧。
         */
        explicit RawStatusFrame(Can::CanFrame frame)
        {
            canId = frame.id;
            ecd = static_cast<uint16_t>(frame.data[0] << 8) | frame.data[1];
            rpm = static_cast<int16_t>(static_cast<uint16_t>(frame.data[2] << 8) | frame.data[3]);
            current = static_cast<int16_t>(static_cast<uint16_t>(frame.data[4] << 8) | frame.data[5]);
            temperature = frame.data[6];
        };

        /**
         * @brief 将帧内容格式化为可读字符串。
         * @return 包含帧所有字段信息的字符串。
         */
        [[nodiscard]] std::string format() const
        {
#ifdef ONE_MOTOR_LINUX
            return std::format("ID: {:X}, Ang: {} RPM, ecd: {} / 8191, CRT: {} mA, temperature: {} deg",
                               canId, rpm, ecd, current, temperature);
#else
            std::ostringstream oss;
            oss << "ID: " << std::hex << std::uppercase << canId << std::dec
                << ", Ang: " << rpm << " RPM, ecd: " << ecd << " / 8191, CRT: "
                << current << " mA, temperature: " << temperature << " deg";
            return oss.str();
#endif
        };

        uint16_t canId; ///< CAN ID
        uint16_t ecd; ///< 编码器原始值 (0~8191)
        int16_t rpm; ///< 转速 (RPM)
        int16_t current; ///< 电机实际电流 (mA)
        uint8_t temperature; ///< 电机温度 (°C)
    };

    struct MotorStatus
    {
        uint16_t last_ecd; ///< 上一次的编码器值
        uint16_t ecd; ///< 当前编码器值
        float angle_single_round; ///< 单圈角度 (0-360°)
        float angular; ///< 角速度 (度/秒)
        int16_t real_current; ///< 实际电流 (mA)
        uint8_t temperature; ///< 温度 (°C)
        float total_angle; ///< 累计总角度 (°)，带方向
        int32_t total_round; ///< 累计总圈数，带方向
        int16_t output_current; ///< PID计算后输出给电机的电流值
        /**
         * @brief 将电机状态格式化为可读字符串。
         * @return 包含状态所有字段信息的字符串。
         */
        [[nodiscard]] std::string format() const
        {
#ifdef ONE_MOTOR_LINUX
            return std::format(
                "M3508 Status:\n"
                "\t- ECD: {} (last: {})\n"
                "\t- Angle (single round): {:.2f} deg\n"
                "\t- Angular Velocity: {:.2f} deg/s\n"
                "\t- Total Angle: {:.2f} deg\n"
                "\t- Total Rounds: {}\n"
                "\t- Real Current: {} mA\n"
                "\t- Output Current: {}\n"
                "\t- Temperature: {} °C",
                ecd, last_ecd,
                angle_single_round,
                angular,
                total_angle,
                total_round,
                real_current,
                output_current,
                temperature);
#else
            std::ostringstream oss;
            oss << "M3508 Status:\n"
                << "\t- ECD: " << ecd << " (last: " << last_ecd << ")\n"
                << "\t- Angle (single round): " << std::fixed << std::setprecision(2) << angle_single_round << " deg\n"
                << "\t- Angular Velocity: " << std::fixed << std::setprecision(2) << angular << " deg/s\n"
                << "\t- Total Angle: " << std::fixed << std::setprecision(2) << total_angle << " deg\n"
                << "\t- Total Rounds: " << total_round << "\n"
                << "\t- Real Current: " << real_current << " mA\n"
                << "\t- Output Current: " << output_current << "\n"
                << "\t- Temperature: " << temperature << " °C";
            return oss.str();
#endif
        };
    };
}

#endif //ONEMOTOR_DJIMOTORFRAME_HPP
