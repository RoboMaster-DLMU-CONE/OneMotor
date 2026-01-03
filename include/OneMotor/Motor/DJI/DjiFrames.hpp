#ifndef ONEMOTOR_DJIMOTORFRAME_HPP
#define ONEMOTOR_DJIMOTORFRAME_HPP

#include <cstdint>
#include <format>
#include <string>

#include <OneMotor/Can/CanFrame.hpp>
#include <OneMotor/Units/Units.hpp>

namespace OneMotor::Motor::DJI {
using namespace OneMotor::Units::literals;
/**
 * @struct RawStatusFrame
 * @brief DJI电机原始状态反馈的CAN帧的直接映射。
 * @details
 * 此结构体用于解析从DJI电机接收到的CAN报文，或构建要发送到电机的CAN报文。
 */
struct RawStatusFrame {

    /**
     * @brief 从通用的 `Can::CanFrame` 构造此结构体。
     * @param frame 从CAN总线接收到的原始帧。
     */
    explicit RawStatusFrame(Can::CanFrame frame) {
        canId = frame.id;
        ecd = static_cast<uint16_t>(frame.data[0] << 8) | frame.data[1];
        rpm = static_cast<int16_t>(static_cast<uint16_t>(frame.data[2] << 8) |
                                   frame.data[3]) *
              rpm;
        current =
            static_cast<int16_t>(static_cast<uint16_t>(frame.data[4] << 8) |
                                 frame.data[5]) *
            mA;
        temperature = mp_units::point<deg_C>(frame.data[6]);
    };

    /**
     * @brief 将帧内容格式化为可读字符串。
     * @return 包含帧所有字段信息的字符串。
     */
    [[nodiscard]] std::string format() const {
        return std::format("ID: {:X}, Ang: {}, ecd: {} / 8191, CRT: {}, "
                           "temperature: {}",
                           canId, rpm, ecd, current,
                           temperature.quantity_from_zero());
    };

    uint16_t canId;                 ///< CAN ID
    uint16_t ecd;                   ///< 编码器原始值 (0~8191)
    Units::RPM rpm;                 ///< 转速 (RPM)
    Units::CurrentMilli current;    ///< 电机实际电流 (mA)
    Units::Temperature temperature; ///< 电机温度 (°C)
};

struct MotorStatus {
    uint16_t last_ecd;                  ///< 上一次的编码器值
    uint16_t ecd;                       ///< 当前编码器值
    Units::AngleDeg angle_single_round; ///< 单圈角度 (0-360°)
    Units::AngulurVelocityDeg angular;  ///< 角速度 (度/秒)
    Units::CurrentMilli real_current;   ///< 实际电流 (mA)
    Units::Temperature temperature;     ///< 温度 (°C)
    Units::AngleDeg total_angle;        ///< 累计总角度 (°)，带方向
    Units::Round total_round;           ///< 累计总圈数，带方向
    Units::CurrentMilli output_current; ///< PID计算后输出给电机的电流值
    /**
     * @brief 将电机状态格式化为可读字符串。
     * @return 包含状态所有字段信息的字符串。
     */
    [[nodiscard]] std::string format() const {
        return std::format("M3508 Status:\n"
                           "\t- ECD: {} (last: {})\n"
                           "\t- Angle (single round): {}\n"
                           "\t- Angular Velocity: {}\n"
                           "\t- Total Angle: {}\n"
                           "\t- Total Rounds: {}\n"
                           "\t- Real Current: {} mA\n"
                           "\t- Output Current: {}\n"
                           "\t- Temperature: {} °C",
                           ecd, last_ecd, angle_single_round, angular,
                           total_angle, total_round, real_current,
                           output_current, temperature.quantity_from_zero());
    };
};
} // namespace OneMotor::Motor::DJI

#endif // ONEMOTOR_DJIMOTORFRAME_HPP
