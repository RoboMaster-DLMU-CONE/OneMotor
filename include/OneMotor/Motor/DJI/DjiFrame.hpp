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
struct RawStatusPlain {
    explicit RawStatusPlain(Can::CanFrame frame) {
        canId = frame.id;
        ecd = static_cast<uint16_t>(frame.data[0] << 8) | frame.data[1];
        rpm = static_cast<int16_t>(static_cast<uint16_t>(frame.data[2] << 8) |
                                   frame.data[3]);
        current_mA = static_cast<int16_t>(
            static_cast<uint16_t>(frame.data[4] << 8) | frame.data[5]);
        temperature_C = frame.data[6];
    };

    uint16_t canId{};        ///< CAN ID
    uint16_t ecd{};          ///< 编码器原始值 (0~8191)
    int16_t rpm{};           ///< 转速 (RPM)
    int16_t current_mA{};    ///< 电机实际电流 (mA)
    uint8_t temperature_C{}; ///< 电机温度 (°C)
};

struct MotorStatusPlain {
    uint16_t last_ecd{};            ///< 上一次的编码器值
    uint16_t ecd{};                 ///< 当前编码器值
    float angle_single_round_deg{}; ///< 单圈角度 (0-360°)
    float angular_deg_s{};          ///< 角速度 (度/秒)
    float real_current_mA{};        ///< 实际电流 (mA)
    float temperature_C{};          ///< 温度 (°C)
    float total_angle_deg{};        ///< 累计总角度 (°)，带方向
    int32_t total_round{};          ///< 累计总圈数，带方向
    float reduced_angle_deg{};      ///< 累计减速后的总角度（°)，带方向
    float reduced_angular_deg_s{};  ///< 累计减速后的总角度（度/秒)，带方向

    float output_current_mA{}; ///< PID计算后输出给电机的电流值
};

struct MotorStatus {
    Units::AngulurVelocityDeg angular{}; ///< 角速度 (度/秒)
    Units::CurrentMilliF real_current{}; ///< 实际电流 (mA)
    Units::Temperature temperature{};    ///< 温度 (°C)
    Units::AngleDeg total_angle{};       ///< 累计总角度 (°)，带方向
    Units::AngleDeg reduced_angle{};     ///< 累计减速后的总角度（°)，带方向
    Units::AngulurVelocityDeg
        reduced_angular{}; ///< 累计减速后的总角度（度/秒)，带方向

    static MotorStatus fromPlain(const MotorStatusPlain &plain) {
        using namespace OneMotor::Units::literals;
        return {.angular = plain.angular_deg_s * deg / s,
                .real_current = plain.real_current_mA * mA,
                .temperature = mp_units::point<deg_C>(plain.temperature_C),
                .total_angle = plain.total_angle_deg * deg,
                .reduced_angle = plain.reduced_angle_deg * deg,
                .reduced_angular = plain.reduced_angular_deg_s * deg / s};
    }

#ifdef ONE_MOTOR_LINUX
    [[nodiscard]] std::string format() const {
        return std::format("M3508 Status:\n"
                           "\t- Angular Velocity: {}\n"
                           "\t- Total Angle: {}\n"
                           "\t- Reduced Angle: {}\n"
                           "\t- Reduced Angular: {}\n"
                           "\t- Real Current: {} mA\n"
                           "\t- Temperature: {} °C",
                           angular, total_angle, reduced_angle, reduced_angular,
                           real_current, temperature.quantity_from_zero());
    };
#endif
};
} // namespace OneMotor::Motor::DJI

#endif // ONEMOTOR_DJIMOTORFRAME_HPP
