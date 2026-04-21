/**
 * @file LkFrame.hpp
 * @brief 达妙电机 CAN 帧定义
 *
 *
 */
#ifndef ONE_MOTOR_LK_FRAME_HPP
#define ONE_MOTOR_LK_FRAME_HPP
#include <cstdint>
#include <one/can/CanFrame.hpp>
#include <one/motor/Units.hpp>
#include <string>

namespace one::motor::lk {

/**
 * @brief 瓴控电机状态结构体
 */
/// @brief 回调线程内部使用的无单位状态（基础单位：rad、rad/s、N·m、摄氏度）
struct MotorStatusPlain {
    MotorStatusPlain() = default;
    explicit MotorStatusPlain(const can::CanFrame &frame) {
        const auto *data = reinterpret_cast<const uint8_t *>(frame.data);

        cmd = data[0];
        temperature = static_cast<int8_t>(data[1]);

        // LK feedback frame uses little-endian layout for 16-bit values.
        iq = static_cast<int16_t>(static_cast<uint16_t>(data[2]) |
                                  (static_cast<uint16_t>(data[3]) << 8));
        angular = static_cast<int16_t>(static_cast<uint16_t>(data[4]) |
                                       (static_cast<uint16_t>(data[5]) << 8));
        encoder = static_cast<uint16_t>(static_cast<uint16_t>(data[6]) |
                                        (static_cast<uint16_t>(data[7]) << 8));
    }

    uint8_t cmd{};        ///< 命令码
    int8_t temperature{}; ///< 电机温度
    int16_t iq{};         ///< 转矩电流

    int16_t angular{};  ///< 速度
    uint16_t encoder{}; ///< 编码器
};

/// @brief 对外暴露的带单位状态，仅在用户线程构造
struct MotorStatus {
    static MotorStatus fromPlain(const MotorStatusPlain &plains) {
        return {.angular = static_cast<float>(plains.angular),
                .current = static_cast<float>(plains.iq)};
    }
    float angular{}; ///< 电机速度 (rad/s)
    float current{}; ///< 电机电流 (mA)
};
} // namespace one::motor::lk
#endif // ONE_MOTOR_LK_FRAME_HPP
