
/**
 * @file DmFrame.hpp
 * @brief 达妙电机 CAN 帧定义
 *
 *
 */
#ifndef ONE_MOTOR_DM_FRAME_HPP
#define ONE_MOTOR_DM_FRAME_HPP
#include <cstdint>
#include <one/can/CanFrame.hpp>
#include <one/motor/Units.hpp>
#include <string>

namespace one::motor::dm {

/**
 * @brief 将无符号整数转换为浮点数
 * @param x_int 要转换的无符号整数
 * @param bits 无符号整数的位数
 * @return 转换后的浮点数值
 */
template <float X_MIN, float X_MAX>
constexpr float uint_to_float(const int x_int, const int bits) {
    constexpr float span = X_MAX - X_MIN;
    constexpr float offset = X_MIN;
    return static_cast<float>(x_int) * span /
               static_cast<float>((1 << bits) - 1) +
           offset;
}

/**
 * @brief 将浮点数转换为无符号整数
 * @param x 要转换的浮点数
 * @param bits 无符号整数的位数
 * @return 转换后的无符号整数值
 */
template <float X_MIN, float X_MAX>
constexpr int float_to_uint(const float x, const int bits) {
    const float span = X_MAX - X_MIN;
    const float offset = X_MIN;
    return static_cast<int>((x - offset) * static_cast<float>((1 << bits) - 1) /
                            span);
}

/**
 * @brief 达妙系列电机状态枚举
 */
enum class DmCode {
    Disabled = 0,        ///< 电机已禁用
    Enabled = 1,         ///< 电机已启用
    OverVoltage = 8,     ///< 过压错误
    UnderVoltage = 9,    ///< 欠压错误
    OverCurrent = 0xA,   ///< 过流错误
    MosOverheat = 0xB,   ///< MOS 过热错误
    RotorOverheat = 0xC, ///< 转子过热错误
    Disconnected = 0xD,  ///< 电机断开连接
    Overloaded = 0xE,    ///< 电机过载
};

template <typename T> struct TypeTag {};

/**
 * @brief 达妙电机状态结构体
 */
/// @brief 回调线程内部使用的无单位状态（基础单位：rad、rad/s、N·m、摄氏度）
struct MotorStatusPlain {
    MotorStatusPlain() = default;
    template <typename Traits>
    explicit MotorStatusPlain(const can::CanFrame &frame, TypeTag<Traits> tag) {
        (void)tag;
        const auto *data = reinterpret_cast<const uint8_t *>(frame.data);
        auto tmp = static_cast<uint16_t>((data[1] << 8) | data[2]);
        position = uint_to_float<Traits::P_MIN, Traits::P_MAX>(tmp, 16); // rad
        tmp = static_cast<uint16_t>((data[3] << 4) | data[4] >> 4);
        velocity =
            uint_to_float<Traits::V_MIN, Traits::V_MAX>(tmp, 12); // rad/s
        tmp = static_cast<uint16_t>(((data[4] & 0x0f) << 8) | data[5]);
        torque = uint_to_float<Traits::T_MIN, Traits::T_MAX>(tmp, 12); // N·m

        ID = data[0] & 0x0F;
        status = static_cast<DmCode>((frame.data[0] >> 4) & 0x0F);
        temperature_MOS = static_cast<float>(data[6]);
        temperature_Rotor = static_cast<float>(data[7]);
    }

    uint8_t ID{};              ///< 电机 ID
    DmCode status{};           ///< 电机状态
    float position{};          ///< rad
    float velocity{};          ///< rad/s
    float torque{};            ///< N·m
    float temperature_MOS{};   ///< 摄氏度
    float temperature_Rotor{}; ///< 摄氏度
};

/// @brief 对外暴露的带单位状态，仅在用户线程构造
struct MotorStatus {
    uint8_t ID{};                           ///< 电机 ID
    DmCode status{};                        ///< 电机状态
    units::Angle position{};                ///< 电机位置 (rad)
    units::AngularVelocity velocity{};      ///< 电机速度 (rad/s)
    units::Torque torque{};                 ///< 电机扭矩 (N*m)
    units::Temperature temperature_MOS{};   ///< MOS 温度 (摄氏度)
    units::Temperature temperature_Rotor{}; ///< 转子温度 (摄氏度)

    static MotorStatus fromPlain(const MotorStatusPlain &plain);
    std::string format() const;
};
} // namespace one::motor::dm
#endif // ONE_MOTOR_DM_FRAME_HPP
