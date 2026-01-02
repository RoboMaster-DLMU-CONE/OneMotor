
/**
 * @file DmFrame.hpp
 * @brief 达妙电机 CAN 帧定义
 *
 *
 */
#ifndef ONE_MOTOR_DM_FRAME_HPP
#define ONE_MOTOR_DM_FRAME_HPP
#include <OneMotor/Can/CanFrame.hpp>
#include <OneMotor/Units/Units.hpp>
#include <string>

namespace OneMotor::Motor::DM {
/// @brief 电机参数限制
/// @{
constexpr float DM_V_MIN = -45.0f;  ///< rad/s
constexpr float DM_V_MAX = 45.0f;   ///< rad/s
constexpr float DM_P_MIN = -12.5f;  ///< rad
constexpr float DM_P_MAX = 12.5f;   ///< rad
constexpr float DM_T_MIN = -18.0f;  ///< N*m
constexpr float DM_T_MAX = 18.0f;   ///< N*m
constexpr float DM_KP_MIN = 0.0f;   ///<
constexpr float DM_KP_MAX = 500.0f; ///<
constexpr float DM_KD_MIN = 0.0f;   ///<
constexpr float DM_KD_MAX = 5.0f;   ///<
/// @}

/**
 * @brief 将无符号整数转换为浮点数
 * @param x_int 要转换的无符号整数
 * @param x_min 浮点数范围的最小值
 * @param x_max 浮点数范围的最大值
 * @param bits 无符号整数的位数
 * @return 转换后的浮点数值
 */
constexpr float uint_to_float(const int x_int, const float x_min,
                              const float x_max, const int bits) {
    const float span = x_max - x_min;
    const float offset = x_min;
    return static_cast<float>(x_int) * span /
               static_cast<float>((1 << bits) - 1) +
           offset;
}

/**
 * @brief 将浮点数转换为无符号整数
 * @param x 要转换的浮点数
 * @param x_min 浮点数范围的最小值
 * @param x_max 浮点数范围的最大值
 * @param bits 无符号整数的位数
 * @return 转换后的无符号整数值
 */
constexpr int float_to_uint(const float x, const float x_min, const float x_max,
                            const int bits) {
    const float span = x_max - x_min;
    const float offset = x_min;
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

/**
 * @brief 达妙电机状态结构体
 */
struct DmStatus {
    DmStatus() = default;

    /**
     * @brief 从 CAN 帧构造一个新的 DmStatus 对象
     * @param frame 从电机接收到的 CAN 帧
     */
    explicit DmStatus(const Can::CanFrame &frame);

    /**
     * @brief 将状态格式化为用于打印的字符串
     * @return 格式化后的字符串
     */
    std::string format() const;

    uint8_t ID;                           ///< 电机 ID
    DmCode status;                        ///< 电机状态
    Units::Angle position;                ///< 电机位置 (rad)
    Units::AngularVelocity velocity;      ///< 电机速度 (rad/s)
    Units::Torque torque;                 ///< 电机扭矩 (N*m)
    Units::Temperature temperature_MOS;   ///< MOS 温度 (摄氏度)
    Units::Temperature temperature_Rotor; ///< 转子温度 (摄氏度)
};
} // namespace OneMotor::Motor::DM
#endif // ONE_MOTOR_DM_FRAME_HPP
