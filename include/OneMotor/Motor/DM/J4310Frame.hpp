#ifndef J4310FRAME_HPP
#define J4310FRAME_HPP
#include "OneMotor/Can/CanFrame.hpp"
#include <string>

namespace OneMotor::Motor::DM
{
    constexpr float DM_V_MIN = -45.0f;
    constexpr float DM_V_MAX = 45.0f;
    constexpr float DM_P_MIN = -12.5f;
    constexpr float DM_P_MAX = 12.5f;
    constexpr float DM_T_MIN = -18.0f;
    constexpr float DM_T_MAX = 18.0f;
    constexpr float DM_KP_MIN = 0.0f;
    constexpr float DM_KP_MAX = 500.0f;
    constexpr float DM_KD_MIN = 0.0f;
    constexpr float DM_KD_MAX = 5.0f;

    constexpr float uint_to_float(const int x_int, const float x_min, const float x_max, const int bits)
    {
        const float span = x_max - x_min;
        const float offset = x_min;
        return static_cast<float>(x_int) * span / static_cast<float>((1 << bits) - 1) + offset;
    }

    constexpr int float_to_uint(const float x, const float x_min, const float x_max, const int bits)
    {
        const float span = x_max - x_min;
        const float offset = x_min;
        return static_cast<int>((x - offset) * static_cast<float>((1 << bits) - 1) / span);
    }

    enum class DMStatus
    {
        Disabled = 0,
        Enabled = 1,
        OverVoltage = 8,
        UnderVoltage = 9,
        OverCurrent = 0xA,
        MosOverheat = 0xB,
        RotorOverheat = 0xC,
        Disconnected = 0xD,
        Overloaded = 0xE,
    };

    struct J4310Status
    {
        J4310Status() = default;

        explicit J4310Status(const Can::CanFrame& frame);

        std::string format();

        uint8_t ID;
        DMStatus status;
        float position;
        float velocity;
        float torque;
        uint8_t temperature_MOS;
        uint8_t temperature_Rotor;
    };
}
#endif //J4310FRAME_HPP
