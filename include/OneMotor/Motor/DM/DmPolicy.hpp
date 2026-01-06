#ifndef ONE_MOTOR_DM_DMPOLICY_HPP_
#define ONE_MOTOR_DM_DMPOLICY_HPP_

#include "OneMotor/Motor/DM/DmFrame.hpp"
#include <OneMotor/Units/Units.hpp>
#include <atomic>
#include <cstdint>

namespace OneMotor::Motor::DM {
struct DmControlOutput {
    enum class Mode { MIT, PosVel, Vel } mode = Mode::MIT;
    uint8_t data[8];
};

template <typename Traits> struct MITPolicy {
    float m_kp{}, m_kd{};
    MITPolicy() = default;
    MITPolicy(float kp, float kd) : m_kp(kp), m_kd(kd) {};
    DmControlOutput compute(std::atomic<float> &pos_ref,
                            std::atomic<float> &ang_ref,
                            std::atomic<float> &tor_ref,
                            Traits::StatusType &status) {
        (void)status;
        const auto position = pos_ref.load(std::memory_order_acquire); // rad
        const auto angular = ang_ref.load(std::memory_order_acquire);  // rad/s
        const auto torque = tor_ref.load(std::memory_order_acquire);   // N·m

        const uint16_t pos =
            float_to_uint<Traits::P_MIN, Traits::P_MAX>(position, 16);
        const uint16_t vel =
            float_to_uint<Traits::V_MIN, Traits::V_MAX>(angular, 12);
        const uint16_t tor =
            float_to_uint<Traits::T_MIN, Traits::T_MAX>(torque, 12);
        const uint16_t kp =
            float_to_uint<Traits::KP_MIN, Traits::KP_MAX>(m_kp, 12);
        const uint16_t kd =
            float_to_uint<Traits::KD_MIN, Traits::KD_MAX>(m_kd, 12);

        return {DmControlOutput::Mode::MIT,
                {static_cast<uint8_t>(pos >> 8),
                 static_cast<uint8_t>(pos & 0xFF),
                 static_cast<uint8_t>(vel >> 4),
                 static_cast<uint8_t>(((vel & 0xF) << 4) | (kp >> 8)),
                 static_cast<uint8_t>(kp & 0xFF), static_cast<uint8_t>(kd >> 4),
                 static_cast<uint8_t>(((kd & 0xF) << 4) | (tor >> 8)),
                 static_cast<uint8_t>(tor & 0xFF)}};
    };
};

template <typename Traits> struct PosVelPolicy {
    DmControlOutput compute(std::atomic<float> &pos_ref,
                            std::atomic<float> &ang_ref,
                            std::atomic<float> &tor_ref,
                            Traits::StatusType &status) {
        (void)status;
        const auto position = pos_ref.load(std::memory_order_acquire); // rad
        const auto angular = ang_ref.load(std::memory_order_acquire);  // rad/s

        const auto *pbuf = reinterpret_cast<const uint8_t *>(&position);
        const auto *vbuf = reinterpret_cast<const uint8_t *>(&angular);

        DmControlOutput out;
        out.mode = DmControlOutput::Mode::PosVel;

        std::copy_n(pbuf, 4, out.data);
        std::copy_n(vbuf, 4, out.data + 4);
        return out;
    }
};

template <typename Traits> struct VelPolicy {
    DmControlOutput compute(std::atomic<float> &pos_ref,
                            std::atomic<float> &ang_ref,
                            std::atomic<float> &tor_ref,
                            Traits::StatusType &status) {
        (void)status;
        const auto angular = ang_ref.load(std::memory_order_acquire); // rad/s

        const auto *vbuf = reinterpret_cast<const uint8_t *>(&angular);

        DmControlOutput out;
        out.mode = DmControlOutput::Mode::PosVel;

        std::copy_n(vbuf, 4, out.data);
        return out;
    }
};

} // namespace OneMotor::Motor::DM

#endif // ONE_MOTOR_DM_DMPOLICY_HPP_
