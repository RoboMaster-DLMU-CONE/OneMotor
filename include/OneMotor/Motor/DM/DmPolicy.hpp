#ifndef ONE_MOTOR_DM_DMPOLICY_HPP_
#define ONE_MOTOR_DM_DMPOLICY_HPP_

#include "DmTraits.hpp"
#include <OneMotor/Units/Units.hpp>
#include <atomic>

namespace OneMotor::Motor::DM {
struct DmControlOutput {
    enum class Mode { MIT, PosVel, Vel } mode = Mode::MIT;

    float position{};
    float angular{};
    float torque{};
    float kp{};
    float kd{};
};

template <typename Traits = DmTraits> struct MITPolicy {
    float m_kp{}, m_kd{};
    MITPolicy() = default;
    MITPolicy(float kp, float kd) : m_kp(kp), m_kd(kd) {};
    DmControlOutput compute(std::atomic<float> &pos_ref,
                            std::atomic<float> &ang_ref,
                            std::atomic<float> &tor_ref,
                            Traits::StatusType &status) {
        (void)status;
        const auto pos = pos_ref.load(std::memory_order_acquire); // rad
        const auto ang = ang_ref.load(std::memory_order_acquire); // rad/s
        const auto tor = tor_ref.load(std::memory_order_acquire); // N·m

        return {
            .mode = DmControlOutput::Mode::MIT,
            .position = pos,
            .angular = ang,
            .torque = tor,
            .kp = m_kp,
            .kd = m_kd,
        };
    }
};

template <typename Traits = DmTraits> struct PosVelPolicy {
    DmControlOutput compute(std::atomic<float> &pos_ref,
                            std::atomic<float> &ang_ref,
                            std::atomic<float> &tor_ref,
                            Traits::StatusType &status) {
        (void)status;
        const auto pos = pos_ref.load(std::memory_order_acquire); // rad
        const auto ang = ang_ref.load(std::memory_order_acquire); // rad/s

        return {
            .mode = DmControlOutput::Mode::PosVel,
            .position = pos,
            .angular = ang,
        };
    }
};

template <typename Traits = DmTraits> struct VelPolicy {
    DmControlOutput compute(std::atomic<float> &pos_ref,
                            std::atomic<float> &ang_ref,
                            std::atomic<float> &tor_ref,
                            Traits::StatusType &status) {
        (void)status;
        const auto ang = ang_ref.load(std::memory_order_acquire); // rad/s

        return {
            .mode = DmControlOutput::Mode::Vel,
            .angular = ang,
        };
    }
};

} // namespace OneMotor::Motor::DM

#endif // ONE_MOTOR_DM_DMPOLICY_HPP_
