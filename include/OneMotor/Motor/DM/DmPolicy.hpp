#ifndef ONE_MOTOR_DM_DMPOLICY_HPP_
#define ONE_MOTOR_DM_DMPOLICY_HPP_

#include "DmTraits.hpp"
#include <OneMotor/Motor/MotorAcessor.hpp>
#include <OneMotor/Units/Units.hpp>

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
    DmControlOutput compute(MotorAcessor *motor, Traits::StatusType &status) {
        (void)status;
        const auto pos_ref = motor->getPosRef(); // rad
        const auto ang_ref = motor->getAngRef(); // rad/s
        const auto tor_ref = motor->getTorRef(); // N·m

        return {
            .mode = DmControlOutput::Mode::MIT,
            .position = pos_ref,
            .angular = ang_ref,
            .torque = tor_ref,
            .kp = m_kp,
            .kd = m_kd,
        };
    }
};

template <typename Traits = DmTraits> struct PosVelPolicy {
    DmControlOutput compute(MotorAcessor *motor, Traits::StatusType &status) {
        (void)status;
        const auto pos_ref = motor->getPosRef(); // rad
        const auto ang_ref = motor->getAngRef(); // rad/s

        return {
            .mode = DmControlOutput::Mode::PosVel,
            .position = pos_ref,
            .angular = ang_ref,
        };
    }
};

template <typename Traits = DmTraits> struct VelPolicy {
    DmControlOutput compute(MotorAcessor *motor, Traits::StatusType &status) {
        (void)status;
        const auto ang_ref = motor->getAngRef(); // rad/s

        return {
            .mode = DmControlOutput::Mode::Vel,
            .angular = ang_ref,
        };
    }
};

} // namespace OneMotor::Motor::DM

#endif // ONE_MOTOR_DM_DMPOLICY_HPP_
