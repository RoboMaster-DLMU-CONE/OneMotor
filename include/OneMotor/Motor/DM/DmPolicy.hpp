#ifndef ONE_MOTOR_DM_DMPOLICY_HPP_
#define ONE_MOTOR_DM_DMPOLICY_HPP_

#include "DmTraits.hpp"
#include <OneMotor/Units/Units.hpp>
#include <mp-units/systems/si/units.h>

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
    DmControlOutput compute(Units::Angle pos_ref,
                            Units::AngularVelocity ang_ref,
                            Units::Torque tor_ref, Traits::StatusType &status) {
        return {
            .mode = DmControlOutput::Mode::MIT,
            .position = pos_ref.numerical_value_in(mp_units::angular::radian),
            .angular = ang_ref.numerical_value_in(mp_units::angular::radian /
                                                  mp_units::si::second),
            .torque = tor_ref.numerical_value_in(mp_units::si::newton *
                                                 mp_units::si::metre),
            .kp = m_kp,
            .kd = m_kd,
        };
    }
};

template <typename Traits = DmTraits> struct PosVelPolicy {
    DmControlOutput compute(Units::Angle pos_ref,
                            Units::AngularVelocity ang_ref,
                            Units::Torque tor_ref, Traits::StatusType &status) {
        return {
            .mode = DmControlOutput::Mode::PosVel,
            .position = pos_ref.numerical_value_in(mp_units::angular::radian),
            .angular = ang_ref.numerical_value_in(mp_units::angular::radian /
                                                  mp_units::si::second),
        };
    }
};

template <typename Traits = DmTraits> struct VelPolicy {
    DmControlOutput compute(Units::Angle pos_ref,
                            Units::AngularVelocity ang_ref,
                            Units::Torque tor_ref, Traits::StatusType &status) {
        return {
            .mode = DmControlOutput::Mode::PosVel,
            .angular = ang_ref.numerical_value_in(mp_units::angular::radian /
                                                  mp_units::si::second),
        };
    }
};

} // namespace OneMotor::Motor::DM

#endif // ONE_MOTOR_DM_DMPOLICY_HPP_
