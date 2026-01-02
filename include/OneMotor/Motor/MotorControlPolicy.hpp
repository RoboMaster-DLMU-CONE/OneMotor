#ifndef ONE_MOTOR_MOTORCONTROLPOLICY_HPP_
#define ONE_MOTOR_MOTORCONTROLPOLICY_HPP_

#include <OneMotor/Units/Units.hpp>

namespace OneMotor::Motor {

template <typename Status> class IControlPolicy {
  public:
    virtual ~IControlPolicy() = default;
    virtual void compute(Units::Angle pos_ref, Units::AngularVelocity ang_ref,
                         Units::Torque tor_ref, Status &status) = 0;
};

struct NullPolicy {
    template <typename Status>
    void compute(Units::Angle pos_ref, Units::AngularVelocity ang_ref,
                 Units::Torque tor_ref, Status &status) {}
};

} // namespace OneMotor::Motor

#endif // ONE_MOTOR_MOTORCONTROLPOLICY_HPP_
