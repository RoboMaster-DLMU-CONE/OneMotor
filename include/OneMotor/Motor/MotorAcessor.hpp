#ifndef ONE_MOTOR_MOTORACESSOR_HPP_
#define ONE_MOTOR_MOTORACESSOR_HPP_

#include <OneMotor/Units/Units.hpp>
namespace OneMotor::Motor {

class MotorAcessor {
  public:
    virtual ~MotorAcessor() = default;
    virtual Units::Angle getPosRef() const = 0;
    virtual Units::AngularVelocity getAngRef() const = 0;
    virtual Units::Torque getTorRef() const = 0;
};

} // namespace OneMotor::Motor

#endif // ONE_MOTOR_MOTORACESSOR_HPP_
