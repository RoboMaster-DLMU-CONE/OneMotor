#ifndef ONE_MOTOR_MOTORACESSOR_HPP_
#define ONE_MOTOR_MOTORACESSOR_HPP_

#include <OneMotor/Units/Units.hpp>
namespace OneMotor::Motor {

class MotorAcessor {
  public:
    virtual ~MotorAcessor() = default;
    // 数值均使用基础单位：rad / (rad/s) / N·m
    virtual float getPosRef() const = 0;
    virtual float getAngRef() const = 0;
    virtual float getTorRef() const = 0;
};

} // namespace OneMotor::Motor

#endif // ONE_MOTOR_MOTORACESSOR_HPP_
