#ifndef ONE_MOTOR_IMOTOR_HPP_
#define ONE_MOTOR_IMOTOR_HPP_
/**
 * @file IMotor.hpp
 * @brief 非模板电机接口，允许使用类型擦除的状态访问
 */

#include <OneMotor/Motor/DM/DmFrame.hpp>
#include <OneMotor/Motor/DJI/DjiFrame.hpp>
#include <OneMotor/Units/Units.hpp>
#include <OneMotor/Util/Error.hpp>
#include <tl/expected.hpp>
#include <variant>

namespace OneMotor::Motor {

using AnyStatus = std::variant<DM::DmStatus, DJI::MotorStatus>;

class IMotor {
  public:
    virtual ~IMotor() = default;
    virtual tl::expected<void, Error> enable() = 0;
    virtual tl::expected<void, Error> disable() = 0;
    virtual tl::expected<void, Error> setPosRef(const Units::Angle &ref) = 0;
    virtual tl::expected<void, Error> setAngRef(
        const Units::AngularVelocity &ref) = 0;
    virtual tl::expected<void, Error> setTorRef(const Units::Torque &ref) = 0;
    virtual tl::expected<void, Error> setRefs(const Units::Angle &pos_ref,
                                              const Units::AngularVelocity &ang_ref,
                                              const Units::Torque &tor_ref) = 0;
    virtual tl::expected<void, Error> setPidParams(float kp, float ki,
                                                   float kd) = 0;
    virtual tl::expected<AnyStatus, Error> getStatusVariant() = 0;
};
} // namespace OneMotor::Motor

#endif // ONE_MOTOR_IMOTOR_HPP_
