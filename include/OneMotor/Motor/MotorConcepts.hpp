#ifndef ONE_MOTOR_MOTORCONCEPTS_HPP_
#define ONE_MOTOR_MOTORCONCEPTS_HPP_

#include "OneMotor/Motor/MotorAcessor.hpp"
#include <OneMotor/Units/Units.hpp>
#include <concepts>

namespace OneMotor {
template <typename S>
concept MotorStatusType = std::default_initializable<S> && std::copyable<S>;

template <typename P, typename Status>
concept ControlPolicy =
    requires(P policy, Motor::MotorAcessor *motor, Status &status) {
        { policy.compute(motor, status) };
    };

template <typename T>
concept MotorTraits = requires {
    { T::name } -> std::convertible_to<const char *>;
};

} // namespace OneMotor

#endif // ONE_MOTOR_MOTORCONCEPTS_HPP_
