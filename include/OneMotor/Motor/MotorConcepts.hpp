#ifndef ONE_MOTOR_MOTORCONCEPTS_HPP_
#define ONE_MOTOR_MOTORCONCEPTS_HPP_

#include <OneMotor/Units/Units.hpp>
#include <atomic>
#include <concepts>

namespace OneMotor {
template <typename S>
concept MotorStatusType = std::default_initializable<S> && std::copyable<S>;

template <typename P, typename Status>
concept ControlPolicy =
    requires(P policy, std::atomic<float> &pos_ref, std::atomic<float> &ang_ref,
             std::atomic<float> &tor_ref, Status &status) {
        { policy.compute(pos_ref, ang_ref, tor_ref, status) };
    };

template <typename T>
concept MotorTraits = requires {
    { T::name } -> std::convertible_to<const char *>;
};

} // namespace OneMotor

#endif // ONE_MOTOR_MOTORCONCEPTS_HPP_
