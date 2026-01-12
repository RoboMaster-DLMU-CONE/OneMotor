#ifndef ONE_MOTOR_MOTORCONCEPTS_HPP_
#define ONE_MOTOR_MOTORCONCEPTS_HPP_
/**
 * @file MotorConcepts.hpp
 * @brief 电机相关概念定义
 */

#include <OneMotor/Units/Units.hpp>
#include <atomic>
#include <concepts>

namespace OneMotor {
/**
 * @brief 电机状态类型概念
 * @tparam S 状态类型
 */
template <typename S>
concept MotorStatusType = std::default_initializable<S> && std::copyable<S>;

/**
 * @brief 控制策略概念
 * @tparam P 策略类型
 * @tparam Status 状态类型
 */
template <typename P, typename Status>
concept ControlPolicy =
    requires(P policy, std::atomic<float> &pos_ref, std::atomic<float> &ang_ref,
             std::atomic<float> &tor_ref, Status &status) {
        { policy.compute(pos_ref, ang_ref, tor_ref, status) };
    };

/**
 * @brief 电机特性概念
 * @tparam T 特性类型
 */
template <typename T>
concept MotorTraits = requires {
    { T::name } -> std::convertible_to<const char *>;
};

} // namespace OneMotor

#endif // ONE_MOTOR_MOTORCONCEPTS_HPP_
