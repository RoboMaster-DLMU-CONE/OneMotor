#ifndef ONE_MOTOR_MOTORCONCEPTS_HPP_
#define ONE_MOTOR_MOTORCONCEPTS_HPP_
/**
 * @file MotorConcepts.hpp
 * @brief 电机相关概念定义
 *
 * 该文件定义了电机控制系统中使用的关键概念（concepts），
 * 用于在编译时验证类型是否满足特定要求。
 */

#include <OneMotor/Units/Units.hpp>
#include <atomic>
#include <concepts>

namespace OneMotor {
/**
 * @brief 电机状态类型概念
 * @tparam S 状态类型
 *
 * 验证类型S是否可以作为电机状态类型使用。
 * 电机状态类型必须是可默认构造和可复制的。
 */
template <typename S>
concept MotorStatusType = std::default_initializable<S> && std::copyable<S>;

/**
 * @brief 控制策略概念
 * @tparam P 策略类型
 * @tparam Status 状态类型
 *
 * 验证类型P是否可以作为控制策略使用。
 * 控制策略必须有一个compute方法，接受位置、角速度、扭矩参考值和状态，
 * 并返回计算结果。
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
 *
 * 验证类型T是否可以作为电机特性使用。
 * 电机特性必须有一个名为name的成员，可以转换为const char*。
 */
template <typename T>
concept MotorTraits = requires {
    { T::name } -> std::convertible_to<const char *>;
};

} // namespace OneMotor

#endif // ONE_MOTOR_MOTORCONCEPTS_HPP_
