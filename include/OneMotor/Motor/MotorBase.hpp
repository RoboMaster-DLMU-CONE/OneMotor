#ifndef ONE_MOTOR_MOTORBASE_HPP_
#define ONE_MOTOR_MOTORBASE_HPP_

#include "MotorAcessor.hpp"
#include <OneMotor/Can/CanDriver.hpp>
#include <OneMotor/Motor/MotorConcepts.hpp>
#include <OneMotor/Units/Units.hpp>
#include <OneMotor/Util/DoubleBuffer.hpp>
#include <OneMotor/Util/Error.hpp>
#include <atomic>
#include <tl/expected.hpp>

namespace OneMotor::Motor {

template <typename T>
concept HasPosHook = requires(T t) {
    { t.afterPosRef() };
};

template <typename T>
concept HasAngHook = requires(T t) {
    { t.afterAngRef() };
};

template <typename T>
concept HasTorHook = requires(T t) {
    { t.afterTorRef() };
};

template <typename Derived, MotorTraits Traits, typename Policy>
    requires MotorStatusType<typename Traits::StatusType> &&
             MotorStatusType<typename Traits::UserStatusType> &&
             ControlPolicy<Policy, typename Traits::StatusType>
class MotorBase : public MotorAcessor {
  public:
    using StatusType = typename Traits::StatusType;       // 内部线程使用的裸状态
    using UserStatusType = typename Traits::UserStatusType; // 对外暴露的状态
    using PolicyType = Policy;
    using TraitsType = Traits;
    explicit MotorBase(Can::CanDriver &driver, Policy policy = Policy{})
        : m_driver(driver), m_policy(std::move(policy)) {}

    tl::expected<void, Error> enable() { return derived().enableImpl(); }

    tl::expected<void, Error> disable() { return derived().disableImpl(); }

    tl::expected<void, Error> setPosRef(Units::Angle ref) {
        m_pos_ref.store(ref.numerical_value_in(mp_units::angular::radian),
                        std::memory_order_release);
        if constexpr (HasPosHook<Derived>)
            return derived().afterPosRef();
        else
            return {};
    }

    tl::expected<void, Error> setAngRef(Units::AngularVelocity ref) {
        m_ang_ref.store(
            ref.numerical_value_in(mp_units::angular::radian /
                                   mp_units::si::second),
            std::memory_order_release);
        if constexpr (HasAngHook<Derived>)
            return derived().afterAngRef();
        else
            return {};
    }

    tl::expected<void, Error> setTorRef(Units::Torque ref) {
        m_tor_ref.store(
            ref.numerical_value_in(mp_units::si::newton * mp_units::si::metre),
            std::memory_order_release);
        if constexpr (HasTorHook<Derived>)
            return derived().afterTorRef();
        else
            return {};
    }

    tl::expected<UserStatusType, Error> getStatus() {
        return derived().getStatusImpl();
    }

    Policy &getPolicy() { return m_policy; }
    const Policy &getPolicy() const { return m_policy; }

    void setPolicy(Policy policy) { m_policy = std::move(policy); }

  protected:
    std::atomic<float> m_pos_ref{}; // rad
    std::atomic<float> m_ang_ref{}; // rad/s
    std::atomic<float> m_tor_ref{}; // N·m
    DoubleBuffer<typename Traits::StatusType> m_buffer{};

    Can::CanDriver &m_driver;
    Policy m_policy;

    float getPosRef() const override {
        return m_pos_ref.load(std::memory_order_acquire);
    }
    float getAngRef() const override {
        return m_ang_ref.load(std::memory_order_acquire);
    }
    float getTorRef() const override {
        return m_tor_ref.load(std::memory_order_acquire);
    }

  private:
    Derived &derived() { return static_cast<Derived &>(*this); }
    const Derived &derived() const {
        return static_cast<const Derived &>(*this);
    }
};
} // namespace OneMotor::Motor

#endif // ONE_MOTOR_MOTORBASE_HPP_
