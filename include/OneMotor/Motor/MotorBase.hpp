#ifndef ONE_MOTOR_MOTORBASE_HPP_
#define ONE_MOTOR_MOTORBASE_HPP_

#include <OneMotor/Can/CanDriver.hpp>
#include <OneMotor/Motor/MotorConcepts.hpp>
#include <OneMotor/Units/Units.hpp>
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
class MotorBase {
  public:
    using StatusType = typename Traits::StatusType;
    using PolicyType = Policy;
    using TraitsType = Traits;
    explicit MotorBase(Can::CanDriver &driver, Policy policy = Policy{})
        : m_driver(driver), m_policy(std::move(policy)) {}

    tl::expected<void, Error> enable() { return derived().enableImpl(); }

    tl::expected<void, Error> disable() { return derived().disableImpl(); }

    tl::expected<void, Error> setPosRef(Units::Angle ref) {
        m_pos_ref.store(ref, std::memory_order_release);
        if constexpr (HasPosHook<Derived>)
            return derived().afterPosRef();
        else
            return {};
    }

    tl::expected<void, Error> setAngRef(Units::AngularVelocity ref) {
        m_ang_ref.store(ref, std::memory_order_release);
        if constexpr (HasAngHook<Derived>)
            return derived().afterAngRef();
        else
            return {};
    }

    tl::expected<void, Error> setTorRef(Units::Torque ref) {
        m_tor_ref.store(ref, std::memory_order_release);
        if constexpr (HasTorHook<Derived>)
            return derived().afterTorRef();
        else
            return {};
    }

    tl::expected<StatusType, Error> getStatus() {
        return derived().getStatusImpl();
    }

    Policy &getPolicy() { return m_policy; }
    const Policy &getPolicy() const { return m_policy; }

    void setPolicy(Policy policy) { m_policy = std::move(policy); }

  protected:
    std::atomic<Units::Angle> m_pos_ref{};
    std::atomic<Units::AngularVelocity> m_ang_ref{};
    std::atomic<Units::Torque> m_tor_ref{};

    Can::CanDriver &m_driver;
    Policy m_policy;

    Units::Angle getPosRef() const {
        return m_pos_ref.load(std::memory_order_acquire);
    }
    Units::AngularVelocity getAngRef() const {
        return m_ang_ref.load(std::memory_order_acquire);
    }
    Units::Torque getTorRef() const {
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
