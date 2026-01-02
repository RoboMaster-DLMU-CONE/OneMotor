#ifndef ONE_MOTOR_MOTORBASE_HPP_
#define ONE_MOTOR_MOTORBASE_HPP_

#include <OneMotor/Can/CanDriver.hpp>
#include <OneMotor/Motor/MotorConcepts.hpp>
#include <OneMotor/Units/Units.hpp>
#include <atomic>

namespace OneMotor::Motor {
template <typename Derived, MotorTraits Traits, typename Policy>
class MotorBase {
  public:
    using StatusType = typename Traits::StatusType;
    using PolicyType = Policy;
    using TraitsType = Traits;
    explicit MotorBase(Can::CanDriver &driver, Policy policy = Policy{}) {}

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
