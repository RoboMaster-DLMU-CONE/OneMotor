#ifndef ONE_DJI_DJIPOLICY_HPP_
#define ONE_DJI_DJIPOLICY_HPP_

#include <OneMotor/Units/Units.hpp>

namespace OneMotor::Motor::DJI {
template <typename Traits, typename Chain> struct DjiPolicy {
  public:
    explicit DjiPolicy(const Chain &chain) : m_chain(chain) {};

    Units::CurrentMilli compute(Units::Angle pos_ref,
                                Units::AngularVelocity ang_ref,
                                Units::Torque tor_ref,
                                Traits::StatusType &status) {}

  private:
    Chain m_chain;
};
} // namespace OneMotor::Motor::DJI

#endif // ONE_DJI_DJIPOLICY_HPP_
