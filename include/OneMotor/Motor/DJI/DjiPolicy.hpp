#ifndef ONE_DJI_DJIPOLICY_HPP_
#define ONE_DJI_DJIPOLICY_HPP_

#include <OneMotor/Motor/MotorAcessor.hpp>
#include <OneMotor/Units/Units.hpp>
#include <cstdint>
using namespace OneMotor::Units::literals;
namespace OneMotor::Motor::DJI {
template <typename Traits, typename Chain> struct DjiPolicy {
  public:
    DjiPolicy(const Chain &chain) : m_chain(chain) {};

    int16_t compute(MotorAcessor *motor, Traits::StatusType &status) {
        float ang_result;
        if constexpr (Chain::Size == 1) {
            auto ang_ref = motor->getAngRef();
            ang_result =
                m_chain.compute(ang_ref.numerical_value_in(deg / s),
                                status.angular.numerical_value_in(deg / s));
        } else if constexpr (Chain::Size == 2) {
            auto pos_ref = motor->getPosRef();
            ang_result =
                m_chain.compute(pos_ref.numerical_value_in(deg),
                                status.total_angle.numerical_value_in(deg),
                                status.angular.numerical_value_in(deg / s));
        }

        return static_cast<int16_t>(ang_result);
    }

  private:
    Chain m_chain;
};
} // namespace OneMotor::Motor::DJI

#endif // ONE_DJI_DJIPOLICY_HPP_
