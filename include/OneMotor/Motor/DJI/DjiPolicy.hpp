#ifndef ONE_DJI_DJIPOLICY_HPP_
#define ONE_DJI_DJIPOLICY_HPP_
#include <OneMotor/Motor/MotorAcessor.hpp>
#include <OneMotor/Units/Units.hpp>
#include <cstdint>
#include <one/PID/PidController.hpp>
using namespace OneMotor::Units::literals;
namespace OneMotor::Motor::DJI {

template <typename Traits, typename Chain> struct DjiPolicy {
  public:
    DjiPolicy(const Chain &chain) : m_chain(chain) {};

    int16_t compute(MotorAcessor *motor, Traits::StatusType &status) {

        float ang_result;
        if constexpr (Chain::Size == 1) {
            auto ang_ref_quantity = motor->getAngRef();
            float ang_ref;
            if constexpr (Traits::has_gearbox) {
                ang_ref = ang_ref_quantity.numerical_value_in(deg / s) *
                          Traits::reduction_ratio;
            } else {
                ang_ref = ang_ref_quantity.numerical_value_in(deg / s);
            }
            m_chain.compute(ang_ref,
                            status.angular.numerical_value_in(deg / s));
        } else if constexpr (Chain::Size == 2) {
            auto pos_ref_quantity = motor->getPosRef();
            float pos_ref;
            if constexpr (Traits::has_gearbox) {
                pos_ref = pos_ref_quantity.numerical_value_in(deg) *
                          Traits::reduction_ratio;
            } else {
                pos_ref = pos_ref_quantity.numerical_value_in(deg);
            }
            ang_result = m_chain.compute(
                pos_ref, status.total_angle.numerical_value_in(deg),
                status.angular.numerical_value_in(deg / s));
        }

        return static_cast<int16_t>(ang_result);
    }

  private:
    Chain m_chain;
};
} // namespace OneMotor::Motor::DJI

#endif // ONE_DJI_DJIPOLICY_HPP_
