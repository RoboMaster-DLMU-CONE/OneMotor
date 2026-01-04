#ifndef ONE_DJI_DJIPOLICY_HPP_
#define ONE_DJI_DJIPOLICY_HPP_
#include <OneMotor/Units/Units.hpp>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <one/PID/PidController.hpp>
using namespace OneMotor::Units::literals;
namespace OneMotor::Motor::DJI {

template <typename Traits, typename Chain> struct DjiPolicy {
  public:
    DjiPolicy(const Chain &chain) : m_chain(chain) {};

    int16_t compute(std::atomic<float> &pos_ref, std::atomic<float> &ang_ref,
                    std::atomic<float> &tor_ref, Traits::StatusType &status) {

        constexpr float rad_to_deg = 180.0f / std::numbers::pi_v<float>;
        float ang_result{};
        if constexpr (Chain::Size == 1) {
            const float ang_ref_rad = ang_ref.load(std::memory_order_acquire);
            float ang_ref_deg;
            if constexpr (Traits::has_gearbox) {
                ang_ref_deg =
                    ang_ref_rad * rad_to_deg * Traits::reduction_ratio;
            } else {
                ang_ref_deg = ang_ref_rad * rad_to_deg;
            }
            ang_result = m_chain.compute(ang_ref_deg, status.angular_deg_s);
        } else if constexpr (Chain::Size == 2) {
            const float pos_ref_rad = pos_ref.load(std::memory_order_acquire);
            float pos_ref_deg;
            if constexpr (Traits::has_gearbox) {
                pos_ref_deg =
                    pos_ref_rad * rad_to_deg * Traits::reduction_ratio;
            } else {
                pos_ref_deg = pos_ref_rad * rad_to_deg;
            }
            ang_result = m_chain.compute(pos_ref_deg, status.total_angle_deg,
                                         status.angular_deg_s);
        }

        return static_cast<int16_t>(ang_result);
    }

    template <size_t Index> auto &getPidController() {
        return m_chain.template get<Index>();
    }

    void resetPidChain() { m_chain.reset(); }

  private:
    Chain m_chain;
};
} // namespace OneMotor::Motor::DJI

#endif // ONE_DJI_DJIPOLICY_HPP_
