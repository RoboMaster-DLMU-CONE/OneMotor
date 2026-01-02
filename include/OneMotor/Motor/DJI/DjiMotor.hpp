#ifndef ONEMOTOR_DJIMOTOR_HPP
#define ONEMOTOR_DJIMOTOR_HPP
#include <concepts>

#include "DjiMotorFrames.hpp"
#include "MotorManager.hpp"
#include "Traits.hpp"
#include <OneMotor/Can/CanDriver.hpp>
#include <OneMotor/Control/PIDChain.hpp>
#include <OneMotor/Util/DoubleBuffer.hpp>
#include <OneMotor/Util/Panic.hpp>

namespace OneMotor::Motor::DJI {
using PIDFeatures =
    Control::FeaturePack<Control::WithDeadband, Control::WithIntegralLimit,
                         Control::WithOutputLimit, Control::WithOutputFilter,
                         Control::WithDerivativeFilter>;

template <typename Traits, uint8_t id>
concept ValidMotorId = requires {
    { Traits::max_id } -> std::convertible_to<uint8_t>;
} && (id >= 1 && id <= Traits::max_id);

template <typename Traits, uint8_t id, typename... PID_Nodes>
    requires ValidMotorId<Traits, id>
class DjiMotor {
  public:
    explicit DjiMotor(Can::CanDriver &driver,
                      const Control::PIDChain<PID_Nodes...> &pid_chain)
        : m_driver(driver), m_pid_chain(pid_chain) {
        (void)m_driver.open().or_else(
            [](const auto &e) { panic(std::move(e.message)); });
        MotorManager &manager = MotorManager::getInstance();
        (void)manager
            .registerMotor(m_driver, Traits::template feedback_id<id>())
            .or_else([](const auto &e) { panic(std::move(e.message)); });
        manager.pushOutput(m_driver, Traits::template control_id<id>(),
                           Traits::template control_offset<id>(), 0, 0);

        (void)driver
            .registerCallback({Traits::template feedback_id<id>()},
                              [this](Can::CanFrame &&frame) {
                                  this->m_disabled_func(std::move(frame));
                              })
            .or_else([](const auto &e) { panic(std::move(e.message)); });
    };

    ~DjiMotor() {
        MotorManager &manager = MotorManager::getInstance();
        (void)manager
            .deregisterMotor(m_driver, Traits::template control_id<id>())
            .or_else([](const auto &e) { panic(std::move(e.message)); });
    };

    tl::expected<void, Error> enable() {
        return m_driver.registerCallback({Traits::template feedback_id<id>()},
                                         [this](Can::CanFrame &&frame) {
                                             this->m_enabled_func(
                                                 std::move(frame));
                                         });
    }

    tl::expected<void, Error> disable() {
        return m_driver.registerCallback({Traits::template feedback_id<id>()},
                                         [this](Can::CanFrame &&frame) {
                                             this->m_disabled_func(
                                                 std::move(frame));
                                         });
    }

    void setAngRef(const float ref) {
        m_ang_ref.store(ref, std::memory_order_release);
    }

    void setPosRef(const float ref) {
        m_pos_ref.store(ref, std::memory_order_release);
    }

    MotorStatus getStatus() { return m_Buffer.readCopy(); }

  private:
    static constexpr float RPM_2_ANGLE_PER_SEC = 6.0f;

    static void trMsgToStatus(const RawStatusFrame &frame,
                              MotorStatus &status) {
        auto &[last_ecd, ecd, angle_single_round, angular, real_current,
               temperature, total_angle, total_round, output_current] = status;

        ecd = frame.ecd;
        real_current = frame.current;
        temperature = frame.temperature;
        angle_single_round = Traits::ecd_to_angle(static_cast<float>(ecd));
        angular = RPM_2_ANGLE_PER_SEC * static_cast<float>(frame.rpm);

        if (ecd - last_ecd > 4096) {
            total_round--;
        } else if (ecd - last_ecd < -4096) {
            total_round++;
        }
        total_angle =
            static_cast<float>(total_round) * 360 + angle_single_round;
        last_ecd = ecd;
    }

    void m_disabled_func(Can::CanFrame &&frame) {
        const auto msg = static_cast<RawStatusFrame>(frame);
        trMsgToStatus(msg, this->m_Buffer.write());
        this->m_Buffer.swap();
    }

    void m_enabled_func(Can::CanFrame &&frame) {
#ifdef CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME
#if CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME != 0
        if (m_skip_frame++ < CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME)
            return;
        m_skip_frame = 0;
#endif
#endif
        const auto msg = static_cast<RawStatusFrame>(frame);
        trMsgToStatus(msg, this->m_Buffer.write());
        float ang_result{};
        if constexpr (Control::PIDChain<PID_Nodes...>::Size == 1) {
            ang_result =
                m_pid_chain.compute(m_ang_ref.load(std::memory_order_acquire),
                                    this->m_Buffer.write().angular);
        } else if constexpr (Control::PIDChain<PID_Nodes...>::Size == 2) {
            ang_result =
                m_pid_chain.compute(m_pos_ref.load(std::memory_order_acquire),
                                    this->m_Buffer.write().total_angle,
                                    this->m_Buffer.write().angular);
        }
        ang_result =
            std::clamp(ang_result, static_cast<float>(-Traits::max_current),
                       static_cast<float>(Traits::max_current));
        const auto output_current = static_cast<int16_t>(ang_result);
        this->m_Buffer.write().output_current = output_current;
        this->m_Buffer.swap();
        const uint8_t hi_byte = output_current >> 8;
        const uint8_t lo_byte = output_current & 0xFF;
        MotorManager::getInstance().pushOutput(
            m_driver, Traits::template control_id<id>(),
            Traits::template control_offset<id>(), lo_byte, hi_byte);
    }

    Can::CanDriver &m_driver;
    DoubleBuffer<MotorStatus> m_Buffer;
    Control::PIDChain<PID_Nodes...> m_pid_chain;

    std::atomic<float> m_ang_ref{};
    std::atomic<float> m_pos_ref{};
#ifdef CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME
#if CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME != 0
    uint8_t m_skip_frame{};
#endif
#endif
};

template <typename Traits, uint8_t id, typename... PID_Nodes>
DjiMotor<Traits, id, PID_Nodes...>
createDjiMotor(Can::CanDriver &driver,
               const Control::PIDChain<PID_Nodes...> &pid_chain) {
    return DjiMotor<Traits, id, PID_Nodes...>(driver, pid_chain);
}
} // namespace OneMotor::Motor::DJI

#endif // ONEMOTOR_DJIMOTOR_HPP
