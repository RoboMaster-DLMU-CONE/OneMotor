#ifndef ONEMOTOR_DJIMOTOR_HPP
#define ONEMOTOR_DJIMOTOR_HPP
#include <concepts>

#include "DjiFrames.hpp"
#include "DjiTraits.hpp"
#include "MotorManager.hpp"
#include "OneMotor/Motor/MotorBase.hpp"
#include <OneMotor/Can/CanDriver.hpp>
#include <OneMotor/Control/PIDChain.hpp>
#include <OneMotor/Util/DoubleBuffer.hpp>
#include <OneMotor/Util/Panic.hpp>

namespace OneMotor::Motor::DJI {

template <typename Traits, typename Policy>
class DjiMotor : public MotorBase<DjiMotor<Traits, Policy>, Traits, Policy> {
  public:
    using Base = MotorBase<DjiMotor<Traits, Policy>, Traits, Policy>;
    friend Base;
    explicit DjiMotor(Can::CanDriver &driver, Policy policy)
        : Base(driver, policy) {
        (void)this->m_driver.open().or_else(
            [](const auto &e) { panic(std::move(e.message)); });
        MotorManager &manager = MotorManager::getInstance();
        (void)manager.registerMotor(this->m_driver, Traits::feedback_id())
            .or_else([](const auto &e) { panic(std::move(e.message)); });
        manager.pushOutput(this->m_driver, Traits::control_id(),
                           Traits::control_offset(), 0, 0);

        (void)driver
            .registerCallback({Traits::feedback_id()},
                              [this](Can::CanFrame &&frame) {
                                  this->m_disabled_func(std::move(frame));
                              })
            .or_else([](const auto &e) { panic(std::move(e.message)); });
    };

    ~DjiMotor() {
        MotorManager &manager = MotorManager::getInstance();
        (void)manager.deregisterMotor(this->m_driver, Traits::control_id())
            .or_else([](const auto &e) { panic(std::move(e.message)); });
    };

  protected:
    tl::expected<void, Error> enableImpl() {
        return this->m_driver.registerCallback(
            {Traits::feedback_id()}, [this](Can::CanFrame &&frame) {
                this->m_enabled_func(std::move(frame));
            });
    }

    tl::expected<void, Error> disableImpl() {
        return this->m_driver.registerCallback(
            {Traits::feedback_id()}, [this](Can::CanFrame &&frame) {
                this->m_disabled_func(std::move(frame));
            });
    }

    MotorStatus getStatusImpl() { return this->m_buffer.readCopy(); }

  private:
    static void trMsgToStatus(const RawStatusFrame &frame,
                              MotorStatus &status) {
        auto &[last_ecd, ecd, angle_single_round, angular, real_current,
               temperature, total_angle, total_round, output_current] = status;

        ecd = frame.ecd;
        real_current = frame.current;
        temperature = frame.temperature;
        angle_single_round =
            Traits::ecd_to_angle(static_cast<float>(ecd)) * deg;
        angular = frame.rpm;

        if (ecd - last_ecd > 4096) {
            --total_round;
        } else if (ecd - last_ecd < -4096) {
            ++total_round;
        }
        total_angle = total_round + angle_single_round;
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
        this->m_Buffer.write().output_current = output_current * mA;
        this->m_Buffer.swap();
        const uint8_t hi_byte = output_current >> 8;
        const uint8_t lo_byte = output_current & 0xFF;
        MotorManager::getInstance().pushOutput(
            m_driver, Traits::template control_id<id>(),
            Traits::template control_offset<id>(), lo_byte, hi_byte);
    }

#ifdef CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME
#if CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME != 0
    uint8_t m_skip_frame{};
#endif
#endif
};

} // namespace OneMotor::Motor::DJI

#endif // ONEMOTOR_DJIMOTOR_HPP
