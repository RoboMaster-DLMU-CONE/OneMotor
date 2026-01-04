#ifndef ONEMOTOR_DJIMOTOR_HPP
#define ONEMOTOR_DJIMOTOR_HPP
#include "DjiFrames.hpp"
#include "DjiPolicy.hpp"
#include "DjiTraits.hpp"
#include "MotorManager.hpp"
#include <OneMotor/Can/CanDriver.hpp>
#include <OneMotor/Motor/MotorBase.hpp>
#include <OneMotor/Util/DoubleBuffer.hpp>
#include <OneMotor/Util/Panic.hpp>
#include <cstdint>
#include <one/PID/PidController.hpp>

namespace OneMotor::Motor::DJI {
using PIDFeatures =
    one::pid::FeaturePack<one::pid::WithDeadband, one::pid::WithIntegralLimit,
                          one::pid::WithOutputLimit, one::pid::WithOutputFilter,
                          one::pid::WithDerivativeFilter>;
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
        trMsgToStatus(msg, this->m_buffer.write());
        this->m_buffer.swap();
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
        trMsgToStatus(msg, this->m_buffer.write());
        int16_t output_current =
            this->m_policy.compute(this, this->m_buffer.write());
        output_current = std::clamp(output_current,
                                    static_cast<int16_t>(-Traits::max_current),
                                    static_cast<int16_t>(Traits::max_current));

        this->m_buffer.write().output_current = output_current * mA;
        this->m_buffer.swap();
        const uint8_t hi_byte = output_current >> 8;
        const uint8_t lo_byte = output_current & 0xFF;
        MotorManager::getInstance().pushOutput(
            this->m_driver, Traits::control_id(), Traits::control_offset(),
            lo_byte, hi_byte);
    }

#ifdef CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME
#if CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME != 0
    uint8_t m_skip_frame{};
#endif
#endif
};

template <uint8_t id, typename Chain>
using M3508 = DjiMotor<M3508Traits<id>, DjiPolicy<M3508Traits<id>, Chain>>;

template <uint8_t id, typename Chain>
using M2006 = DjiMotor<M2006Traits<id>, DjiPolicy<M2006Traits<id>, Chain>>;

template <uint8_t id, typename Chain>
using GM6020 = DjiMotor<GM6020VoltageTraits<id>, DjiPolicy<GM6020VoltageTraits<id>, Chain>>;

} // namespace OneMotor::Motor::DJI

#endif // ONEMOTOR_DJIMOTOR_HPP
