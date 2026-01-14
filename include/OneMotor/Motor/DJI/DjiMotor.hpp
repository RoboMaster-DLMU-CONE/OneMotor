#ifndef ONEMOTOR_DJIMOTOR_HPP
#define ONEMOTOR_DJIMOTOR_HPP
/**
 * @file DjiMotor.hpp
 * @brief 大疆电机控制实现
 */

#include "DjiFrame.hpp"
#include "DjiPolicy.hpp"
#include "DjiTraits.hpp"
#include "MotorManager.hpp"
#include "OneMotor/Thread/Othread.hpp"
#include <OneMotor/Can/CanDriver.hpp>
#include <OneMotor/Motor/MotorBase.hpp>
#include <OneMotor/Util/DoubleBuffer.hpp>
#include <OneMotor/Util/Panic.hpp>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <tl/expected.hpp>
#include <utility>

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
    DjiMotor() = default;

    explicit DjiMotor(Can::CanDriver &driver, Policy policy)
    {
        auto result = init(driver, std::move(policy));
        if (!result) {
            panic(result.error().message);
        }
    }

    tl::expected<void, Error> init(Can::CanDriver &driver, Policy policy) {
        if (auto base_result = Base::init(driver, std::move(policy));
            !base_result) {
            return base_result;
        }

        auto driver_result = this->driver();
        if (!driver_result) {
            this->resetInitialization();
            return tl::make_unexpected(driver_result.error());
        }
        auto &driver_ref = driver_result->get();
        if (auto open_result = driver_ref.open(); !open_result) {
            this->resetInitialization();
            return open_result;
        }

        MotorManager &manager = MotorManager::getInstance();
        if (auto register_result =
                manager.registerMotor(driver_ref, Traits::feedback_id());
            !register_result) {
            this->resetInitialization();
            return register_result;
        }

        manager.pushOutput(driver_ref, Traits::control_id(),
                           Traits::control_offset(), 0, 0);

        if (auto callback_result =
                driver_ref.registerCallback(
                    {Traits::feedback_id()},
                    [this](Can::CanFrame frame) { this->m_disabled_func(frame); });
            !callback_result) {
            (void)manager.deregisterMotor(driver_ref,
                                          Traits::control_id());
            this->resetInitialization();
            return callback_result;
        }

        return {};
    }

    ~DjiMotor()
    {
        MotorManager &manager = MotorManager::getInstance();
        if (!this->initialized()) {
            return;
        }
        auto driver_result = this->driver();
        if (!driver_result) {
            return;
        }
        auto &driver_ref = driver_result->get();
        (void)manager.deregisterMotor(driver_ref, Traits::control_id())
            .or_else([](const auto &) { panic("Failed to deregister motor."); });
    }

  protected:
    tl::expected<void, Error> enableImpl() override
    {
        auto driver_result = this->driver();
        if (!driver_result) {
            return tl::make_unexpected(driver_result.error());
        }
        auto &driver_ref = driver_result->get();
        return driver_ref.registerCallback(
            {Traits::feedback_id()},
            [this](Can::CanFrame frame) { this->m_enabled_func(frame); });
    }

    tl::expected<void, Error> disableImpl() override
    {
        auto driver_result = this->driver();
        if (!driver_result) {
            return tl::make_unexpected(driver_result.error());
        }
        auto &driver_ref = driver_result->get();
        return driver_ref.registerCallback(
            {Traits::feedback_id()},
            [this](Can::CanFrame frame) { this->m_disabled_func(frame); });
    }

    tl::expected<typename Traits::UserStatusType, Error> getStatusImpl()
        override
    {
        return Traits::UserStatusType::fromPlain(this->m_buffer.readCopy());
    }

    tl::expected<void, Error> afterPidParams(float kp, float ki, float kd)
        override
    {
        // 注意对DJI电机来说，默认基类的修改PID参数方法传入的参数只会覆盖至最外环
        //
        // 可以用自旋锁在回调时保护计算中的Chain
        // 但是实际上动态调整PID参数的时候应该不多
        // 目前的方法在修改参数时会略微延时，但能保证计算过程安全，且避免引入自旋锁带来的开销
        if (auto disable_result = disableImpl(); !disable_result) {
            return disable_result;
        }
        Thread::sleep_for(std::chrono::milliseconds(10));
        auto policy_result = this->getPolicy();
        if (!policy_result) {
            return tl::make_unexpected(policy_result.error());
        }
        auto &policy = policy_result->get();
        auto &node = policy.template getPidController<0>();
        node.Kp = kp;
        node.Ki = ki;
        node.Kd = kd;
        policy.resetPidChain();
        return enableImpl();
    }

    private:
        static void trMsgToStatus(const RawStatusPlain& frame,
                                  MotorStatusPlain& status)
        {
            status.ecd = frame.ecd;
            status.real_current_mA = static_cast<float>(frame.current_mA);
            status.temperature_C = static_cast<float>(frame.temperature_C);
            status.angle_single_round_deg =
                Traits::ecd_to_angle(static_cast<float>(status.ecd));
            status.angular_deg_s =
                static_cast<float>(frame.rpm) * 6.0f; // rpm -> deg/s

            if (status.ecd - status.last_ecd > 4096)
            {
                --status.total_round;
            }
            else if (status.ecd - status.last_ecd < -4096)
            {
                ++status.total_round;
            }
            status.total_angle_deg =
                static_cast<float>(status.total_round) * 360.0f +
                status.angle_single_round_deg;
            if constexpr (Traits::has_gearbox)
            {
                status.reduced_angle_deg =
                    status.total_angle_deg / Traits::reduction_ratio;
            }
            else
            {
                status.reduced_angle_deg = status.total_angle_deg;
            }
            status.last_ecd = status.ecd;
        }

        void m_disabled_func(Can::CanFrame frame)
        {
            const auto msg = RawStatusPlain(frame);
            trMsgToStatus(msg, this->m_buffer.write());
            this->m_buffer.swap();
        }

        void m_enabled_func(Can::CanFrame frame)
        {
#ifdef CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME
#if CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME != 0
            if (m_skip_frame++ < CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME)
                return;
            m_skip_frame = 0;
#endif
#endif
            const auto msg = RawStatusPlain(frame);
            trMsgToStatus(msg, this->m_buffer.write());
            auto policy_result = this->getPolicy();
            if (!policy_result) {
                return;
            }
            auto &policy = policy_result->get();
            int16_t output_current =
                policy.compute(this->m_pos_ref, this->m_ang_ref,
                               this->m_tor_ref, this->m_buffer.write());
            output_current = std::clamp(output_current,
                                        static_cast<int16_t>(-Traits::max_current),
                                        static_cast<int16_t>(Traits::max_current));

            this->m_buffer.write().output_current_mA = output_current;
            this->m_buffer.swap();
            const uint8_t hi_byte = output_current >> 8;
            const uint8_t lo_byte = output_current & 0xFF;
            auto driver_result = this->driver();
            if (!driver_result) {
                return;
            }
            auto &driver_ref = driver_result->get();
            MotorManager::getInstance().pushOutput(
                driver_ref, Traits::control_id(), Traits::control_offset(),
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

    /**
     * @brief 创建M3508电机实例的辅助函数
     * @tparam id 电机ID
     * @tparam Chain PID控制器链类型
     * @param driver CAN驱动引用
     * @param chain PID控制器链实例
     * @return M3508电机实例
     */
    template <uint8_t id, typename Chain>
    DjiMotor<M3508Traits<id>, DjiPolicy<M3508Traits<id>, Chain>>
    makeM3508(Can::CanDriver& driver, const Chain& chain)
    {
        return DjiMotor<M3508Traits<id>, DjiPolicy<M3508Traits<id>, Chain>>(
            driver, DjiPolicy<M3508Traits<id>, Chain>{chain});
    };

    template <uint8_t id, typename Chain>
    using M2006 = DjiMotor<M2006Traits<id>, DjiPolicy<M2006Traits<id>, Chain>>;

    /**
     * @brief 创建M2006电机实例的辅助函数
     * @tparam id 电机ID
     * @tparam Chain PID控制器链类型
     * @param driver CAN驱动引用
     * @param chain PID控制器链实例
     * @return M2006电机实例
     */
    template <uint8_t id, typename Chain>
    DjiMotor<M2006Traits<id>, DjiPolicy<M2006Traits<id>, Chain>>
    makeM2006(Can::CanDriver& driver, const Chain& chain)
    {
        return DjiMotor<M2006Traits<id>, DjiPolicy<M2006Traits<id>, Chain>>(
            driver, DjiPolicy<M2006Traits<id>, Chain>{chain});
    };

    template <uint8_t id, typename Chain>
    using GM6020_Voltage = DjiMotor<GM6020VoltageTraits<id>,
                                    DjiPolicy<GM6020VoltageTraits<id>, Chain>>;

    /**
     * @brief 创建GM6020电压模式电机实例的辅助函数
     * @tparam id 电机ID
     * @tparam Chain PID控制器链类型
     * @param driver CAN驱动引用
     * @param chain PID控制器链实例
     * @return GM6020电压模式电机实例
     */
    template <uint8_t id, typename Chain>
    DjiMotor<GM6020VoltageTraits<id>, DjiPolicy<GM6020VoltageTraits<id>, Chain>>
    makeGM6020_Voltage(Can::CanDriver& driver, const Chain& chain)
    {
        return DjiMotor<GM6020VoltageTraits<id>,
                        DjiPolicy<GM6020VoltageTraits<id>, Chain>>(
            driver, DjiPolicy<GM6020VoltageTraits<id>, Chain>{chain});
    };

    template <uint8_t id, typename Chain>
    using GM6020_Current = DjiMotor<GM6020CurrentTraits<id>,
                                    DjiPolicy<GM6020CurrentTraits<id>, Chain>>;

    /**
     * @brief 创建GM6020电流模式电机实例的辅助函数
     * @tparam id 电机ID
     * @tparam Chain PID控制器链类型
     * @param driver CAN驱动引用
     * @param chain PID控制器链实例
     * @return GM6020电流模式电机实例
     */
    template <uint8_t id, typename Chain>
    DjiMotor<GM6020CurrentTraits<id>, DjiPolicy<GM6020CurrentTraits<id>, Chain>>
    makeGM6020_Current(Can::CanDriver& driver, const Chain& chain)
    {
        return DjiMotor<GM6020CurrentTraits<id>,
                        DjiPolicy<GM6020CurrentTraits<id>, Chain>>(
            driver, DjiPolicy<GM6020CurrentTraits<id>, Chain>{chain});
    };
} // namespace OneMotor::Motor::DJI

#endif // ONEMOTOR_DJIMOTOR_HPP
