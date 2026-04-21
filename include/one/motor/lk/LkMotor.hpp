#ifndef ONE_MOTOR_LK_DMMOTOR_HPP_
#define ONE_MOTOR_LK_DMMOTOR_HPP_

#include "LkFrame.hpp"
#include <one/motor/IMotor.hpp>

#include "LkFrame.hpp"
#include "MotorManager.hpp"
#include "one/utils/DoubleBuffer.hpp"

#include <cstdint>
#include <cstring>
#include <one/can/CanDriver.hpp>
#include <one/can/CanFrame.hpp>
#include <one/motor/Error.hpp>
#include <one/motor/Units.hpp>
#include <one/utils/Panic.hpp>

namespace one::motor::lk {
namespace detail {

static constexpr double rad2dps(const float rad_per_sec) noexcept {
    return rad_per_sec * 180.0f * std::numbers::pi_v<float>;
}

} // namespace detail
template <uint8_t id> class LkMotor : public IMotor {
  public:
    /**
     * @brief 默认构造函数
     */
    LkMotor() = default;

    /**
     * @brief 构造函数
     * @param driver CAN驱动引用
     * @param param
     *
     * 构造LkMotor实例并初始化。
     * 如果初始化失败，将触发panic。
     */
    explicit LkMotor(can::CanDriver &driver) {
        if (auto result = LkMotor::init(driver); !result) {
            panic(result.error().message);
        }
    }

    /**
     * @brief 初始化电机
     * @param driver CAN驱动引用
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 初始化电机，包括基类初始化、CAN驱动打开、
     * 电机注册到管理器以及回调函数注册。
     */
    tl::expected<void, Error> init(can::CanDriver &driver) {
        if (const auto base_result = IMotor::init(driver); !base_result) {
            return base_result;
        }

        auto l_driver = this->driver();
        if (auto open_result = l_driver->open(); !open_result) {
            this->resetInitialization();
            return open_result;
        }

        MotorManager &manager = MotorManager::getInstance();
        if (auto register_result =
                manager.registerMotor(*l_driver, m_feedback_id);
            !register_result) {
            this->resetInitialization();
            return register_result;
        }

        // currently only support speed mode
        manager.pushOutput(*l_driver, 0x281, m_control_offset, 0, 0);

        if (auto callback_result = l_driver->registerCallback(
                {m_feedback_id},
                [this](can::CanFrame frame) { this->m_disabled_func(frame); });
            !callback_result) {
            (void)manager.deregisterMotor(*l_driver, m_feedback_id);
            this->resetInitialization();
            return callback_result;
        }

        return {};
    }

    /**
     * @brief 析构函数
     *
     * 在析构时注销电机，确保资源正确释放。
     */
    ~LkMotor() override {
        MotorManager &manager = MotorManager::getInstance();
        if (!this->initialized()) {
            return;
        }
        (void)manager.deregisterMotor(*this->driver(), m_feedback_id)
            .or_else(
                [](const auto &) { panic("Failed to deregister motor."); });
    }

    /**
     * @brief 使能电机的实现方法
     * @return 操作结果
     *
     * 注册使能状态下的回调函数，用于处理电机反馈数据。
     */
    tl::expected<void, Error> enable() override {
        if (!this->initialized()) {
            return tl::make_unexpected(Error{ErrorCode::MotorNotInitialized,
                                             "Motor has not been initialized"});
        }
        auto res = this->driver()->registerCallback(
            {m_feedback_id},
            [this](can::CanFrame frame) { this->m_enabled_func(frame); });
        if (res) {
            m_enabled = true;
        }
        return res;
    }

    /**
     * @brief 禁用电机的实现方法
     * @return 操作结果
     *
     * 注册禁用状态下的回调函数，用于处理电机反馈数据。
     */
    tl::expected<void, Error> disable() override {
        if (!this->initialized()) {
            return tl::make_unexpected(Error{ErrorCode::MotorNotInitialized,
                                             "Motor has not been initialized"});
        }
        auto res = this->driver()->registerCallback(
            {m_feedback_id},
            [this](can::CanFrame frame) { this->m_disabled_func(frame); });
        if (res) {
            m_enabled = false;
        }
        return res;
    }

    tl::expected<AnyStatus, Error> getStatusVariant() override {
        return MotorStatus::fromPlain(this->m_buffer.readCopy());
    }

    tl::expected<AnyPlainStatus, Error> getPlainStatusVariant() override {
        return this->m_buffer.readCopy();
    }

    MotorStatus getStatus() {
        return MotorStatus::fromPlain(this->m_buffer.readCopy());
    }

    MotorStatusPlain getStatusPlain() { return this->m_buffer.readCopy(); }

    void setPosUnitRef(const units::Angle &ref) noexcept override {
        m_pos_ref.store(ref.numerical_value_in(units::radian),
                        std::memory_order_release);
    };

    void setPosRef(const float ref) noexcept override {
        m_pos_ref.store(ref, std::memory_order_release);
    };

    void setAngUnitRef(const units::AngularVelocity &ref) noexcept override {
        m_ang_ref.store(ref.numerical_value_in(units::radian / units::second),
                        std::memory_order_release);
    };

    void setAngRef(float ref) noexcept override {
        m_ang_ref.store(ref, std::memory_order_release);
    };

    void setTorUnitRef(const units::Torque &ref) noexcept override {
        m_tor_ref.store(ref.numerical_value_in(units::newton * units::metre),
                        std::memory_order_release);
    };

    void setTorRef(float ref) noexcept override {
        m_tor_ref.store(ref, std::memory_order_release);
    };

    void setUnitRefs(const units::Angle &pos_ref,
                     const units::AngularVelocity &ang_ref,
                     const units::Torque &tor_ref) noexcept override {
        m_pos_ref.store(pos_ref.numerical_value_in(units::radian),
                        std::memory_order_release);
        m_ang_ref.store(
            ang_ref.numerical_value_in(units::radian / units::second),
            std::memory_order_release);
        m_tor_ref.store(
            tor_ref.numerical_value_in(units::newton * units::metre),
            std::memory_order_release);
    };

    void setRefs(float pos_ref, float ang_ref,
                 float tor_ref) noexcept override {
        m_pos_ref.store(pos_ref, std::memory_order_release);
        m_ang_ref.store(ang_ref, std::memory_order_release);
        m_tor_ref.store(tor_ref, std::memory_order_release);
    };

  private:
    /**
     * @brief 禁用状态下处理反馈数据的回调函数
     * @param frame CAN帧数据
     *
     * 在电机禁用状态下，仅更新状态但不进行控制计算。
     */
    void m_disabled_func(const can::CanFrame frame) {
        m_buffer.write() = MotorStatusPlain(frame);
        m_buffer.swap();
    }

    /**
     * @brief 使能状态下处理反馈数据的回调函数
     * @param frame CAN帧数据
     *
     * 在电机使能状态下，更新状态并根据PID控制器计算输出电流。
     */
    void m_enabled_func(can::CanFrame frame) {
#ifdef CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME
#if CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME != 0
        if (m_skip_frame++ < CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME)
            return;
        m_skip_frame = 0;
#endif
#endif
        m_buffer.write() = MotorStatusPlain(frame);
        int16_t output_angular =
            static_cast<int16_t>(detail::rad2dps(std::clamp(
                m_ang_ref.load(std::memory_order_acquire),
                static_cast<float>(std::numeric_limits<int16_t>::min()),
                static_cast<float>(std::numeric_limits<int16_t>::max()))));
        const uint8_t hi_byte = output_angular >> 8;
        const uint8_t lo_byte = output_angular & 0xFF;
        MotorManager::getInstance().pushOutput(
            *this->driver(), 0x281, m_control_offset, lo_byte, hi_byte);
        m_buffer.swap();
    }

#ifdef CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME
#if CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME != 0
    uint8_t m_skip_frame{}; ///< 跳帧计数器，用于跳过指定数量的反馈帧
#endif
#endif

    /**
     * @var m_last_ecd
     * @brief 上一次的编码器读数
     *
     * 用于检测编码器数值变化，判断电机旋转方向和圈数。
     */
    // uint16_t m_last_ecd{};

    /**
     * @var m_total_round
     * @brief 累计圈数
     *
     * 记录电机从初始化以来的总旋转圈数。
     */
    // int32_t m_total_round{};

    /**
     * @var m_offset_angle
     * @brief 启动时的初始偏移角度
     *
     * 记录电机启动时的角度作为零点偏移，用于计算绝对角度。
     */
    // float m_offset_angle{};

    /**
     * @var m_initialized
     * @brief 是否已完成初始化
     *
     * 标记电机是否已完成初始化，用于确定是否需要记录零点偏移。
     */
    bool m_initialized = false;
    bool m_enabled = false;

    uint8_t m_control_offset{};
    static constexpr uint16_t m_feedback_id = id + 0x140;

    std::atomic<float> m_pos_ref{};
    std::atomic<float> m_ang_ref{};
    std::atomic<float> m_tor_ref{};

    DoubleBuffer<MotorStatusPlain> m_buffer{};
};

} // namespace one::motor::lk

#endif // ONE_MOTOR_DM_DMMOTOR_HPP_
