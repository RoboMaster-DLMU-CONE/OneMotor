#ifndef ONEMOTOR_DJIMOTOR_HPP
#define ONEMOTOR_DJIMOTOR_HPP
/**
 * @file DjiMotor.hpp
 * @brief 大疆电机控制实现
 */

#include "DjiFrame.hpp"
#include "DjiModels.hpp"
#include "DjiParam.hpp"
#include "MotorManager.hpp"
#include <one/motor/Error.hpp>

#include <one/motor/IMotor.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <one/PID/PidController.hpp>
#include <one/can/CanDriver.hpp>
#include <one/thread/Othread.hpp>
#include <one/utils/DoubleBuffer.hpp>
#include <one/utils/Panic.hpp>
#include <tl/expected.hpp>
#include <utility>

namespace one::motor::dji {
/**
 * @brief 定义用于DJI电机的PID特性包
 *
 * 包含死区、积分限幅、输出限幅、输出滤波和微分滤波等特性
 */

/**
 * @class DjiMotor
 * @brief 大疆电机实现类
 *
 * DjiMotor类继承自MotorBase，实现了对大疆系列电机的控制，
 * 包括M3508、M2006、GM6020等型号。
 *
 * 该类负责处理CAN通信、状态更新、PID控制等功能。
 */
template <typename Model> class DjiMotor : public IMotor {
  public:
    /**
     * @brief 默认构造函数
     */
    DjiMotor() = default;

    /**
     * @brief 构造函数
     * @param driver CAN驱动引用
     * @param param
     *
     * 构造DjiMotor实例并初始化。
     * 如果初始化失败，将触发panic。
     */
    explicit DjiMotor(can::CanDriver &driver, const Param &param)
        : m_param(param) {
        if (auto result = DjiMotor::init(driver, param); !result) {
            panic(result.error().message);
        }
    }

    /**
     * @brief 初始化电机
     * @param driver CAN驱动引用
     * @param param
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 初始化电机，包括基类初始化、CAN驱动打开、
     * 电机注册到管理器以及回调函数注册。
     */
    tl::expected<void, Error> init(can::CanDriver &driver, const Param &param) {
        if (auto base_result = IMotor::init(driver); !base_result) {
            return base_result;
        }
        readParam(param);

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

        manager.pushOutput(*l_driver, m_control_id, m_control_offset, 0, 0);

        if (auto callback_result = l_driver->registerCallback(
                {m_feedback_id},
                [this](can::CanFrame frame) { this->m_disabled_func(frame); });
            !callback_result) {
            (void)manager.deregisterMotor(*l_driver, m_control_id);
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
    ~DjiMotor() override {
        MotorManager &manager = MotorManager::getInstance();
        if (!this->initialized()) {
            return;
        }
        (void)manager.deregisterMotor(*this->driver(), m_control_id)
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

    tl::expected<void, Error> setParam(const Param &param) {
        if (!m_enabled) {
            readParam(param);
            return {};
        }
        return tl::make_unexpected(
            Error{ErrorCode::MotorNotDisabled,
                  "Param must be set when motor is disabled."});
    }

    void setPosUnitRef(const units::Angle &ref) noexcept override {
        m_pos_ref.store(ref.force_numerical_value_in(rad),
                        std::memory_order_release);
    };
    void setPosRef(float ref) noexcept override {
        m_pos_ref.store(ref, std::memory_order_release);
    };
    void setAngUnitRef(const units::AngularVelocity &ref) noexcept override {
        m_ang_ref.store(ref.force_numerical_value_in(rad / s),
                        std::memory_order_release);
    };
    void setAngRef(float ref) noexcept override {
        m_ang_ref.store(ref, std::memory_order_release);
    };
    void setTorUnitRef(const units::Torque &ref) noexcept override {
        m_tor_ref.store(ref.force_numerical_value_in(N * m),
                        std::memory_order_release);
    };
    void setTorRef(float ref) noexcept override {
        m_tor_ref.store(ref, std::memory_order_release);
    };
    void setUnitRefs(const units::Angle &pos_ref,
                     const units::AngularVelocity &ang_ref,
                     const units::Torque &tor_ref) noexcept override {
        m_pos_ref.store(pos_ref.force_numerical_value_in(rad),
                        std::memory_order_release);
        m_ang_ref.store(ang_ref.force_numerical_value_in(rad / s),
                        std::memory_order_release);
        m_tor_ref.store(tor_ref.force_numerical_value_in(N * m),
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
     * @brief 更新电机状态
     * @param frame 原始反馈帧数据
     * @param status 要更新的状态对象
     *
     * 根据CAN帧数据更新电机状态，包括角度、速度、电流、温度等。
     */
    void updateStatus(const RawStatusPlain &frame, MotorStatusPlain &status) {
        status.real_current_mA = static_cast<float>(frame.current_mA);
        status.temperature_C = static_cast<float>(frame.temperature_C);
        status.angle_single_round_rad =
            Model::ecd_to_angle(static_cast<float>(frame.ecd));
        status.angular_rad_s = static_cast<float>(frame.rpm) * 2.0f *
                               std::numbers::pi_v<float> /
                               60.0f; // rpm -> rad/s

        [[unlikely]] if (!m_initialized) {
            m_last_ecd = frame.ecd;
            m_total_round = 0;
            // 记录当前的绝对角度作为零点偏移量
            // 注意：此时 total_round 为 0，所以绝对角度就是
            // angle_single_round_deg
            m_offset_angle = status.angle_single_round_rad;
            m_initialized = true;
        }
        if (const int32_t diff = frame.ecd - m_last_ecd; diff > 4096) {
            --status.total_round;
        } else if (diff < -4096) {
            ++status.total_round;
        }
        m_last_ecd = frame.ecd;
        const float current_abs_total_angle =
            static_cast<float>(m_total_round) * 2 * std::numbers::pi_v<float> +
            status.angle_single_round_rad;
        status.total_angle_rad = current_abs_total_angle - m_offset_angle;

        if constexpr (Model::has_gearbox) {
            status.reduced_angle_rad =
                status.total_angle_rad / Model::reduction_ratio;
            status.reduced_angular_rad_s =
                status.angular_rad_s / Model::reduction_ratio;
        } else {
            status.reduced_angle_rad = status.total_angle_rad;
            status.reduced_angular_rad_s = status.angular_rad_s;
        }
    }

    /**
     * @brief 禁用状态下处理反馈数据的回调函数
     * @param frame CAN帧数据
     *
     * 在电机禁用状态下，仅更新状态但不进行控制计算。
     */
    void m_disabled_func(can::CanFrame frame) {
        const auto msg = RawStatusPlain(frame);
        updateStatus(msg, this->m_buffer.write());
        this->m_buffer.swap();
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
        const auto msg = RawStatusPlain(frame);
        updateStatus(msg, this->m_buffer.write());
        int16_t output_current = m_compute_func();
        output_current = std::clamp(output_current,
                                    static_cast<int16_t>(-Model::max_current),
                                    static_cast<int16_t>(Model::max_current));
        const uint8_t hi_byte = output_current >> 8;
        const uint8_t lo_byte = output_current & 0xFF;
        MotorManager::getInstance().pushOutput(
            *this->driver(), m_control_id, m_control_offset, lo_byte, hi_byte);
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
    uint16_t m_last_ecd{};

    /**
     * @var m_total_round
     * @brief 累计圈数
     *
     * 记录电机从初始化以来的总旋转圈数。
     */
    int32_t m_total_round{};

    /**
     * @var m_offset_angle
     * @brief 启动时的初始偏移角度
     *
     * 记录电机启动时的角度作为零点偏移，用于计算绝对角度。
     */
    float m_offset_angle{};

    /**
     * @var m_initialized
     * @brief 是否已完成初始化
     *
     * 标记电机是否已完成初始化，用于确定是否需要记录零点偏移。
     */
    bool m_initialized = false;

    bool m_enabled = false;

    uint16_t computeAng() {
        const float ang_ref_rad = m_ang_ref.load(std::memory_order_acquire);
        return static_cast<uint16_t>(
            std::get<AngMode>(m_param.mode)
                .pid_controller.compute(
                    ang_ref_rad, m_buffer.write().reduced_angular_rad_s));
    }

    uint16_t computePosAng() {
        // const float ang_ref_rad = m_ang_ref.load(std::memory_order_acquire);
        // if constexpr (Model::has_gearbox) {
        //     ang_ref_rad *= Model::reduction_ratio;
        // }
        const float pos_ref_rad = m_pos_ref.load(std::memory_order_acquire);
        auto status = m_buffer.write();
        return static_cast<uint16_t>(
            std::get<PosAngMode>(m_param.mode)
                .pid_chain.compute(
                    pos_ref_rad,
                    {status.reduced_angle_rad, status.reduced_angular_rad_s}));
    }
    uint16_t computeMIT() {
        auto [kp, kd] = std::get<MITMode>(m_param.mode);
        const auto status = m_buffer.write();
        const float tff = m_tor_ref.load(std::memory_order_acquire);
        const float pdes = m_pos_ref.load(std::memory_order_acquire);
        const float qdes = m_ang_ref.load(std::memory_order_acquire);
        return static_cast<uint16_t>(
            tff + kp * (pdes - status.reduced_angle_rad) +
            kd * (qdes - status.reduced_angular_rad_s));
    }

    void readParam(const Param &param) {
        m_param = param;
        m_control_id = Model::control_id(m_param.id);
        m_control_offset = Model::control_offset(m_param.id);
        m_feedback_id = Model::feedback_id(m_param.id);
        std::visit(
            [this]<typename T>(T &mode) {
                if constexpr (std::same_as<T, MITMode>) {
                    m_compute_func = [this] { return this->computeMIT(); };
                } else if constexpr (std::same_as<T, PosAngMode>) {
                    m_compute_func = [this] { return this->computePosAng(); };
                } else {
                    m_compute_func = [this] { return this->computeAng(); };
                }
            },
            m_param.mode);
    }

    std::function<uint16_t()> m_compute_func{};

    Param m_param{0, MITMode{}};

    uint16_t m_control_id{};
    uint8_t m_control_offset{};
    uint16_t m_feedback_id{};

    std::atomic<float> m_pos_ref{};
    std::atomic<float> m_ang_ref{};
    std::atomic<float> m_tor_ref{};

    DoubleBuffer<MotorStatusPlain> m_buffer{};
};

using M3508 = DjiMotor<M3508Model>;
using M2006 = DjiMotor<M2006Model>;
using GM6020_Voltage = DjiMotor<GM6020VoltageModel>;
using GM6020_Current = DjiMotor<GM6020CurrentModel>;

} // namespace one::motor::dji

#endif // ONEMOTOR_DJIMOTOR_HPP
