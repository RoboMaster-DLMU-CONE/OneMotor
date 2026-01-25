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
/**
 * @brief 定义用于DJI电机的PID特性包
 *
 * 包含死区、积分限幅、输出限幅、输出滤波和微分滤波等特性
 */
using PIDFeatures =
    one::pid::FeaturePack<one::pid::WithDeadband, one::pid::WithIntegralLimit,
                          one::pid::WithOutputLimit, one::pid::WithOutputFilter,
                          one::pid::WithDerivativeFilter>;

/**
 * @class DjiMotor
 * @brief 大疆电机实现类
 * @tparam Traits 电机特性类型，定义了电机的具体属性
 * @tparam Policy 控制策略类型，定义了电机的控制算法
 *
 * DjiMotor类继承自MotorBase，实现了对大疆系列电机的控制，
 * 包括M3508、M2006、GM6020等型号。
 *
 * 该类负责处理CAN通信、状态更新、PID控制等功能。
 */
template <typename Traits, typename Policy>
class DjiMotor : public MotorBase<DjiMotor<Traits, Policy>, Traits, Policy> {
  public:
    /**
     * @typedef Base
     * @brief 基类类型别名
     */
    using Base = MotorBase<DjiMotor<Traits, Policy>, Traits, Policy>;
    friend Base;

    /**
     * @brief 默认构造函数
     */
    DjiMotor() = default;

    /**
     * @brief 构造函数
     * @param driver CAN驱动引用
     * @param policy 控制策略实例
     *
     * 构造DjiMotor实例并初始化。
     * 如果初始化失败，将触发panic。
     */
    explicit DjiMotor(Can::CanDriver &driver, Policy policy) {
        auto result = init(driver, std::move(policy));
        if (!result) {
            panic(result.error().message);
        }
    }

    /**
     * @brief 初始化电机
     * @param driver CAN驱动引用
     * @param policy 控制策略实例
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 初始化电机，包括基类初始化、CAN驱动打开、
     * 电机注册到管理器以及回调函数注册。
     */
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

        if (auto callback_result = driver_ref.registerCallback(
                {Traits::feedback_id()},
                [this](Can::CanFrame frame) { this->m_disabled_func(frame); });
            !callback_result) {
            (void)manager.deregisterMotor(driver_ref, Traits::control_id());
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
    ~DjiMotor() {
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
            .or_else(
                [](const auto &) { panic("Failed to deregister motor."); });
    }

  protected:
    /**
     * @brief 使能电机的实现方法
     * @return 操作结果
     *
     * 注册使能状态下的回调函数，用于处理电机反馈数据。
     */
    tl::expected<void, Error> enableImpl() override {
        auto driver_result = this->driver();
        if (!driver_result) {
            return tl::make_unexpected(driver_result.error());
        }
        auto &driver_ref = driver_result->get();
        return driver_ref.registerCallback(
            {Traits::feedback_id()},
            [this](Can::CanFrame frame) { this->m_enabled_func(frame); });
    }

    /**
     * @brief 禁用电机的实现方法
     * @return 操作结果
     *
     * 注册禁用状态下的回调函数，用于处理电机反馈数据。
     */
    tl::expected<void, Error> disableImpl() override {
        auto driver_result = this->driver();
        if (!driver_result) {
            return tl::make_unexpected(driver_result.error());
        }
        auto &driver_ref = driver_result->get();
        return driver_ref.registerCallback(
            {Traits::feedback_id()},
            [this](Can::CanFrame frame) { this->m_disabled_func(frame); });
    }

    /**
     * @brief 获取电机状态的实现方法
     * @return 电机状态
     *
     * 从双缓冲中获取最新的电机状态。
     */
    tl::expected<typename Traits::UserStatusType, Error>
    getStatusImpl() override {
        return Traits::UserStatusType::fromPlain(this->m_buffer.readCopy());
    }

    /**
     * @brief PID参数设置后的处理方法
     * @param kp 比例参数
     * @param ki 积分参数
     * @param kd 微分参数
     * @return 操作结果
     *
     * 更新PID控制器参数。由于DJI电机的特殊性，
     * 此方法会临时禁用电机，更新参数后再重新使能。
     *
     * @note 对DJI电机来说，默认基类的修改PID参数方法传入的参数只会覆盖至最外环
     *
     * @note 可以用自旋锁在回调时保护计算中的Chain
     *       但是实际上动态调整PID参数的时候应该不多
     *       目前的方法在修改参数时会略微延时，但能保证计算过程安全，且避免引入自旋锁带来的开销
     */
    tl::expected<void, Error> afterPidParams(float kp, float ki,
                                             float kd) override {
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
        status.angle_single_round_deg =
            Traits::ecd_to_angle(static_cast<float>(frame.ecd));
        status.angular_deg_s =
            static_cast<float>(frame.rpm) * 6.0f; // rpm -> deg/s

        [[unlikely]] if (!m_initialized) {
            m_last_ecd = frame.ecd;
            m_total_round = 0;
            // 记录当前的绝对角度作为零点偏移量
            // 注意：此时 total_round 为 0，所以绝对角度就是
            // angle_single_round_deg
            m_offset_angle = status.angle_single_round_deg;
            m_initialized = true;
        }
        if (const int32_t diff = frame.ecd - m_last_ecd; diff > 4096) {
            --status.total_round;
        } else if (diff < -4096) {
            ++status.total_round;
        }
        m_last_ecd = frame.ecd;
        float current_abs_total_angle =
            static_cast<float>(m_total_round) * 360.f +
            status.angle_single_round_deg;
        status.total_angle_deg = current_abs_total_angle - m_offset_angle;

        if constexpr (Traits::has_gearbox) {
            status.reduced_angle_deg =
                status.total_angle_deg / Traits::reduction_ratio;
            status.reduced_angular_deg_s =
                status.angular_deg_s / Traits::reduction_ratio;
        } else {
            status.reduced_angle_deg = status.total_angle_deg;
            status.reduced_angular_deg_s = status.angular_deg_s;
        }
    }

    /**
     * @brief 禁用状态下处理反馈数据的回调函数
     * @param frame CAN帧数据
     *
     * 在电机禁用状态下，仅更新状态但不进行控制计算。
     */
    void m_disabled_func(Can::CanFrame frame) {
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
    void m_enabled_func(Can::CanFrame frame) {
#ifdef CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME
#if CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME != 0
        if (m_skip_frame++ < CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME)
            return;
        m_skip_frame = 0;
#endif
#endif
        const auto msg = RawStatusPlain(frame);
        updateStatus(msg, this->m_buffer.write());
        auto policy_result = this->getPolicy();
        if (!policy_result) {
            return;
        }
        auto &policy = policy_result->get();
        int16_t output_current =
            policy.compute(this->m_pos_ref, this->m_ang_ref, this->m_tor_ref,
                           this->m_buffer.write());
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
        MotorManager::getInstance().pushOutput(driver_ref, Traits::control_id(),
                                               Traits::control_offset(),
                                               lo_byte, hi_byte);
    }

#ifdef CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME
#if CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME != 0
    uint8_t m_skip_frame{};  ///< 跳帧计数器，用于跳过指定数量的反馈帧
#endif
#endif

    /**
     * @var m_last_ecd
     * @brief 上一次的编码器读数
     *
     * 用于检测编码器数值变化，判断电机旋转方向和圈数。
     */
    uint16_t m_last_ecd = 0;

    /**
     * @var m_total_round
     * @brief 累计圈数
     *
     * 记录电机从初始化以来的总旋转圈数。
     */
    int32_t m_total_round = 0;

    /**
     * @var m_offset_angle
     * @brief 启动时的初始偏移角度
     *
     * 记录电机启动时的角度作为零点偏移，用于计算绝对角度。
     */
    float m_offset_angle = 0.0f;

    /**
     * @var m_initialized
     * @brief 是否已完成初始化
     *
     * 标记电机是否已完成初始化，用于确定是否需要记录零点偏移。
     */
    bool m_initialized = false;
};

/**
 * @brief M3508电机类型别名
 * @tparam id 电机ID
 * @tparam Chain PID控制器链类型
 */
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
makeM3508(Can::CanDriver &driver, const Chain &chain) {
    return DjiMotor<M3508Traits<id>, DjiPolicy<M3508Traits<id>, Chain>>(
        driver, DjiPolicy<M3508Traits<id>, Chain>{chain});
};

/**
 * @brief M2006电机类型别名
 * @tparam id 电机ID
 * @tparam Chain PID控制器链类型
 */
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
makeM2006(Can::CanDriver &driver, const Chain &chain) {
    return DjiMotor<M2006Traits<id>, DjiPolicy<M2006Traits<id>, Chain>>(
        driver, DjiPolicy<M2006Traits<id>, Chain>{chain});
};

/**
 * @brief GM6020电压模式电机类型别名
 * @tparam id 电机ID
 * @tparam Chain PID控制器链类型
 */
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
makeGM6020_Voltage(Can::CanDriver &driver, const Chain &chain) {
    return DjiMotor<GM6020VoltageTraits<id>,
                    DjiPolicy<GM6020VoltageTraits<id>, Chain>>(
        driver, DjiPolicy<GM6020VoltageTraits<id>, Chain>{chain});
};

/**
 * @brief GM6020电流模式电机类型别名
 * @tparam id 电机ID
 * @tparam Chain PID控制器链类型
 */
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
makeGM6020_Current(Can::CanDriver &driver, const Chain &chain) {
    return DjiMotor<GM6020CurrentTraits<id>,
                    DjiPolicy<GM6020CurrentTraits<id>, Chain>>(
        driver, DjiPolicy<GM6020CurrentTraits<id>, Chain>{chain});
};
} // namespace OneMotor::Motor::DJI

#endif // ONEMOTOR_DJIMOTOR_HPP
