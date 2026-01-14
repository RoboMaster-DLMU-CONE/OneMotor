#ifndef ONE_MOTOR_MOTORBASE_HPP_
#define ONE_MOTOR_MOTORBASE_HPP_
/**
 * @file MotorBase.hpp
 * @brief 电机基类定义
 */

#include <OneMotor/Can/CanDriver.hpp>
#include <OneMotor/Motor/IMotor.hpp>
#include <OneMotor/Motor/MotorConcepts.hpp>
#include <OneMotor/Units/Units.hpp>
#include <OneMotor/Util/DoubleBuffer.hpp>
#include <OneMotor/Util/Error.hpp>
#include <atomic>
#include <concepts>
#include <functional>
#include <optional>
#include <tl/expected.hpp>
#include <utility>

namespace OneMotor::Motor {

template <typename T>
concept HasPosHook = requires(T t) {
    { t.afterPosRef() } -> std::same_as<tl::template expected<void, Error>>;
};

template <typename T>
concept HasAngHook = requires(T t) {
    { t.afterAngRef() } -> std::same_as<tl::template expected<void, Error>>;
};

template <typename T>
concept HasTorHook = requires(T t) {
    { t.afterTorRef() } -> std::same_as<tl::template expected<void, Error>>;
};

template <typename T>
concept HasRefsHook = requires(T t) {
    { t.afterRefs() } -> std::same_as<tl::template expected<void, Error>>;
};

template <typename T>
concept HasPidHook = requires(T t, float p, float i, float d) {
    {
        t.afterPidParams(p, i, d)
    } -> std::same_as<tl::template expected<void, Error>>;
};

/**
 * @class MotorBase
 * @brief 电机基类，提供通用的电机控制接口
 * @tparam Derived 派生类类型
 * @tparam Traits 电机特性类型
 * @tparam Policy 控制策略类型
 */
template <typename Derived, MotorTraits Traits, typename Policy>
    requires MotorStatusType<typename Traits::StatusType> &&
             MotorStatusType<typename Traits::UserStatusType> &&
             ControlPolicy<Policy, typename Traits::StatusType>
class MotorBase : public IMotor {
  public:
    using StatusType = typename Traits::StatusType; ///< 内部线程使用的裸状态
    using UserStatusType = typename Traits::UserStatusType; ///< 对外暴露的状态
    using PolicyType = Policy;
    using TraitsType = Traits;

    MotorBase() = default;
    virtual ~MotorBase() = default;

    MotorBase(const MotorBase &) = delete;
    MotorBase &operator=(const MotorBase &) = delete;

    /**
     * @brief 初始化电机
     */
    tl::expected<void, Error> init(Can::CanDriver &driver, Policy policy) {
        if (m_initialized) {
            return tl::make_unexpected(
                Error{ErrorCode::MotorAlreadyInitialized,
                      "Motor already initialized"});
        }
        m_driver = &driver;
        m_policy.emplace(std::move(policy));
        m_initialized = true;
        return {};
    }

    /**
     * @brief 使能电机
     * @return 操作结果
     */
    tl::expected<void, Error> enable() final {
        if (auto guard = ensureInitialized(); !guard) {
            return tl::make_unexpected(guard.error());
        }
        return derived().enableImpl();
    }

    /**
     * @brief 禁用电机
     * @return 操作结果
     */
    tl::expected<void, Error> disable() final {
        if (auto guard = ensureInitialized(); !guard) {
            return tl::make_unexpected(guard.error());
        }
        return derived().disableImpl();
    }

    /**
     * @brief 设置位置参考值
     * @param ref 位置参考值
     * @return 操作结果
     */
    tl::expected<void, Error> setPosRef(const Units::Angle &ref) final {
        m_pos_ref.store(ref.numerical_value_in(mp_units::angular::radian),
                        std::memory_order_release);
        if constexpr (HasPosHook<Derived>)
            return derived().afterPosRef();
        else
            return {};
    }

    /**
     * @brief 设置角速度参考值
     * @param ref 角速度参考值
     * @return 操作结果
     */
    tl::expected<void, Error> setAngRef(const Units::AngularVelocity &ref) final {
        m_ang_ref.store(
            ref.numerical_value_in(mp_units::angular::radian /
                                   mp_units::si::second),
            std::memory_order_release);
        if constexpr (HasAngHook<Derived>)
            return derived().afterAngRef();
        else
            return {};
    }

    /**
     * @brief 设置扭矩参考值
     * @param ref 扭矩参考值
     * @return 操作结果
     */
    tl::expected<void, Error> setTorRef(const Units::Torque &ref) final {
        m_tor_ref.store(
            ref.numerical_value_in(mp_units::si::newton * mp_units::si::metre),
            std::memory_order_release);
        if constexpr (HasTorHook<Derived>)
            return derived().afterTorRef();
        else
            return {};
    }

    /**
     * @brief 同时设置位置、角速度和扭矩参考值
     * @param pos_ref 位置参考值
     * @param ang_ref 角速度参考值
     * @param tor_ref 扭矩参考值
     * @return 操作结果
     */
    tl::expected<void, Error> setRefs(const Units::Angle &pos_ref,
                                      const Units::AngularVelocity &ang_ref,
                                      const Units::Torque &tor_ref) final {
        m_pos_ref.store(pos_ref.numerical_value_in(mp_units::angular::radian),
                        std::memory_order_release);
        m_ang_ref.store(ang_ref.numerical_value_in(mp_units::angular::radian /
                                                    mp_units::si::second),
                        std::memory_order_release);
        m_tor_ref.store(tor_ref.numerical_value_in(
                             mp_units::si::newton * mp_units::si::metre),
                        std::memory_order_release);

        if constexpr (HasRefsHook<Derived>)
            return derived().afterRefs();
        else
            return {};
    };

    /**
     * @brief 设置PID参数
     * @param kp 比例参数
     * @param ki 积分参数
     * @param kd 微分参数
     * @return 操作结果
     */
    tl::expected<void, Error> setPidParams(float kp, float ki, float kd) final {
        m_kp.store(kp, std::memory_order_release);
        m_ki.store(ki, std::memory_order_release);
        m_kd.store(kd, std::memory_order_release);
        if constexpr (HasPidHook<Derived>)
            return derived().afterPidParams(kp, ki, kd);
        else
            return {};
    }

    /**
     * @brief 获取电机状态
     * @return 电机状态
     */
    tl::expected<UserStatusType, Error> getStatus() {
        if (auto guard = ensureInitialized(); !guard) {
            return tl::make_unexpected(guard.error());
        }
        return derived().getStatusImpl();
    }

    tl::expected<AnyStatus, Error> getStatusVariant() override {
        auto status = getStatus();
        if (!status) {
            return tl::make_unexpected(status.error());
        }
        return AnyStatus{*status};
    }

    /**
     * @brief 获取控制策略引用
     * @return 控制策略引用
     */
    tl::expected<std::reference_wrapper<Policy>, Error> getPolicy() {
        if (auto guard = ensureInitialized(); !guard) {
            return tl::make_unexpected(guard.error());
        }
        return std::ref(*m_policy);
    }

    /**
     * @brief 获取控制策略常量引用
     * @return 控制策略常量引用
     */
    tl::expected<std::reference_wrapper<const Policy>, Error> getPolicy()
        const {
        if (auto guard = ensureInitialized(); !guard) {
            return tl::make_unexpected(guard.error());
        }
        return std::cref(*m_policy);
    }

    /**
     * @brief 设置控制策略
     * @param policy 新的控制策略
     */
    void setPolicy(Policy policy) { m_policy.emplace(std::move(policy)); }

  protected:
    virtual tl::expected<void, Error> enableImpl() = 0;
    virtual tl::expected<void, Error> disableImpl() = 0;
    virtual tl::expected<UserStatusType, Error> getStatusImpl() = 0;

    virtual tl::expected<void, Error> afterPosRef() { return {}; }
    virtual tl::expected<void, Error> afterAngRef() { return {}; }
    virtual tl::expected<void, Error> afterTorRef() { return {}; }
    virtual tl::expected<void, Error> afterRefs() { return {}; }
    virtual tl::expected<void, Error> afterPidParams(float, float, float) {
        return {};
    }

    void resetInitialization() noexcept {
        m_initialized = false;
        m_driver = nullptr;
        m_policy.reset();
    }

    bool initialized() const noexcept {
        return m_initialized && m_driver && m_policy.has_value();
    }

    tl::expected<std::reference_wrapper<Can::CanDriver>, Error> driver() {
        if (auto guard = ensureInitialized(); !guard) {
            return tl::make_unexpected(guard.error());
        }
        return std::ref(*m_driver);
    }

    tl::expected<std::reference_wrapper<const Can::CanDriver>, Error> driver()
        const {
        if (auto guard = ensureInitialized(); !guard) {
            return tl::make_unexpected(guard.error());
        }
        return std::cref(*m_driver);
    }

    /**
     * @brief 获取位置参考值
     * @return 位置参考值
     */
    float getPosRef() const {
        return m_pos_ref.load(std::memory_order_acquire);
    }

    /**
     * @brief 获取角速度参考值
     * @return 角速度参考值
     */
    float getAngRef() const {
        return m_ang_ref.load(std::memory_order_acquire);
    }

    /**
     * @brief 获取扭矩参考值
     * @return 扭矩参考值
     */
    float getTorRef() const {
        return m_tor_ref.load(std::memory_order_acquire);
    }

    std::atomic<float> m_pos_ref{}; ///< 位置参考值 (rad)
    std::atomic<float> m_ang_ref{}; ///< 角速度参考值 (rad/s)
    std::atomic<float> m_tor_ref{}; ///< 扭矩参考值 (N·m)
    std::atomic<float> m_kp{}; ///< 比例参数
    std::atomic<float> m_ki{}; ///< 积分参数
    std::atomic<float> m_kd{}; ///< 微分参数
    DoubleBuffer<typename Traits::StatusType> m_buffer{}; ///< 状态双缓冲

    Can::CanDriver *m_driver = nullptr; ///< CAN驱动
    std::optional<Policy> m_policy; ///< 控制策略
    bool m_initialized = false;

  private:
    tl::expected<void, Error> ensureInitialized() const {
        if (initialized())
            return {};
        return tl::make_unexpected(
            Error{ErrorCode::MotorNotInitialized, "Motor not initialized"});
    }

    Derived &derived() { return static_cast<Derived &>(*this); }
    const Derived &derived() const {
        return static_cast<const Derived &>(*this);
    }
};
} // namespace OneMotor::Motor

#endif // ONE_MOTOR_MOTORBASE_HPP_
