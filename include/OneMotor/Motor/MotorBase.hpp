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

/**
 * @defgroup MotorHooks 电机钩子概念
 * @{
 * 这些概念定义了派生类可以实现的钩子函数，以便在某些操作后执行额外逻辑
 */

/**
 * @brief 概念HasPosHook，用于检查类型T是否具有afterPosRef方法
 * @tparam T 要检查的类型
 */
template <typename T>
concept HasPosHook = requires(T t) {
    { t.afterPosRef() } -> std::same_as<tl::template expected<void, Error>>;
};

/**
 * @brief 概念HasAngHook，用于检查类型T是否具有afterAngRef方法
 * @tparam T 要检查的类型
 */
template <typename T>
concept HasAngHook = requires(T t) {
    { t.afterAngRef() } -> std::same_as<tl::template expected<void, Error>>;
};

/**
 * @brief 概念HasTorHook，用于检查类型T是否具有afterTorRef方法
 * @tparam T 要检查的类型
 */
template <typename T>
concept HasTorHook = requires(T t) {
    { t.afterTorRef() } -> std::same_as<tl::template expected<void, Error>>;
};

/**
 * @brief 概念HasRefsHook，用于检查类型T是否具有afterRefs方法
 * @tparam T 要检查的类型
 */
template <typename T>
concept HasRefsHook = requires(T t) {
    { t.afterRefs() } -> std::same_as<tl::template expected<void, Error>>;
};

/**
 * @brief 概念HasPidHook，用于检查类型T是否具有afterPidParams方法
 * @tparam T 要检查的类型
 */
template <typename T>
concept HasPidHook = requires(T t, float p, float i, float d) {
    {
        t.afterPidParams(p, i, d)
    } -> std::same_as<tl::template expected<void, Error>>;
};

/** @} */ // end of MotorHooks

/**
 * @class MotorBase
 * @brief 电机基类，提供通用的电机控制接口
 * @tparam Derived 派生类类型，遵循CRTP模式
 * @tparam Traits 电机特性类型，定义了电机的具体属性和行为
 * @tparam Policy 控制策略类型，定义了电机的控制算法
 *
 * MotorBase是一个抽象基类，提供了电机控制的基本功能接口，
 * 包括使能/禁用、设置参考值、获取状态等功能。
 *
 * 该类使用CRTP（奇异递归模板模式）来实现静态多态性，
 * 避免了虚函数调用的开销，同时保持了接口的一致性。
 */
template <typename Derived, MotorTraits Traits, typename Policy>
    requires MotorStatusType<typename Traits::StatusType> &&
             MotorStatusType<typename Traits::UserStatusType> &&
             ControlPolicy<Policy, typename Traits::StatusType>
class MotorBase : public IMotor {
  public:
    /**
     * @typedef StatusType
     * @brief 内部线程使用的原始状态类型
     */
    using StatusType = typename Traits::StatusType;

    /**
     * @typedef UserStatusType
     * @brief 对外暴露的用户友好的状态类型
     */
    using UserStatusType = typename Traits::UserStatusType;

    /**
     * @typedef PolicyType
     * @brief 控制策略类型
     */
    using PolicyType = Policy;

    /**
     * @typedef TraitsType
     * @brief 电机特性类型
     */
    using TraitsType = Traits;

    /**
     * @brief 默认构造函数
     */
    MotorBase() = default;

    /**
     * @brief 虚析构函数
     */
    virtual ~MotorBase() = default;

    /**
     * @brief 删除拷贝构造函数
     */
    MotorBase(const MotorBase &) = delete;

    /**
     * @brief 删除赋值运算符
     */
    MotorBase &operator=(const MotorBase &) = delete;

    /**
     * @brief 初始化电机
     * @param driver CAN驱动引用
     * @param policy 控制策略实例
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 该函数初始化电机，设置CAN驱动和控制策略。
     * 如果电机已经初始化，则返回错误。
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
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 使能电机，允许电机响应控制指令。
     * 具体实现由派生类提供。
     */
    tl::expected<void, Error> enable() final {
        if (auto guard = ensureInitialized(); !guard) {
            return tl::make_unexpected(guard.error());
        }
        return derived().enableImpl();
    }

    /**
     * @brief 禁用电机
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 禁用电机，停止电机响应控制指令。
     * 具体实现由派生类提供。
     */
    tl::expected<void, Error> disable() final {
        if (auto guard = ensureInitialized(); !guard) {
            return tl::make_unexpected(guard.error());
        }
        return derived().disableImpl();
    }

    /**
     * @brief 设置位置参考值
     * @param ref 位置参考值，使用Units::Angle类型
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 设置电机的目标位置，如果派生类实现了HasPosHook概念，
     * 则在设置后调用相应的钩子函数。
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
     * @param ref 角速度参考值，使用Units::AngularVelocity类型
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 设置电机的目标角速度，如果派生类实现了HasAngHook概念，
     * 则在设置后调用相应的钩子函数。
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
     * @param ref 扭矩参考值，使用Units::Torque类型
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 设置电机的目标扭矩，如果派生类实现了HasTorHook概念，
     * 则在设置后调用相应的钩子函数。
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
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 同时设置电机的位置、角速度和扭矩参考值，
     * 如果派生类实现了HasRefsHook概念，则在设置后调用相应的钩子函数。
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
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 设置PID控制器的参数，如果派生类实现了HasPidHook概念，
     * 则在设置后调用相应的钩子函数。
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
     * @return 电机状态，成功返回UserStatusType，失败返回Error
     *
     * 获取电机的当前状态，包括位置、速度、扭矩等信息。
     * 具体实现由派生类提供。
     */
    tl::expected<UserStatusType, Error> getStatus() {
        if (auto guard = ensureInitialized(); !guard) {
            return tl::make_unexpected(guard.error());
        }
        return derived().getStatusImpl();
    }

    /**
     * @brief 获取电机状态（变体类型）
     * @return 电机状态，成功返回AnyStatus，失败返回Error
     *
     * 获取电机的当前状态，返回一个变体类型。
     * 这个方法重写了IMotor接口中的方法。
     */
    tl::expected<AnyStatus, Error> getStatusVariant() override {
        auto status = getStatus();
        if (!status) {
            return tl::make_unexpected(status.error());
        }
        return AnyStatus{*status};
    }

    /**
     * @brief 获取控制策略引用
     * @return 控制策略引用，成功返回Policy引用，失败返回Error
     *
     * 获取当前电机使用的控制策略的引用。
     */
    tl::expected<std::reference_wrapper<Policy>, Error> getPolicy() {
        if (auto guard = ensureInitialized(); !guard) {
            return tl::make_unexpected(guard.error());
        }
        return std::ref(*m_policy);
    }

    /**
     * @brief 获取控制策略常量引用
     * @return 控制策略常量引用，成功返回Policy常量引用，失败返回Error
     *
     * 获取当前电机使用的控制策略的常量引用。
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
     *
     * 设置新的控制策略，替换当前的控制策略。
     */
    void setPolicy(Policy policy) { m_policy.emplace(std::move(policy)); }

  protected:
    /**
     * @brief 使能电机的实现方法（纯虚函数）
     * @return 操作结果
     *
     * 由派生类实现具体的使能逻辑。
     */
    virtual tl::expected<void, Error> enableImpl() = 0;

    /**
     * @brief 禁用电机的实现方法（纯虚函数）
     * @return 操作结果
     *
     * 由派生类实现具体的禁用逻辑。
     */
    virtual tl::expected<void, Error> disableImpl() = 0;

    /**
     * @brief 获取电机状态的实现方法（纯虚函数）
     * @return 电机状态
     *
     * 由派生类实现具体的状态获取逻辑。
     */
    virtual tl::expected<UserStatusType, Error> getStatusImpl() = 0;

    /**
     * @brief 位置参考值设置后的钩子函数
     * @return 操作结果
     *
     * 当派生类实现HasPosHook概念时，此函数会被调用。
     * 默认实现为空操作。
     */
    virtual tl::expected<void, Error> afterPosRef() { return {}; }

    /**
     * @brief 角速度参考值设置后的钩子函数
     * @return 操作结果
     *
     * 当派生类实现HasAngHook概念时，此函数会被调用。
     * 默认实现为空操作。
     */
    virtual tl::expected<void, Error> afterAngRef() { return {}; }

    /**
     * @brief 扭矩参考值设置后的钩子函数
     * @return 操作结果
     *
     * 当派生类实现HasTorHook概念时，此函数会被调用。
     * 默认实现为空操作。
     */
    virtual tl::expected<void, Error> afterTorRef() { return {}; }

    /**
     * @brief 多个参考值设置后的钩子函数
     * @return 操作结果
     *
     * 当派生类实现HasRefsHook概念时，此函数会被调用。
     * 默认实现为空操作。
     */
    virtual tl::expected<void, Error> afterRefs() { return {}; }

    /**
     * @brief PID参数设置后的钩子函数
     * @param kp 比例参数
     * @param ki 积分参数
     * @param kd 微分参数
     * @return 操作结果
     *
     * 当派生类实现HasPidHook概念时，此函数会被调用。
     * 默认实现为空操作。
     */
    virtual tl::expected<void, Error> afterPidParams(float, float, float) {
        return {};
    }

    /**
     * @brief 重置初始化状态
     *
     * 将电机重置到未初始化状态，清除相关资源。
     */
    void resetInitialization() noexcept {
        m_initialized = false;
        m_driver = nullptr;
        m_policy.reset();
    }

    /**
     * @brief 检查电机是否已初始化
     * @return 如果已初始化返回true，否则返回false
     */
    bool initialized() const noexcept {
        return m_initialized && m_driver && m_policy.has_value();
    }

    /**
     * @brief 获取CAN驱动引用
     * @return CAN驱动引用，成功返回CanDriver引用，失败返回Error
     */
    tl::expected<std::reference_wrapper<Can::CanDriver>, Error> driver() {
        if (auto guard = ensureInitialized(); !guard) {
            return tl::make_unexpected(guard.error());
        }
        return std::ref(*m_driver);
    }

    /**
     * @brief 获取CAN驱动常量引用
     * @return CAN驱动常量引用，成功返回CanDriver常量引用，失败返回Error
     */
    tl::expected<std::reference_wrapper<const Can::CanDriver>, Error> driver()
        const {
        if (auto guard = ensureInitialized(); !guard) {
            return tl::make_unexpected(guard.error());
        }
        return std::cref(*m_driver);
    }

    /**
     * @brief 获取位置参考值
     * @return 位置参考值（弧度）
     *
     * 获取当前存储的位置参考值，以弧度为单位。
     */
    float getPosRef() const {
        return m_pos_ref.load(std::memory_order_acquire);
    }

    /**
     * @brief 获取角速度参考值
     * @return 角速度参考值（弧度/秒）
     *
     * 获取当前存储的角速度参考值，以弧度/秒为单位。
     */
    float getAngRef() const {
        return m_ang_ref.load(std::memory_order_acquire);
    }

    /**
     * @brief 获取扭矩参考值
     * @return 扭矩参考值（牛·米）
     *
     * 获取当前存储的扭矩参考值，以牛·米为单位。
     */
    float getTorRef() const {
        return m_tor_ref.load(std::memory_order_acquire);
    }

    /**
     * @var m_pos_ref
     * @brief 位置参考值 (弧度)
     *
     * 使用原子操作存储位置参考值，确保线程安全。
     */
    std::atomic<float> m_pos_ref{};

    /**
     * @var m_ang_ref
     * @brief 角速度参考值 (弧度/秒)
     *
     * 使用原子操作存储角速度参考值，确保线程安全。
     */
    std::atomic<float> m_ang_ref{};

    /**
     * @var m_tor_ref
     * @brief 扭矩参考值 (牛·米)
     *
     * 使用原子操作存储扭矩参考值，确保线程安全。
     */
    std::atomic<float> m_tor_ref{};

    /**
     * @var m_kp
     * @brief 比例参数
     *
     * 使用原子操作存储比例参数，确保线程安全。
     */
    std::atomic<float> m_kp{};

    /**
     * @var m_ki
     * @brief 积分参数
     *
     * 使用原子操作存储积分参数，确保线程安全。
     */
    std::atomic<float> m_ki{};

    /**
     * @var m_kd
     * @brief 微分参数
     *
     * 使用原子操作存储微分参数，确保线程安全。
     */
    std::atomic<float> m_kd{};

    /**
     * @var m_buffer
     * @brief 状态双缓冲
     *
     * 使用双缓冲机制确保状态读取的线程安全性。
     */
    DoubleBuffer<typename Traits::StatusType> m_buffer{};

    /**
     * @var m_driver
     * @brief CAN驱动指针
     *
     * 指向关联的CAN驱动实例。
     */
    Can::CanDriver *m_driver = nullptr;

    /**
     * @var m_policy
     * @brief 控制策略
     *
     * 可选的控制策略实例，使用std::optional管理。
     */
    std::optional<Policy> m_policy;

    /**
     * @var m_initialized
     * @brief 初始化标志
     *
     * 标记电机是否已完成初始化。
     */
    bool m_initialized = false;

  private:
    /**
     * @brief 确保电机已初始化
     * @return 如果已初始化返回void，否则返回Error
     *
     * 检查电机是否已正确初始化，如果没有则返回错误。
     */
    tl::expected<void, Error> ensureInitialized() const {
        if (initialized())
            return {};
        return tl::make_unexpected(
            Error{ErrorCode::MotorNotInitialized, "Motor not initialized"});
    }

    /**
     * @brief 获取派生类引用
     * @return 派生类引用
     *
     * 使用CRTP模式获取派生类的引用。
     */
    Derived &derived() { return static_cast<Derived &>(*this); }

    /**
     * @brief 获取派生类常量引用
     * @return 派生类常量引用
     *
     * 使用CRTP模式获取派生类的常量引用。
     */
    const Derived &derived() const {
        return static_cast<const Derived &>(*this);
    }
};
} // namespace OneMotor::Motor

#endif // ONE_MOTOR_MOTORBASE_HPP_
