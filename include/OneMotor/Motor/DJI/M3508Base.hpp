/**
 * @file M3508Base.hpp
 * @brief 定义了M3508电机的抽象基类。
 * @author MoonFeather
 * @date 2025-06-26
 */
#ifndef M3508BASE_HPP
#define M3508BASE_HPP
#include <expected>
#include "M3508Frames.hpp"
#include "OneMotor/Control/PID.hpp"
#include "OneMotor/Util/SpinLock.hpp"

namespace OneMotor::Motor::DJI
{
    /**
     * @brief PID控制器的类型别名，方便在M3508类中使用。
     * @details 预设了位置式PID，并启用了死区、限幅等常用特性。
     */
    using PIDController = Control::PIDController<
        Control::Positional, float, Control::WithDeadband, Control::WithIntegralLimit, Control::WithOutputLimit,
        Control::WithOutputFilter, Control::WithDerivativeFilter>;

    /**
     * @class M3508Base
     * @brief M3508电机的抽象基类，提供通用接口和基础功能。
     * @tparam id 电机的ID (1-8)。这决定了它在CAN总线上的地址以及在MotorManager中的位置。
     * @details
     * 此基类处理与 `MotorManager` 的交互（注册/注销），
     * 并定义了启用(enable)、禁用(disable)、关闭(shutdown)电机和获取状态的公共接口。
     * 具体的控制逻辑（例如，在启用状态下如何处理CAN反馈）由派生类通过重写纯虚函数 `enabled_func_` 和 `disabled_func_` 来实现。
     */
    template <uint8_t id>
    class M3508Base
    {
        using Result = std::expected<void, std::string>;

    public:
        /**
         * @brief 虚析构函数。
         * @details 确保在对象销毁时，能正确地从MotorManager中注销电机。
         */
        virtual ~M3508Base();

        /**
         * @brief 获取电机当前的状态。
         * @return M3508Status 结构体的一个拷贝，包含了角度、速度、电流等信息。
         * @note 此操作是线程安全的。
         */
        M3508Status getStatus() noexcept;

        /**
         * @brief 禁用电机。
         * @details 电机将停止主动控制，但仍会接收和更新状态。控制器输出将被忽略。
         * @return Result 操作结果。
         */
        Result disable() noexcept;

        /**
         * @brief 启用电机。
         * @details 电机将根据设定的模式和目标值开始主动控制。
         * @return Result 操作结果。
         */
        Result enable() noexcept;

        /**
         * @brief 关闭电机。
         * @details 电机将完全停止响应，不进行任何处理。
         * @return Result 操作结果。
         */
        Result shutdown() noexcept;

    protected:
        /**
         * @brief 基类构造函数。
         * @param driver 电机连接的CAN总线驱动实例。
         * @details 在构造时会自动向MotorManager注册该电机。
         */
        explicit M3508Base(Can::CanDriver& driver);

        /**
         * @brief 纯虚函数，由派生类实现在“禁用”状态下的CAN帧处理逻辑。
         * @param frame 接收到的CAN帧。
         */
        virtual void disabled_func_(Can::CanFrame&& frame) = 0;

        /**
         * @brief 纯虚函数，由派生类实现在“启用”状态下的CAN帧处理逻辑。
         * @param frame 接收到的CAN帧。
         */
        virtual void enabled_func_(Can::CanFrame&& frame) = 0;

        /**
         * @brief “关闭”状态下的回调函数，不做任何事情。
         * @param frame 接收到的CAN帧 (未使用)。
         */
        static void shutdown_func_([[maybe_unused]] Can::CanFrame&& frame)
        {
        };

        Can::CanDriver& driver_; ///< CAN总线驱动的引用
        Util::SpinLock status_lock_; ///< 用于保护状态访问的自旋锁
        M3508Status status_; ///< 电机的状态信息
        static constexpr uint16_t canId_ = id + 0x200; ///< 电机的CAN ID (标准ID范围 0x201-0x208)
    };
}

#endif //M3508BASE_HPP
