/**
 * @file M3508.hpp
 * @brief 定义了M3508电机的具体实现，分为速度和位置两种控制模式。
 * @author MoonFeather
 * @date 2025-06-26
 */
#ifndef M3508_HPP
#define M3508_HPP

#include "M3508Base.hpp"

#include "OneMotor/Can/CanDriver.hpp"
#include "OneMotor/Control/PID.hpp"
#include <atomic>


namespace OneMotor::Motor::DJI
{
    /**
     * @enum MotorMode
     * @brief 定义了M3508电机的控制模式。
     */
    enum class MotorMode
    {
        Position, ///< 位置闭环控制模式
        Angular, ///< 速度闭环控制模式
    };

    /**
     * @brief M3508电机类的前向声明。
     * @tparam id 电机ID (1-8)。
     * @tparam mode 电机控制模式 (Position或Angular)。
     */
    template <uint8_t id, MotorMode mode>
    class M3508;

    /**
     * @class M3508<id, MotorMode::Angular>
     * @brief M3508电机在速度控制模式下的具体实现。
     * @details
     * 这是一个最终类，继承自 `M3508Base`。
     * 它实现了一个单闭环PID控制器，直接控制电机的角速度。
     */
    template <uint8_t id>
    class M3508<id, MotorMode::Angular>final : public M3508Base<id>
    {
    public:
        /**
         * @brief 速度模式的构造函数。
         * @param driver CAN总线驱动实例。
         * @param ang_params 速度环的PID参数。
         */
        explicit M3508(Can::CanDriver& driver, const Control::PID_Params<float>& ang_params);

        /**
         * @brief 设置目标角速度。
         * @param ref 目标角速度 (单位：度/秒)。
         * @note 此操作是线程安全的。
         */
        void setRef(float ref) noexcept;

        /**
         * @brief 在运行时修改速度环的PID参数。
         * @param func 一个函数，其参数为PID控制器的指针，可在函数体内修改参数。
         * @note 此操作是线程安全的。
         */
        void editAngPID(const std::function<void(PIDController*)>& func);

    private:
        /// @brief 在禁用状态下处理CAN帧，仅更新状态。
        void disabled_func_(Can::CanFrame&& frame) override;
        /// @brief 在启用状态下处理CAN帧，执行PID计算并推送输出电流。
        void enabled_func_(Can::CanFrame&& frame) override;
        std::unique_ptr<PIDController> ang_pid_; ///< 速度环PID控制器
        std::atomic<float> ang_ref_; ///< 目标角速度
    };

    /**
     * @class M3508<id, MotorMode::Position>
     * @brief M3508电机在位置控制模式下的具体实现。
     * @details
     * 这是一个最终类，继承自 `M3508Base`。
     * 它实现了一个串级PID控制器：一个位置环PID和一个速度环PID。
     * 位置环的输出作为速度环的输入。
     */
    template <uint8_t id>
    class M3508<id, MotorMode::Position>final : public M3508Base<id>
    {
    public:
        /**
         * @brief 位置模式的构造函数。
         * @param driver CAN总线驱动实例。
         * @param pos_params 位置环的PID参数。
         * @param ang_params 速度环的PID参数。
         */
        explicit M3508(Can::CanDriver& driver,
                       const Control::PID_Params<float>& pos_params,
                       const Control::PID_Params<float>& ang_params);
        /**
         * @brief 设置速度环的最大输出，即位置环PID的输出限幅。
         * @param ang_ref 速度上限值 (单位：度/秒)。
         * @note 此操作是线程安全的。
         */
        void setAngRef(float ang_ref) noexcept;

        /**
         * @brief 设置目标位置（角度）。
         * @param pos_ref 目标总角度 (单位：度)。
         * @note 此操作是线程安全的。
         */
        void setPosRef(float pos_ref) noexcept;

        /**
         * @brief 在运行时修改位置环的PID参数。
         * @param func 一个函数，其参数为PID控制器的指针，可在函数体内修改参数。
         * @note 此操作是线程安全的。
         */
        void editPosPID(const std::function<void(PIDController*)>& func);

        /**
         * @brief 在运行时修改速度环的PID参数。
         * @param func 一个函数，其参数为PID控制器的指针，可在函数体内修改参数。
         * @note 此操作是线程安全的。
         */
        void editAngPID(const std::function<void(PIDController*)>& func);

    private:
        /// @brief 在禁用状态下处理CAN帧，仅更新状态。
        void disabled_func_(Can::CanFrame&& frame) override;
        /// @brief 在启用状态下处理CAN帧，执行串级PID计算并推送输出电流。
        void enabled_func_(Can::CanFrame&& frame) override;
        std::unique_ptr<PIDController> pos_pid_; ///< 位置环PID控制器
        std::unique_ptr<PIDController> ang_pid_; ///< 速度环PID控制器
        std::atomic<float> pos_ref_; ///< 目标位置（总角度）
        std::atomic<float> ang_ref_; ///< 速度限制（位置环输出限幅）
    };
}

#endif //M3508_HPP
