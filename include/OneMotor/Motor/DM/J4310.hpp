/**
 * @file J4310.hpp
 * @author MF (you@domain.com)
 * @brief DM J4310 电机驱动程序
 * @version 0.1
 * @date 2025-06-27
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef J4310_HPP
#define J4310_HPP
#include "J4310Frame.hpp"
#include "OneMotor/Can/CanDriver.hpp"
#include "OneMotor/Util/SpinLock.hpp"

namespace OneMotor::Motor::DM
{
    /**
     * @brief DM J4310 电机驱动类
     */
    class J4310
    {
        using Result = std::expected<void, std::string>;

    public:
        J4310() = delete;
        /**
         * @brief 构造一个新的 J4310 对象
         * @param driver CAN 驱动程序引用
         * @param canId 电机 CAN ID
         * @param masterId 用于接收消息的主机 CAN ID
         */
        explicit J4310(Can::CanDriver& driver, uint16_t canId, uint16_t masterId);
        /**
         * @brief 启用电机
         * @return 操作结果
         */
        Result enable();
        /**
         * @brief 禁用电机
         * @return 操作结果
         */
        Result disable();
        /**
         * @brief 将当前位置设置为零
         * @return 操作结果
         */
        Result setZeroPosition();
        /**
         * @brief 清除电机错误
         * @return 操作结果
         */
        Result cleanError();
        /**
         * @brief MIT 控制模式
         * @param position 目标位置 (rad)
         * @param velocity 目标速度 (rad/s)
         * @param torque 目标扭矩 (N*m)
         * @param kp 位置增益
         * @param kd 速度增益
         * @return 操作结果
         */
        Result MITControl(float position, float velocity, float torque, float kp, float kd);
        /**
         * @brief 位置和速度控制模式
         * @param position 目标位置 (rad)
         * @param velocity 目标速度 (rad/s)
         * @return 操作结果
         */
        Result posVelControl(float position, float velocity);
        /**
         * @brief 速度控制模式
         * @param velocity 目标速度 (rad/s)
         * @return 操作结果
         */
        Result velControl(float velocity);
        /**
         * @brief 获取电机状态
         * @return 一个包含状态或错误字符串的 std::expected 对象
         */
        std::expected<J4310Status, std::string> getStatus();

    private:
        J4310Status status_{};
        Util::SpinLock lock_;
        Can::CanDriver& driver_;
        uint16_t canId_;
        uint16_t masterId_;
    };
}

#endif //J4310_HPP
