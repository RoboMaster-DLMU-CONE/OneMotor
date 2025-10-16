/**
 * @file J4310.hpp
 * @author MoonFeather
 * @brief DM J4310 电机驱动程序
 * @date 2025-06-27
 *
 *
 */
#ifndef J4310_HPP
#define J4310_HPP
#include "J4310Frame.hpp"
#include "OneMotor/Can/CanDriver.hpp"
#include "OneMotor/Util/DoubleBuffer.hpp"

namespace OneMotor::Motor::DM
{
    /**
     * @brief DM J4310 电机驱动类
     */
    class J4310
    {
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
        tl::expected<void, Error> enable();
        /**
         * @brief 禁用电机
         * @return 操作结果
         */
        tl::expected<void, Error> disable();
        /**
         * @brief 将当前位置设置为零
         * @return 操作结果
         */
        tl::expected<void, Error> setZeroPosition();
        /**
         * @brief 清除电机错误
         * @return 操作结果
         */
        tl::expected<void, Error> cleanError();
        /**
         * @brief MIT 控制模式
         * @param position 目标位置 (rad)
         * @param velocity 目标速度 (rad/s)
         * @param torque 目标扭矩 (N*m)
         * @param kp 位置增益
         * @param kd 速度增益
         * @return 操作结果
         */
        tl::expected<void, Error> MITControl(float position, float velocity, float torque, float kp, float kd);
        /**
         * @brief 位置和速度控制模式
         * @param position 目标位置 (rad)
         * @param velocity 目标速度 (rad/s)
         * @return 操作结果
         */
        tl::expected<void, Error> posVelControl(float position, float velocity);
        /**
         * @brief 速度控制模式
         * @param velocity 目标速度 (rad/s)
         * @return 操作结果
         */
        tl::expected<void, Error> velControl(float velocity);

        /**
         *
         * @brief 刷新电机状态，仅支持V13以上固件
         * @return 操作结果
         */
        tl::expected<void, Error> refreshStatus() noexcept;

        /**
         * @brief 获取电机上一次接收到到的反馈帧状态
         * 对电机进行各种控制操作均会获得反馈帧
         * @note 若想获取当前最新状态，需配合 @ref refreshStatus "refreshStatus" 函数，或直接使用 @ref getNewStatus "getNewStatus"函数
         * @return @ref J4310Status "J4310Status"
         */
        J4310Status getStatus();

        /**
         * @brief 获取电机最新状态
         * @param usec 指定向电机请求获取反馈帧后的延时时间。延时过低可能来不及接收最新状态；延时过高可能导致主线程阻塞
         * @return 包含电机状态或错误信息的expected结构体
         */
        tl::expected<J4310Status, Error> getNewStatus(uint16_t usec = 500);

    private:
        DoubleBuffer<J4310Status> m_Buffer{};
        Can::CanDriver& driver_;
        uint16_t canId_;
        uint16_t masterId_;
    };
}

#endif //J4310_HPP
