/**
 * @file MotorGuard.hpp
 * @brief 定义了MotorGuard类
 * @author MoonFeather
 * @date 2025-07-06
 */

#ifndef MOTORGUARD_HPP
#define MOTORGUARD_HPP
#include <memory>
#include "OneMotor/Can/CanDriver.hpp"

namespace OneMotor::Motor::DJI
{
    /**
     * @brief 防止电机驱动器在运行结束时输出过高电流的保护类
     *
     * MotorGuard实现了一种看门狗机制，用于监控CAN总线上的电机控制帧。
     * 如果在指定时间内没有检测到ID为0x200的控制帧，它会触发一个熔断操作，
     * 发送预设的CAN帧来将电机电流设置为安全值或零，从而防止电机失控。
     *
     * @par Usage
     * @code
     * // 创建一个驱动器列表
     * std::vector<OneMotor::Motor::DJI::MotorGuard::DriverPair> driver_set;
     *
     * // 添加一个CAN接口，超时后不发送任何特定数据（电机将停止）
     * driver_set.push_back({"can0", std::nullopt});
     *
     * // 添加另一个CAN接口，并指定超时后发送的数据
     * std::array<uint8_t, 16> exit_data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
     *                                      0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};
     * driver_set.push_back({"can1", exit_data});
     *
     * // 获取MotorGuard实例并启动监控
     * OneMotor::Motor::DJI::MotorGuard::getInstance().guard(driver_set);
     *
     * // ... 您的应用程序主循环 ...
     * @endcode
     */
    class MotorGuard
    {
    public:
        /**
         * @brief 定义电机退出时发送的帧数据
         *
         * 这是一个16字节数组的可选类型。如果提供了数据，当看门狗超时时，
         * 这些数据将被分成两个CAN帧（ID 0x200和0x1FF）发送出去。
         */
        using ExitFrameData = std::optional<std::array<uint8_t, 16>>;
        /**
         * @brief 定义驱动器名称和退出帧数据的配对
         */
        using DriverPair = std::pair<std::string, ExitFrameData>;
        /**
         * @brief 获取MotorGuard的单例实例
         *
         * @return MotorGuard& 单例实例的引用
         */
        static MotorGuard& getInstance();
        /**
         * @brief 启动对指定CAN总线的监控
         *
         * @param driver_set 一个包含驱动器名称和可选退出帧数据的向量
         */
        void guard(
            const std::vector<DriverPair>& driver_set);
        MotorGuard(MotorGuard&) = delete;
        MotorGuard& operator=(const MotorGuard&) = delete;

    private:
        /**
         * @brief 存储每个驱动器的看门狗状态
         */
        struct WatchdogState
        {
            std::atomic<std::chrono::steady_clock::time_point> last_fed_time_; ///< 上次喂狗的时间点
            std::atomic<bool> triggered_; ///< 熔断是否已触发
        };

        MotorGuard();
        std::vector<std::shared_ptr<Can::CanDriver>> drivers; ///< 受监控的CAN驱动器列表

        std::unordered_map<std::shared_ptr<Can::CanDriver>, std::unique_ptr<WatchdogState>> watchdog_states_;
        ///< 驱动器到其看门狗状态的映射
        std::unordered_map<std::shared_ptr<Can::CanDriver>, std::array<uint8_t, 16>> driver_exit_data_; ///< 驱动器退出时发送的数据
        std::mutex state_mutex_; ///< 用于保护看门狗状态的互斥锁
        std::chrono::milliseconds check_timeout_{15}; ///< 看门狗检查超时时间
        std::jthread watchdog_monitor_; ///< 监控线程

        /**
         * @brief "喂狗"函数，重置指定驱动器的看门狗计时器
         *
         * @param driver 需要重置看门狗的驱动器
         */
        void feed_watchdog(const std::shared_ptr<Can::CanDriver>& driver);
        /**
         * @brief 看门狗监控线程的执行函数
         *
         * @param stop_token 用于停止线程的停止令牌
         */
        void watchdog_monitor_func_(const std::stop_token& stop_token);
        /**
         * @brief 熔断操作，当看门狗超时时调用
         *
         * @param driver 超时的驱动器
         */
        void circuit_breaker_action_(const std::shared_ptr<Can::CanDriver>& driver);
    };
}

#endif //MOTORGUARD_HPP
