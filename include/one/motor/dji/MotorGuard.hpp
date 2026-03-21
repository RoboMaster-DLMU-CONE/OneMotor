/**
 * @file MotorGuard.hpp
 * @brief 定义了 MotorGuard 类
 * @author MoonFeather
 * @date 2025-07-06
 */

#ifndef MOTORGUARD_HPP
#define MOTORGUARD_HPP
#include <memory>
#include <one/can/CanDriver.hpp>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace one::motor::dji {
/**
 * @brief 防止电机驱动器在运行结束时输出过高电流的保护类
 *
 * MotorGuard 实现了一种看门狗机制，用于监控 CAN 总线上的电机控制帧。
 * 如果在指定时间内没有检测到注册的控制帧 ID，它会触发一个熔断操作，
 * 发送预设的 CAN 帧来将电机电流设置为安全值或零，从而防止电机失控。
 *
 * @par Usage
 * @code
 * // 创建一个驱动器列表，每个驱动器指定需要监控的 ID 列表
 * std::vector<OneMotor::Motor::DJI::MotorGuard::DriverPair> driver_set;
 *
 * // 添加一个 CAN 接口，监控 ID 0x200 和 0x201，超时后不发送任何特定数据
 * driver_set.push_back({"can0", {0x200, 0x201}, std::nullopt});
 *
 * // 添加另一个 CAN 接口，监控多个 ID，并指定超时后发送的数据
 * std::array<uint8_t, 16> exit_data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
 * 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};
 * driver_set.push_back({"can1", {0x200, 0x201, 0x202}, exit_data});
 *
 * // 获取 MotorGuard 实例并启动监控
 * OneMotor::Motor::DJI::MotorGuard::getInstance().guard(driver_set);
 *
 * // ... 您的应用程序主循环 ...
 * @endcode
 */
class MotorGuard {
  public:
    /**
     * @brief 定义电机退出时发送的帧数据
     *
     * 这是一个 16 字节数组的可选类型。如果提供了数据，当看门狗超时时，
     * 这些数据将被分成两个 CAN 帧（ID 0x200 和 0x1FF）发送出去。
     */
    using ExitFrameData = std::optional<std::array<uint8_t, 16>>;
    /**
     * @brief 定义驱动器名称、监控 ID 列表和退出帧数据的配对
     */
    using DriverPair = std::tuple<std::string, std::vector<uint16_t>, ExitFrameData>;
    /**
     * @brief 获取 MotorGuard 的单例实例
     *
     * @return MotorGuard& 单例实例的引用
     */
    static MotorGuard &getInstance();
    /**
     * @brief 启动对指定 CAN 总线的监控
     *
     * @param driver_set 一个包含驱动器名称、监控 ID 列表和可选退出帧数据的向量
     */
    void guard(const std::vector<DriverPair> &driver_set);
    MotorGuard(MotorGuard &) = delete;
    MotorGuard &operator=(const MotorGuard &) = delete;

  private:
    /**
     * @brief 存储每个 ID 的看门狗状态
     */
    struct WatchdogState {
        std::atomic<std::chrono::steady_clock::time_point>
            last_fed_time_;           ///< 上次喂狗的时间点
        std::atomic<bool> triggered_; ///< 熔断是否已触发
    };

    MotorGuard();

    /**
     * @brief 存储每个驱动器的退出数据
     */
    struct DriverData {
        std::shared_ptr<can::CanDriver> driver;
        std::array<uint8_t, 16> exit_data;
    };

    std::unordered_map<std::shared_ptr<can::CanDriver>,
                       std::unordered_map<uint16_t, std::unique_ptr<WatchdogState>>>
        driver_id_states_;
    ///< 驱动器 -> (ID -> 看门狗状态) 的嵌套映射

    std::unordered_map<std::shared_ptr<can::CanDriver>, std::array<uint8_t, 16>>
        driver_exit_data_; ///< 驱动器退出时发送的数据

    std::mutex state_mutex_;                ///< 用于保护看门狗状态的互斥锁
    std::chrono::milliseconds check_timeout_{50}; ///< 看门狗检查超时时间
    std::jthread watchdog_monitor_;               ///< 监控线程

    /**
     * @brief "喂狗"函数，重置指定驱动器和 ID 的看门狗计时器
     *
     * @param driver 需要重置看门狗的驱动器
     * @param id 需要重置看门狗的 CAN ID
     */
    void feed_watchdog(const std::shared_ptr<can::CanDriver> &driver, uint16_t id);
    /**
     * @brief 看门狗监控线程的执行函数
     *
     * @param stop_token 用于停止线程的停止令牌
     */
    void watchdog_monitor_func_(const std::stop_token &stop_token);
    /**
     * @brief 熔断操作，当看门狗超时时调用
     *
     * @param driver 超时的驱动器
     */
    void circuit_breaker_action_(const std::shared_ptr<can::CanDriver> &driver);
};
} // namespace one::motor::dji

#endif // MOTORGUARD_HPP
