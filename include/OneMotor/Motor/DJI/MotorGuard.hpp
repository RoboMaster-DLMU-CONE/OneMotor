#ifndef MOTORGUARD_HPP
#define MOTORGUARD_HPP
#include <condition_variable>
#include <memory>

#include "OneMotor/Can/CanDriver.hpp"

#include <unordered_set>

namespace OneMotor::Motor::DJI
{
    class MotorGuard
    {
    public:
        static MotorGuard& getInstance();
        [[noreturn]] void guard(const std::unordered_set<std::string>& interface_set);
        MotorGuard(MotorGuard&) = delete;
        MotorGuard& operator=(const MotorGuard&) = delete;
        ~MotorGuard();

    private:
        struct WatchdogState
        {
            std::atomic<bool> is_fed_{false};
            std::atomic<std::chrono::steady_clock::time_point> last_fed_time_;
        };

        MotorGuard();
        std::vector<std::shared_ptr<Can::CanDriver>> drivers;
        std::vector<std::jthread> guard_tasks_;

        std::unordered_map<std::shared_ptr<Can::CanDriver>, std::unique_ptr<WatchdogState>> watchdog_states_;
        std::atomic<bool> should_stop_{false};
        std::condition_variable watchdog_cv_;
        std::mutex watchdog_mutex_;
        std::chrono::milliseconds timeout_{5};
        std::jthread watchdog_monitor_;

        void guard_task_func_();
        void feed_watchdog(const std::shared_ptr<Can::CanDriver>& driver);
        void watchdog_monitor_func_();
        void circuit_breaker_action(const std::shared_ptr<Can::CanDriver>& driver);
    };
}

#endif //MOTORGUARD_HPP
