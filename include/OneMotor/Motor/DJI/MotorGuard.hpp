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
        void guard(const std::unordered_set<std::string>& interface_set);
        MotorGuard(MotorGuard&) = delete;
        MotorGuard& operator=(const MotorGuard&) = delete;

    private:
        struct WatchdogState
        {
            std::atomic<std::chrono::steady_clock::time_point> last_fed_time_;
            std::atomic<bool> triggered_;
        };

        MotorGuard();
        std::vector<std::shared_ptr<Can::CanDriver>> drivers;

        std::unordered_map<std::shared_ptr<Can::CanDriver>, std::unique_ptr<WatchdogState>> watchdog_states_;
        std::mutex state_mutex_;
        std::chrono::milliseconds check_timeout_{50};
        std::jthread watchdog_monitor_;

        void feed_watchdog(const std::shared_ptr<Can::CanDriver>& driver);
        void watchdog_monitor_func_(const std::stop_token& stop_token);
        static void circuit_breaker_action(const std::shared_ptr<Can::CanDriver>& driver);
    };
}

#endif //MOTORGUARD_HPP
