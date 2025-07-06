#include "OneMotor/Motor/DJI/MotorGuard.hpp"
#include "OneMotor/Util/Panic.hpp"

namespace OneMotor::Motor::DJI
{
    void MotorGuard::guard(const std::unordered_set<std::string>& interface_set)
    {
        for (auto& interface : interface_set)
        {
            auto driver = std::make_shared<Can::CanDriver>(interface);
            drivers.push_back(driver);
            watchdog_states_[driver] = std::make_unique<WatchdogState>();
            watchdog_states_[driver]->last_fed_time_ = std::chrono::steady_clock::now();
            watchdog_states_[driver]->triggered_.store(false, std::memory_order_relaxed);
        }

        for (const auto& driver : drivers)
        {
            if (auto result = driver->open(); !result)
            {
                Util::om_panic(result.error().data());
            }
        }

        for (const auto& driver : drivers)
        {
            auto result = driver->registerCallback({0x200}, [this, driver](Can::CanFrame&&)
            {
                this->feed_watchdog(driver);
            });
        }
        watchdog_monitor_ = std::jthread([this](const std::stop_token& stop_token)
        {
            this->watchdog_monitor_func_(stop_token);
        });
    }

    void MotorGuard::feed_watchdog(const std::shared_ptr<Can::CanDriver>& driver)
    {
        std::lock_guard lock(state_mutex_);
        if (const auto it = watchdog_states_.find(driver); it != watchdog_states_.end())
        {
            it->second->last_fed_time_ = std::chrono::steady_clock::now();
            it->second->triggered_.store(false, std::memory_order_release);
        }
    }

    void MotorGuard::watchdog_monitor_func_(const std::stop_token& stop_token)
    {
        while (!stop_token.stop_requested())
        {
            std::this_thread::sleep_for(check_timeout_);
            if (stop_token.stop_requested()) break;

            auto now = std::chrono::steady_clock::now();
            bool all_triggered = true;
            std::lock_guard lock(state_mutex_);

            for (auto& [driver, state] : watchdog_states_)
            {
                if (state->triggered_.load(std::memory_order_acquire)) continue;
                auto time_since_fed = now - state->last_fed_time_.load(std::memory_order_acquire);
                if (time_since_fed > check_timeout_)
                {
                    circuit_breaker_action(driver);
                    state->triggered_.store(true);
                }
                else
                {
                    all_triggered = false;
                }
            }
            if (all_triggered)
            {
                std::exit(0);
            }
        }
    }

    void MotorGuard::circuit_breaker_action(const std::shared_ptr<Can::CanDriver>& driver)
    {
        Can::CanFrame frame{};
        frame.dlc = 8;
        frame.id = 0x200;
        auto _ = driver->send(frame);
        frame.id = 0x1FF;
        _ = driver->send(frame);
    }


    MotorGuard& MotorGuard::getInstance()
    {
        static MotorGuard _instance;
        return _instance;
    }

    MotorGuard::MotorGuard() = default;
}
