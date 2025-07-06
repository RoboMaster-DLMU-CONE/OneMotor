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
            auto result = driver->registerCallback({0x200, 0x1FF}, [this, driver](Can::CanFrame&&)
            {
                this->feed_watchdog(driver);
            });
        }
        watchdog_monitor_ = std::jthread([this](const std::stop_token& stop_token)
        {
            this->watchdog_monitor_func_();
        });
        for (auto& driver : drivers)
        {
            guard_tasks_.emplace_back([this](const std::stop_token& stop_token)
            {
                this->guard_task_func_();
            });
        }
    }

    void MotorGuard::feed_watchdog(const std::shared_ptr<Can::CanDriver>& driver)
    {
        if (const auto it = watchdog_states_.find(driver); it != watchdog_states_.end())
        {
            it->second->is_fed_ = true;
            it->second->last_fed_time_ = std::chrono::steady_clock::now();
            std::lock_guard lock(watchdog_mutex_);
            watchdog_cv_.notify_one();
        }
    }

    void MotorGuard::watchdog_monitor_func_()
    {
        while (!should_stop_.load(std::memory_order_acquire))
        {
            std::unique_lock lock(watchdog_mutex_);
            watchdog_cv_.wait_for(lock, std::chrono::milliseconds(30));
            if (should_stop_) break;
            lock.unlock();

            auto now = std::chrono::steady_clock::now();
            for (auto& [driver, state] : watchdog_states_)
            {
                auto time_since_fed = now - state->last_fed_time_.load(std::memory_order_acquire);
                if (time_since_fed > timeout_)
                {
                    circuit_breaker_action(driver);
                }
                else if (state->is_fed_.load())
                {
                    state->is_fed_ = false;
                }
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
