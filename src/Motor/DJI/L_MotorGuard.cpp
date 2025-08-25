#include "OneMotor/Motor/DJI/MotorGuard.hpp"
#include "OneMotor/Util/Panic.hpp"

namespace OneMotor::Motor::DJI
{
    void MotorGuard::guard(const std::vector<DriverPair>& driver_set)
    {
        for (auto& [name, data] : driver_set)
        {
            auto driver = std::make_shared<Can::CanDriver>(name);
            drivers.push_back(driver);
            watchdog_states_[driver] = std::make_unique<WatchdogState>();
            watchdog_states_[driver]->last_fed_time_ = std::chrono::steady_clock::now();
            watchdog_states_[driver]->triggered_.store(false, std::memory_order_relaxed);
            std::array<uint8_t, 16> temp{};
            if (data.has_value()) temp = data.value();
            driver_exit_data_[driver] = temp;
        }

        for (const auto& driver : drivers)
        {
            (void)driver->open().or_else([](const auto& e)
            {
                panic(std::move(e.message));
            });
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
            {
                std::lock_guard lock(state_mutex_);
                for (auto& [driver, state] : watchdog_states_)
                {
                    if (state->triggered_.load(std::memory_order_acquire)) continue;
                    auto time_since_fed = now - state->last_fed_time_.load(std::memory_order_acquire);
                    if (time_since_fed > std::chrono::milliseconds(10))
                    {
                        circuit_breaker_action_(driver);
                        state->triggered_.store(true);
                    }
                }
            }
        }
    }

    void MotorGuard::circuit_breaker_action_(const std::shared_ptr<Can::CanDriver>& driver)
    {
        Can::CanFrame frame1;
        frame1.dlc = 8;
        frame1.id = 0x200;
        Can::CanFrame frame2;
        frame2.dlc = 8;
        frame2.id = 0x1FF;
        std::copy_n(driver_exit_data_[driver].data(), 8, frame1.data);
        std::copy_n(driver_exit_data_[driver].data() + 8, 8, frame2.data);

        for (int i = 0; i < 3; ++i)
        {
            (void)driver->send(frame1);
            (void)driver->send(frame2);
        }
    }


    MotorGuard& MotorGuard::getInstance()
    {
        static MotorGuard _instance;
        return _instance;
    }

    MotorGuard::MotorGuard() = default;
}
