#include <one/motor/dji/MotorGuard.hpp>
#include <one/utils/Panic.hpp>

namespace one::motor::dji {
void MotorGuard::guard(const std::vector<DriverPair> &driver_set) {
    for (const auto &[name, ids, data] : driver_set) {
        auto driver = std::make_shared<can::CanDriver>(name);

        // 初始化该驱动器的所有 ID 状态
        auto &id_states = driver_id_states_[driver];
        for (const auto &id : ids) {
            id_states[id] = std::make_unique<WatchdogState>();
            id_states[id]->last_fed_time_ = std::chrono::steady_clock::now();
            id_states[id]->triggered_.store(false, std::memory_order_relaxed);
        }

        // 设置退出数据
        std::array<uint8_t, 16> temp{};
        if (data.has_value()) {
            temp = data.value();
        }
        driver_exit_data_[driver] = temp;

        // 为每个 ID 注册回调
        for (const auto &id : ids) {
            (void)driver->registerCallback(
                {id}, [this, driver, id](can::CanFrame) {
                    this->feed_watchdog(driver, id);
                });
        }

        // 打开 CAN 驱动
        (void)driver->open().or_else(
            [](const auto &) { panic("Failed to open CAN driver."); });
    }

    // 启动监控线程
    watchdog_monitor_ = std::jthread([this](const std::stop_token &stop_token) {
        this->watchdog_monitor_func_(stop_token);
    });
}

void MotorGuard::feed_watchdog(const std::shared_ptr<can::CanDriver> &driver,
                               uint16_t id) {
    std::lock_guard lock(state_mutex_);
    if (const auto driver_it = driver_id_states_.find(driver);
        driver_it != driver_id_states_.end()) {
        const auto &id_states = driver_it->second;
        if (const auto id_it = id_states.find(id); id_it != id_states.end()) {
            id_it->second->last_fed_time_ = std::chrono::steady_clock::now();
            id_it->second->triggered_.store(false, std::memory_order_release);
        }
    }
}

void MotorGuard::watchdog_monitor_func_(const std::stop_token &stop_token) {
    while (!stop_token.stop_requested()) {
        std::this_thread::sleep_for(check_timeout_);
        if (stop_token.stop_requested())
            break;

        auto now = std::chrono::steady_clock::now();
        std::unordered_map<std::shared_ptr<can::CanDriver>, bool>
            driver_trigger_map;
        {
            std::lock_guard lock(state_mutex_);
            for (auto &[driver, id_states] : driver_id_states_) {
                bool any_triggered = false;
                for (auto &[id, state] : id_states) {
                    if (state->triggered_.load(std::memory_order_acquire))
                        continue;
                    auto time_since_fed = now - state->last_fed_time_.load(
                                                   std::memory_order_acquire);
                    if (time_since_fed > std::chrono::milliseconds(50)) {
                        state->triggered_.store(true,
                                                std::memory_order_release);
                        any_triggered = true;
                    }
                }
                driver_trigger_map[driver] = any_triggered;
            }
        }

        // 在锁外执行熔断操作
        for (const auto &[driver, should_trigger] : driver_trigger_map) {
            if (should_trigger) {
                circuit_breaker_action_(driver);
            }
        }
    }
}

void MotorGuard::circuit_breaker_action_(
    const std::shared_ptr<can::CanDriver> &driver) {
    can::CanFrame frame1;
    frame1.dlc = 8;
    frame1.id = 0x200;
    can::CanFrame frame2;
    frame2.dlc = 8;
    frame2.id = 0x1FF;
    std::copy_n(driver_exit_data_[driver].data(), 8, frame1.data);
    std::copy_n(driver_exit_data_[driver].data() + 8, 8, frame2.data);
    (void)driver->send(frame1);
    (void)driver->send(frame2);
}

MotorGuard &MotorGuard::getInstance() {
    static MotorGuard _instance;
    return _instance;
}

MotorGuard::MotorGuard() = default;
} // namespace one::motor::dji
