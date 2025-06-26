#include "OneMotor/Can/CanFrame.hpp"
#include "OneMotor/Motor/DJI/MotorManager.hpp"

using Result = std::expected<void, std::string>;

namespace OneMotor::Motor::DJI
{
    MotorManager::~MotorManager()
    {
        stop_.store(true, std::memory_order_release);
        if (thread_->joinable()) { thread_->join(); }
    }

    MotorManager& MotorManager::getInstance()
    {
        static MotorManager instance;
        return instance;
    }

    Result MotorManager::registerMotor(Can::CanDriver& driver, uint16_t canId) noexcept
    {
        if (auto& set = driver_motor_ids[&driver]; !set.contains(canId))
        {
            if (set.size() >= OM_CAN_MAX_DJI_MOTOR)
            {
                return std::unexpected(std::format("Specified CanDriver has exceeded Max DJI Motor count ({}).",
                                                   OM_CAN_MAX_DJI_MOTOR));
            }
            set.insert(canId);
            return {};
        }
        return std::unexpected(std::format("Re-registration detected on CAN ID: {}.",
                                           canId));
    }

    // ReSharper disable once CppParameterMayBeConstPtrOrRef
    Result MotorManager::deregisterMotor(Can::CanDriver& driver, const uint16_t canId) noexcept
    {
        if (const auto it = driver_motor_ids.find(&driver); it != driver_motor_ids.end())
        {
            if (it->second.empty())
            {
                return std::unexpected("Too many times of deregistration.");
            }
            it->second.erase(canId);
            if (it->second.empty())
            {
                driver_motor_ids.erase(it);
            }
            return {};
        }
        return std::unexpected("Can't find specified CanDriver in MotorManager.");
    }

    template <uint8_t id>
    void MotorManager::pushOutput(Can::CanDriver& driver, const uint8_t lo_value, const uint8_t hi_value) noexcept
    {
        constexpr uint8_t base_index = (id - 1) * 2;
        auto& lock = driver_motor_outputs[&driver].second;
        lock.lock();
        auto& output_buffer = driver_motor_outputs[&driver].first;

        output_buffer[base_index] = hi_value;
        output_buffer[base_index + 1] = lo_value;
        lock.unlock();
    }

    MotorManager::MotorManager()
    {
        thread_ = std::make_unique<thread::Othread>([&]()
        {
            while (!stop_.load(std::memory_order_acquire))
            {
                for (auto& [driver, pair] : driver_motor_outputs)
                {
                    auto& lock = pair.second;
                    auto& output = pair.first;
                    Can::CanFrame frame;
                    frame.dlc = 8;
                    frame.id = 0x200;
                    lock.lock();
                    std::copy_n(output.data(), 8, frame.data);
                    [[maybe_unused]] auto _ = driver->send(frame);
                    frame.id = 0x1FF;
                    std::copy_n(output.data() + 8, 8, frame.data);
                    lock.unlock();
                    _ = driver->send(frame);
                }
                thread::Othread::sleep_for(300000);
            }
        });
    }

    template void MotorManager::pushOutput<1>(Can::CanDriver& driver, uint8_t lo_value, uint8_t hi_value) noexcept;
    template void MotorManager::pushOutput<2>(Can::CanDriver& driver, uint8_t lo_value, uint8_t hi_value) noexcept;
    template void MotorManager::pushOutput<3>(Can::CanDriver& driver, uint8_t lo_value, uint8_t hi_value) noexcept;
    template void MotorManager::pushOutput<4>(Can::CanDriver& driver, uint8_t lo_value, uint8_t hi_value) noexcept;
    template void MotorManager::pushOutput<5>(Can::CanDriver& driver, uint8_t lo_value, uint8_t hi_value) noexcept;
    template void MotorManager::pushOutput<6>(Can::CanDriver& driver, uint8_t lo_value, uint8_t hi_value) noexcept;
    template void MotorManager::pushOutput<7>(Can::CanDriver& driver, uint8_t lo_value, uint8_t hi_value) noexcept;
    template void MotorManager::pushOutput<8>(Can::CanDriver& driver, uint8_t lo_value, uint8_t hi_value) noexcept;
}
