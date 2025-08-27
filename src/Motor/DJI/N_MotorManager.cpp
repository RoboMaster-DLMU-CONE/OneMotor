#include "OneMotor/Can/CanFrame.hpp"
#include "OneMotor/Motor/DJI/MotorManager.hpp"
#ifdef ONE_MOTOR_LINUX
#include <format>
#endif
#include <sstream>

using tl::unexpected;
using enum OneMotor::ErrorCode;

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

    tl::expected<void, Error> MotorManager::registerMotor(Can::CanDriver& driver, uint16_t canId) noexcept
    {
        if (auto& set = driver_motor_ids[&driver]; !set.contains(canId))
        {
            if (set.size() >= OM_CAN_MAX_DJI_MOTOR)
            {
#ifdef ONE_MOTOR_LINUX
                return unexpected(Error{
                    DJIMotorManagerError, std::format(
                        "Specified CanDriver has exceeded Max DJI Motor count ({}).",
                        OM_CAN_MAX_DJI_MOTOR)
                });
#else
                std::ostringstream oss;
                oss << "Specified CanDriver has exceeded Max DJI Motor count (" << OM_CAN_MAX_DJI_MOTOR << ").";
                return unexpected(Error{DJIMotorManagerError, oss.str()});
#endif
            }
            set.insert(canId);
            return {};
        }
#ifdef ONE_MOTOR_LINUX
        return unexpected(Error{
            DJIMotorManagerError, std::format("Re-registration detected on CAN ID: {}.", canId)
        });
#else
        {
            std::ostringstream oss;
            oss << "Re-registration detected on CAN ID: " << canId << ".";
            return unexpected(Error{DJIMotorManagerError, oss.str()});
        }
#endif
    }

    // ReSharper disable once CppParameterMayBeConstPtrOrRef
    tl::expected<void, Error> MotorManager::deregisterMotor(Can::CanDriver& driver, const uint16_t canId) noexcept
    {
        if (const auto it = driver_motor_ids.find(&driver); it != driver_motor_ids.end())
        {
            if (it->second.empty())
            {
                return unexpected(Error{DJIMotorManagerError, "Too many times of deregistration."});
            }
            it->second.erase(canId);
            if (it->second.empty())
            {
                driver_motor_ids.erase(it);
            }
            return {};
        }
        return unexpected(Error{DJIMotorManagerError, "Can't find specified CanDriver in MotorManager."});
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
        thread_ = std::make_unique<Thread::Othread>([&]()
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
                    static constexpr uint8_t zero_bytes[8] = {0};
                    if (memcmp(output.data() + 8, zero_bytes, 8) != 0)
                    {
                        frame.id = 0x1FF;
                        std::copy_n(output.data() + 8, 8, frame.data);
                        _ = driver->send(frame);
                    }
                    lock.unlock();
                }

                Thread::sleep_for(std::chrono::milliseconds(1));
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
