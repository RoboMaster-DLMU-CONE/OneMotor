#include "one-motor/motor/DJI/MotorManager.hpp"

namespace OneMotor::Motor::DJI
{
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
            return {};
        }
        return std::unexpected("Can't find specified CanDriver in MotorManager.");
    }

    MotorManager::MotorManager()
    = default;
}
