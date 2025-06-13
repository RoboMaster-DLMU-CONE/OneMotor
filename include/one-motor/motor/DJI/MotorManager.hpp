#ifndef MOTORMANAGER_HPP
#define MOTORMANAGER_HPP

#include <unordered_map>
#include <one-motor/can/CanDriver.hpp>

namespace OneMotor::Motor::DJI
{
    class MotorManager
    {
    public:
        using Result = std::expected<void, std::string>;

        MotorManager(MotorManager&) = delete;
        MotorManager(MotorManager&&) = delete;
        MotorManager& operator=(MotorManager&) = delete;
        MotorManager& operator=(MotorManager&&) = delete;
        static MotorManager& getInstance();
        Result registerMotor(Can::CanDriver& driver, uint16_t canId) noexcept;
        Result deregisterMotor(Can::CanDriver& driver, uint16_t canId) noexcept;

    private:
        MotorManager();
        std::unordered_map<Can::CanDriver*, std::set<uint16_t>> driver_motor_ids;
    };
}

#endif //MOTORMANAGER_HPP
