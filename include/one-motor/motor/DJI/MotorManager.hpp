#ifndef MOTORMANAGER_HPP
#define MOTORMANAGER_HPP

#include <unordered_map>
#include <one-motor/can/CanDriver.hpp>

#include "one-motor/thread/Othread.hpp"
#include "one-motor/util/SpinLock.hpp"

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
        ~MotorManager();
        static MotorManager& getInstance();
        Result registerMotor(Can::CanDriver& driver, uint16_t canId) noexcept;
        Result deregisterMotor(Can::CanDriver& driver, uint16_t canId) noexcept;
        template <uint8_t id>
        void pushOutput(Can::CanDriver& driver, uint8_t lo_value, uint8_t hi_value) noexcept;

    private:
        using OutputArray = std::array<uint8_t, 16>;
        using OutputPair = std::pair<OutputArray, Util::SpinLock>;
        MotorManager();
        std::unordered_map<Can::CanDriver*, std::set<uint16_t>> driver_motor_ids;
        std::unordered_map<Can::CanDriver*, OutputPair> driver_motor_outputs;
        std::atomic<bool> stop_{false};
        std::unique_ptr<thread::Othread> thread_;
    };
}

#endif //MOTORMANAGER_HPP
