#include "one-motor/motor/DJI/M3508.hpp"
#include <string>

#include "one-motor/motor/DJI/MotorManager.hpp"
#include "one-motor/util/Panic.hpp"

namespace OneMotor::Motor::DJI
{
    M3508::M3508(Can::CanDriver& driver, const uint8_t id): driver_(driver)
    {
        if (id > 8 || id < 1)
        {
            Util::om_panic("M3508 Only support 1 <= id <= 8.");
        }
        canId_ = id + 0x200;
        MotorManager& manager = MotorManager::getInstance();
        if (auto result = manager.registerMotor(driver_, canId_); !result)
        {
            Util::om_panic(std::move(result.error()));
        }
    }

    M3508::~M3508()
    {
        MotorManager& manager = MotorManager::getInstance();
        if (auto result = manager.deregisterMotor(driver_, canId_); !result)
        {
            Util::om_panic(std::move(result.error()));
        }
    }
}
