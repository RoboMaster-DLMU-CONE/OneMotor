#include "one-motor/motor/DJI/M3508.hpp"
#include <string>

#include "one-motor/motor/DJI/MotorManager.hpp"
#include "one-motor/util/Panic.hpp"

namespace OneMotor::Motor::DJI
{
    template <uint8_t id>
    M3508<id>::M3508(Can::CanDriver& driver): driver_(driver)
    {
        static_assert(id >= 1 && id <= 8, "M3508 Only support 1 <= id <= 8.");
        MotorManager& manager = MotorManager::getInstance();
        if (auto result = manager.registerMotor(driver_, canId_); !result)
        {
            Util::om_panic(std::move(result.error()));
        }
        manager.pushOutput<id>(driver_, 0, 0);
    }

    template <uint8_t id>
    M3508<id>::~M3508()
    {
        MotorManager& manager = MotorManager::getInstance();
        if (auto result = manager.deregisterMotor(driver_, canId_); !result)
        {
            Util::om_panic(std::move(result.error()));
        }
    }

    template class M3508<1>;
    template class M3508<2>;
    template class M3508<3>;
    template class M3508<4>;
    template class M3508<5>;
    template class M3508<6>;
    template class M3508<7>;
    template class M3508<8>;
}
