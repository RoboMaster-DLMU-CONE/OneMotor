#ifndef M3508_HPP
#define M3508_HPP
#include "M3508Frames.hpp"
#include "one-motor/can/CanDriver.hpp"

namespace OneMotor::Motor::DJI
{
    class M3508
    {
    public:
        explicit M3508(Can::CanDriver& driver, uint8_t id);
        ~M3508();

    private:
        Can::CanDriver& driver_;
        uint16_t canId_{};
    };
}

#endif //M3508_HPP
