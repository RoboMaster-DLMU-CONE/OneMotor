#ifndef CANFRAME_HPP
#define CANFRAME_HPP
#include <cstdint>
#define ONE_MOTOR_CAN_MAX_DLEN 8

#ifdef ONE_MOTOR_LINUX
#include <linux/can.h>
#endif


namespace OneMotor::Can
{
    struct CanFrame
    {
#ifdef ONE_MOTOR_LINUX
        explicit operator can_frame() const;
        explicit CanFrame(const can_frame&);
#endif
        CanFrame() = default;
        uint32_t id{};
        uint8_t dlc{};
        uint8_t flags{};
        uint8_t data[ONE_MOTOR_CAN_MAX_DLEN]{};
    };
}

#endif //CANFRAME_HPP
