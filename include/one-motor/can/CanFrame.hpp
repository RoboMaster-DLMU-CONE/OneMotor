#ifndef CANFRAME_HPP
#define CANFRAME_HPP
#include <cstddef>
#include <cstdint>
#include <type_traits>
#define ONE_MOTOR_CAN_MAX_DLEN 8

#ifdef ONE_MOTOR_LINUX
#include <linux/can.h>
#endif


namespace OneMotor::Can
{
    struct CanFrame
    {
        CanFrame() = default;
        uint32_t id{};
        uint8_t dlc{};
        uint8_t __pad{};
        uint8_t __res0{};
        uint8_t len8_dlc{};
        uint8_t data[ONE_MOTOR_CAN_MAX_DLEN]alignas(8){};
    };
#ifdef ONE_MOTOR_LINUX
    static_assert(sizeof(CanFrame) == sizeof(can_frame),
                  "Size mismatch between CanFrame and can_frame.");
    static_assert(alignof(CanFrame) == alignof(can_frame),
                  "Alignment mismatch between CanFrame and can_frame.");
    static_assert(std::is_standard_layout_v<CanFrame>,
                  "CanFrame must be a standard-layout type for safe casting.");
    static_assert(offsetof(CanFrame, id) == offsetof(can_frame, can_id),
                  "Offset mismatch for 'id' member.");
    static_assert(offsetof(CanFrame, dlc) == offsetof(can_frame, can_dlc),
                  "Offset mismatch for 'dlc' member.");
    static_assert(offsetof(CanFrame, data) == offsetof(can_frame, data),
                  "Offset mismatch for 'data' member.");
#endif
}

#endif //CANFRAME_HPP
