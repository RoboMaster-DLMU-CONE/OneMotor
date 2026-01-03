#ifndef ONEMOTOR_TRAITS_HPP
#define ONEMOTOR_TRAITS_HPP
#include <cstdint>

namespace OneMotor::Motor::DJI {

template <uint8_t motor_id> struct M3508Traits {
    static constexpr uint16_t max_current = 16384;
    static constexpr uint16_t encoder_resolution = 8192;
    static constexpr bool gearbox = true;
    static constexpr uint8_t reduction_ratio = 19;
    static constexpr uint8_t max_id = 8;
    static_assert(motor_id > 0 && motor_id < max_id, "Invalid motor_id");

    static consteval uint16_t control_id() {
        if constexpr (motor_id <= 4) {
            return 0x200;
        } else {
            return 0x1FF;
        }
    }

    static consteval uint16_t control_offset() {
        if constexpr (motor_id <= 4) {
            return (motor_id - 1) * 2;
        } else {
            return (motor_id - 4 - 1) * 2;
        }
    }

    static consteval uint16_t feedback_id() { return motor_id + 0x200; }
};

template <uint8_t motor_id> struct M2006Traits {
    static constexpr uint16_t max_current = 16384;
    static constexpr uint16_t encoder_resolution = 8192;
    static constexpr bool gearbox = true;
    static constexpr uint8_t reduction_ratio = 36;
    static constexpr uint8_t max_id = 8;
    static_assert(motor_id > 0 && motor_id < max_id, "Invalid motor_id");

    static consteval uint16_t control_id() {
        if constexpr (motor_id <= 4) {
            return 0x200;
        } else {
            return 0x1FF;
        }
    }

    static consteval uint16_t control_offset() {
        if constexpr (motor_id <= 4) {
            return (motor_id - 1) * 2;
        } else {
            return (motor_id - 4 - 1) * 2;
        }
    }

    static consteval uint16_t feedback_id() { return motor_id + 0x200; }
};

template <uint8_t motor_id> struct GM6020VoltageTraits {
    static constexpr uint16_t max_current = 16384;
    static constexpr uint16_t encoder_resolution = 8192;
    static constexpr bool gearbox = false;
    static constexpr uint8_t max_id = 7;
    static_assert(motor_id > 0 && motor_id < max_id, "Invalid motor_id");

    static consteval uint16_t control_id() {
        if constexpr (motor_id <= 4) {
            return 0x1FF;
        } else {
            return 0x2FF;
        }
    }

    static consteval uint16_t control_offset() {
        if constexpr (motor_id <= 4) {
            return (motor_id - 1) * 2;
        } else {
            return (motor_id - 4 - 1) * 2;
        }
    }

    static consteval uint16_t feedback_id() { return motor_id + 0x204; }
};

template <uint8_t motor_id> struct GM6020CurrentTraits {
    static constexpr uint16_t max_output = 25000;
    static constexpr uint16_t encoder_resolution = 8192;
    static constexpr bool gearbox = false;
    static constexpr uint8_t max_id = 7;
    static_assert(motor_id > 0 && motor_id < max_id, "Invalid motor_id");

    static consteval uint16_t control_id() {
        if constexpr (motor_id <= 4) {
            return 0x1FE;
        } else {
            return 0x2FE;
        }
    }

    static consteval uint16_t control_offset() {
        if constexpr (motor_id <= 4) {
            return (motor_id - 1) * 2;
        } else {
            return (motor_id - 4 - 1) * 2;
        }
    }

    static consteval uint16_t feedback_id() { return motor_id + 0x204; }
};
} // namespace OneMotor::Motor::DJI

#endif // ONEMOTOR_TRAITS_HPP
