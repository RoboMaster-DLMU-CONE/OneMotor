#ifndef ONEMOTOR_TRAITS_HPP
#define ONEMOTOR_TRAITS_HPP
#include <cstdint>

namespace OneMotor::Motor::DJI
{
    template <typename Derived>
    struct MotorTraits
    {
        static consteval bool has_gearbox()
        {
            return Derived::gearbox;
        }

        static constexpr float ecd_to_angle(const float ecd)
        {
            return ecd * 360.f / Derived::encoder_resolution;
        }

        template <uint8_t motor_id>
        static consteval uint16_t control_id()
        {
            return Derived::template cal_control_id<motor_id>();
        }

        template <uint8_t motor_id>
        static consteval uint8_t control_offset()
        {
            return Derived::template cal_control_offset<motor_id>();
        }

        template <uint8_t motor_id>
        static consteval uint16_t feedback_id()
        {
            return Derived::template cal_feedback_id<motor_id>();
        }
    };

    struct M3508Traits : MotorTraits<M3508Traits>
    {
        static constexpr uint16_t max_current = 16384;
        static constexpr uint16_t encoder_resolution = 8192;
        static constexpr bool gearbox = true;
        static constexpr uint8_t reduction_ratio = 19;
        static constexpr uint8_t max_id = 8;

        template <uint8_t motor_id>
        static consteval uint16_t cal_control_id()
        {
            if constexpr (motor_id <= 4)
            {
                return 0x200;
            }
            else
            {
                return 0x1FF;
            }
        }

        template <uint8_t motor_id>
        static consteval uint16_t cal_control_offset()
        {
            if constexpr (motor_id <= 4)
            {
                return (motor_id - 1) * 2;
            }
            else
            {
                return (motor_id - 4 - 1) * 2;
            }
        }

        template <uint8_t motor_id>
        static consteval uint16_t cal_feedback_id()
        {
            return motor_id + 0x200;
        }
    };

    struct M2006Traits : MotorTraits<M2006Traits>
    {
        static constexpr uint16_t max_current = 16384;
        static constexpr uint16_t encoder_resolution = 8192;
        static constexpr bool gearbox = true;
        static constexpr uint8_t reduction_ratio = 36;
        static constexpr uint8_t max_id = 8;

        template <uint8_t motor_id>
        static consteval uint16_t cal_control_id()
        {
            if constexpr (motor_id <= 4)
            {
                return 0x200;
            }
            else
            {
                return 0x1FF;
            }
        }

        template <uint8_t motor_id>
        static consteval uint16_t cal_control_offset()
        {
            if constexpr (motor_id <= 4)
            {
                return (motor_id - 1) * 2;
            }
            else
            {
                return (motor_id - 4 - 1) * 2;
            }
        }

        template <uint8_t motor_id>
        static consteval uint16_t cal_feedback_id()
        {
            return motor_id + 0x200;
        }
    };

    struct GM6020VoltageTraits : MotorTraits<GM6020VoltageTraits>
    {
        static constexpr uint16_t max_current = 16384;
        static constexpr uint16_t encoder_resolution = 8192;
        static constexpr bool gearbox = false;
        static constexpr uint8_t max_id = 7;

        template <uint8_t motor_id>
        static consteval uint16_t cal_control_id()
        {
            if constexpr (motor_id <= 4)
            {
                return 0x1FF;
            }
            else
            {
                return 0x2FF;
            }
        }

        template <uint8_t motor_id>
        static consteval uint16_t cal_control_offset()
        {
            if constexpr (motor_id <= 4)
            {
                return (motor_id - 1) * 2;
            }
            else
            {
                return (motor_id - 4 - 1) * 2;
            }
        }

        template <uint8_t motor_id>
        static consteval uint16_t cal_feedback_id()
        {
            return motor_id + 0x204;
        }
    };

    struct GM6020CurrentTraits : MotorTraits<GM6020CurrentTraits>
    {
        static constexpr uint16_t max_output = 25000;
        static constexpr uint16_t encoder_resolution = 8192;
        static constexpr bool gearbox = false;
        static constexpr uint8_t max_id = 7;

        template <uint8_t motor_id>
        static consteval uint16_t cal_control_id()
        {
            if constexpr (motor_id <= 4)
            {
                return 0x1FE;
            }
            else
            {
                return 0x2FE;
            }
        }

        template <uint8_t motor_id>
        static consteval uint16_t cal_control_offset()
        {
            if constexpr (motor_id <= 4)
            {
                return (motor_id - 1) * 2;
            }
            else
            {
                return (motor_id - 4 - 1) * 2;
            }
        }

        template <uint8_t motor_id>
        static consteval uint16_t cal_feedback_id()
        {
            return motor_id + 0x204;
        }
    };
}

#endif //ONEMOTOR_TRAITS_HPP
