#include "OneMotor/Motor/DM/J4310Frame.hpp"
#ifdef ONE_MOTOR_LINUX
#include <format>
#endif
#include <sstream>
#include <iomanip>

namespace OneMotor::Motor::DM
{
    J4310Status::J4310Status(const Can::CanFrame& frame)
    {
        const auto* data = reinterpret_cast<const uint8_t*>(frame.data);
        auto tmp = static_cast<uint16_t>((data[1] << 8) | data[2]);
        position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);
        tmp = static_cast<uint16_t>((data[3] << 4) | data[4] >> 4);
        velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);
        tmp = static_cast<uint16_t>(((data[4] & 0x0f) << 8) | data[5]);
        torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

        ID = data[0] & 0x0F;
        status = static_cast<DMStatus>((frame.data[0] >> 4) & 0x0F);
        temperature_MOS = data[6];
        temperature_Rotor = data[7];
    }

    std::string J4310Status::format()
    {
#ifdef ONE_MOTOR_LINUX
        return std::format("ID: {:X}, Position: {}, Velocity: {}, Torque: {}, temperature: {},{} deg, ",
                           ID, position, velocity, torque, temperature_MOS,
                           temperature_Rotor);
#else
        std::ostringstream oss;
        oss << "ID: " << std::hex << std::uppercase << ID << std::dec
            << ", Position: " << position
            << ", Velocity: " << velocity
            << ", Torque: " << torque
            << ", temperature: " << temperature_MOS
            << "," << temperature_Rotor << " deg, ";
        return oss.str();
#endif
    }
}
