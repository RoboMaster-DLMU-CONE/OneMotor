#include <OneMotor/Motor/DM/DmFrame.hpp>
#include <format>
#include <mp-units/framework.h>

using namespace OneMotor::Units::literals;
namespace OneMotor::Motor::DM {
DmStatus::DmStatus(const Can::CanFrame &frame) {
    const auto *data = reinterpret_cast<const uint8_t *>(frame.data);
    auto tmp = static_cast<uint16_t>((data[1] << 8) | data[2]);
    position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16) * rad;
    tmp = static_cast<uint16_t>((data[3] << 4) | data[4] >> 4);
    velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12) * rad / s;
    tmp = static_cast<uint16_t>(((data[4] & 0x0f) << 8) | data[5]);
    torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12) * N * m;

    ID = data[0] & 0x0F;
    status = static_cast<DmCode>((frame.data[0] >> 4) & 0x0F);
    temperature_MOS = mp_units::point<deg_C>(data[6]);
    temperature_Rotor = mp_units::point<deg_C>(data[7]);
}

std::string DmStatus::format() const {
    return std::format("ID: {:X}, Position: {}, Velocity: {}, Torque: {}, "
                       "temperature: {},{} deg, ",
                       ID, position, velocity, torque,
                       temperature_MOS.quantity_from_zero(),
                       temperature_Rotor.quantity_from_zero());
}
} // namespace OneMotor::Motor::DM
