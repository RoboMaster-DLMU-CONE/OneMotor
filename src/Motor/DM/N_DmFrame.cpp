#include <OneMotor/Motor/DM/DmFrame.hpp>
#include <format>

namespace OneMotor::Motor::DM {

DmStatus DmStatus::fromPlain(const DmStatusPlain &plain) {
    using namespace OneMotor::Units::literals;
    return {
        .ID = plain.ID,
        .status = plain.status,
        .position = plain.position * rad,
        .velocity = plain.velocity * rad / s,
        .torque = plain.torque * N * m,
        .temperature_MOS = mp_units::point<deg_C>(plain.temperature_MOS),
        .temperature_Rotor = mp_units::point<deg_C>(plain.temperature_Rotor),
    };
}

#ifdef ONE_MOTOR_LINUX
std::string DmStatus::format() const {
    return std::format("ID: {:X}, Position: {}, Velocity: {}, Torque: {}, "
                       "temperature: {},{} deg, ",
                       ID, position, velocity, torque,
                       temperature_MOS.quantity_from_zero(),
                       temperature_Rotor.quantity_from_zero());
}
#endif
} // namespace OneMotor::Motor::DM
