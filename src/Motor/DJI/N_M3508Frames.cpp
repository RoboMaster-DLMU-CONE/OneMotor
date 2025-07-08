#ifdef ONE_MOTOR_LINUX
#include <format>
#else
#include <sstream>
#include <iomanip>
#endif
// TODO: waiting for zephyr gcc14...
// I had been removing std::expected all day long...
// didn't expect zephyr gcc 12 have no std::format...


#include "OneMotor/Can/CanFrame.hpp"
#include "OneMotor/Motor/DJI/M3508Frames.hpp"

namespace OneMotor::Motor::DJI
{
    M3508RawStatusFrame::operator Can::CanFrame() const
    {
        Can::CanFrame frame{};
        frame.id = canId;
        frame.dlc = 8;
        frame.data[0] = static_cast<uint8_t>(ecd >> 8);
        frame.data[1] = static_cast<uint8_t>(ecd & 0xFF);
        frame.data[2] = static_cast<uint8_t>(rpm >> 8);
        frame.data[3] = static_cast<uint8_t>(rpm & 0xFF);
        frame.data[4] = static_cast<uint8_t>(current >> 8);
        frame.data[5] = static_cast<uint8_t>(current & 0xFF);
        frame.data[6] = temperature;
        return frame;
    }

    M3508RawStatusFrame::M3508RawStatusFrame(const Can::CanFrame frame)
    {
        canId = frame.id;
        ecd = static_cast<uint16_t>(frame.data[0] << 8) | frame.data[1];
        rpm = static_cast<int16_t>(static_cast<uint16_t>(frame.data[2] << 8) | frame.data[3]);
        current = static_cast<int16_t>(static_cast<uint16_t>(frame.data[4] << 8) | frame.data[5]);
        temperature = frame.data[6];
    }

    inline std::string M3508RawStatusFrame::format() const
    {
#ifdef ONE_MOTOR_LINUX
        return std::format("ID: {:X}, Ang: {} RPM, ecd: {} / 8191, CRT: {} mA, temperature: {} deg",
                           canId, rpm, ecd, current, temperature);
#else
        std::ostringstream oss;
        oss << "ID: " << std::hex << std::uppercase << canId << std::dec
            << ", Ang: " << rpm << " RPM, ecd: " << ecd << " / 8191, CRT: "
            << current << " mA, temperature: " << temperature << " deg";
        return oss.str();
#endif
    }

    std::string M3508Status::format() const
    {
#ifdef ONE_MOTOR_LINUX
        return std::format(
            "M3508 Status:\n"
            "\t- ECD: {} (last: {})\n"
            "\t- Angle (single round): {:.2f} deg\n"
            "\t- Angular Velocity: {:.2f} deg/s\n"
            "\t- Total Angle: {:.2f} deg\n"
            "\t- Total Rounds: {}\n"
            "\t- Real Current: {} mA\n"
            "\t- Output Current: {}\n"
            "\t- Temperature: {} °C",
            ecd, last_ecd,
            angle_single_round,
            angular,
            total_angle,
            total_round,
            real_current,
            output_current,
            temperature);
#else
        std::ostringstream oss;
        oss << "M3508 Status:\n"
            << "\t- ECD: " << ecd << " (last: " << last_ecd << ")\n"
            << "\t- Angle (single round): " << std::fixed << std::setprecision(2) << angle_single_round << " deg\n"
            << "\t- Angular Velocity: " << std::fixed << std::setprecision(2) << angular << " deg/s\n"
            << "\t- Total Angle: " << std::fixed << std::setprecision(2) << total_angle << " deg\n"
            << "\t- Total Rounds: " << total_round << "\n"
            << "\t- Real Current: " << real_current << " mA\n"
            << "\t- Output Current: " << output_current << "\n"
            << "\t- Temperature: " << temperature << " °C";
        return oss.str();
#endif
    }
}
