#include <format>

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
        return std::format("ID: {:X}, Ang: {} RPM, ecd: {} / 8191, CRT: {} mA, temperature: {} deg",
                           canId, rpm, ecd, current, temperature);
    }

    std::string M3508Status::format() const
    {
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
    }
}
