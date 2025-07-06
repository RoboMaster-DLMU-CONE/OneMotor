#include <OneMotor/Motor/DJI/MotorGuard.hpp>

using OneMotor::Motor::DJI::MotorGuard;

int main()
{
    std::array<uint8_t, 16> exit_frame_data{};
    exit_frame_data[0] = 0x07;
    exit_frame_data[1] = 0x21;

    MotorGuard::getInstance().guard({{"can0", exit_frame_data}});

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}
