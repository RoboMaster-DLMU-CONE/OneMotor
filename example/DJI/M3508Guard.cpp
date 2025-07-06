#include <OneMotor/Motor/DJI/MotorGuard.hpp>

using OneMotor::Motor::DJI::MotorGuard;

int main()
{
    MotorGuard::getInstance().guard({"can0"});
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}
