#include <OneMotor/Can/CanDriver.hpp>
#include <OneMotor/Control/PID.hpp>
#include <OneMotor/Motor/DJI/M3508.hpp>

#include "OneMotor/Thread/Othread.hpp"

#include <iostream>
#include <string>

using OneMotor::Control::PID_Params;
using OneMotor::Motor::DJI::PIDController;
using OneMotor::Control::Positional;
using OneMotor::Can::CanDriver;
using OneMotor::Motor::DJI::M3508;
using enum OneMotor::Motor::DJI::MotorMode;

static constexpr PID_Params<float> POS_DEFAULT_PARAMS{
    .Kp = 3,
    .Ki = 0,
    .Kd = 0,
    .MaxOutput = 3000,
    .Deadband = 40,
    .IntegralLimit = 500,
};
static constexpr PID_Params<float> ANG_DEFAULT_PARAMS{
    .Kp = 0.6,
    .Ki = 0.05,
    .Kd = 0.1,
    .MaxOutput = 8000,
    .Deadband = 50,
    .IntegralLimit = 100,
};

int main()
{
    CanDriver driver("can0");
    (void)driver.open()
                .or_else([](const auto& err) { std::cerr << err.message << std::endl; });
    M3508<1, Position> m1(driver, POS_DEFAULT_PARAMS, ANG_DEFAULT_PARAMS);
    m1.setPosRef(1000);
    (void)m1.enable();

    std::thread thread([&]
    {
        while (true)
        {
            std::cout << m1.getStatus().total_angle << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(700));
        }
    });
    thread.detach();

    while (true)
    {
        std::string param_to_change;
        std::cout << "Enter parameter to change (ref, kp, kd, ki) or 'exit': ";
        std::cin >> param_to_change;

        if (param_to_change == "ref")
        {
            float ref;
            std::cout << "Enter new ref value: ";
            std::cin >> ref;
            m1.setPosRef(ref);
        }
        else if (param_to_change == "exit")
        {
            break;
        }
        else
        {
            std::cout << "Invalid parameter." << std::endl;
        }
    }

    (void)driver.close();
    return 0;
}
