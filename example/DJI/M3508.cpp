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


[[noreturn]] int main()
{
    constexpr PID_Params<float> params{
        .Kp = 5.0,
        .Ki = 0.2,
        .Kd = 0.1,
        .MaxOutput = 10000,
        .Deadband = 10,
        .IntegralLimit = 1000,
    };
    CanDriver driver("can0");
    (void)driver.open().map_error([](const auto& err) { std::cerr << err.message << std::endl; });
    M3508<1, Angular> m1(driver, params);
    m1.setRef(2000);
    (void)m1.enable();

    while (true)
    {
        std::string param_to_change;
        std::cout << "Enter parameter to change (ref, kp, kd, ki): ";
        std::cin >> param_to_change;

        if (param_to_change == "ref")
        {
            float ref;
            std::cout << "Enter new ref value: ";
            std::cin >> ref;
            m1.setRef(ref);
        }
        else if (param_to_change == "kp")
        {
            float kp;
            std::cout << "Enter new Kp value: ";
            std::cin >> kp;
            m1.editAngPID([=](PIDController* pid)
            {
                pid->Kp = kp;
                pid->reset();
            });
        }
        else if (param_to_change == "kd")
        {
            float kd;
            std::cout << "Enter new Kd value: ";
            std::cin >> kd;
            m1.editAngPID([=](PIDController* pid)
            {
                pid->Kd = kd;
                pid->reset();
            });
        }
        else if (param_to_change == "ki")
        {
            float ki;
            std::cout << "Enter new Ki value: ";
            std::cin >> ki;
            m1.editAngPID([=](PIDController* pid)
            {
                pid->Ki = ki;
                pid->reset();
            });
        }
        else
        {
            std::cout << "Invalid parameter." << std::endl;
        }
    }

    (void)driver.close().map_error([](const auto& err) { std::cerr << err.message << std::endl; });
}
