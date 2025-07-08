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
    .Kp = 6,
    .Ki = 0,
    .Kd = 0,
    .MaxOutput = 2000,
    .Deadband = 30,
    .IntegralLimit = 500,
};
static constexpr PID_Params<float> ANG_DEFAULT_PARAMS{
    .Kp = 5,
    .Ki = 0,
    .Kd = 0,
    .MaxOutput = 15000,
    .Deadband = 30,
    .IntegralLimit = 1500,
};

[[noreturn]] int main()
{
    CanDriver driver("can0");
    driver.open();
    M3508<1, Position> m1(driver, POS_DEFAULT_PARAMS, ANG_DEFAULT_PARAMS);
    m1.setPosRef(1000);
    m1.enable();


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
        std::cout << "Enter parameter to change (ref, kp, kd, ki): ";
        std::cin >> param_to_change;

        if (param_to_change == "ref")
        {
            float ref;
            std::cout << "Enter new ref value: ";
            std::cin >> ref;
            m1.setPosRef(ref);
        }
        else
        {
            std::cout << "Invalid parameter." << std::endl;
        }
    }

    driver.close();
}
