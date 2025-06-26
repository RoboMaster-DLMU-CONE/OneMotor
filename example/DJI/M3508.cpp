#include <one-motor/can/CanDriver.hpp>
#include <one-motor/control/PID.hpp>
#include <one-motor/motor/DJI/M3508.hpp>

#include "one-motor/thread/Othread.hpp"

#include <print>

using OneMotor::Control::PID_Params;
using OneMotor::Can::CanDriver;
using OneMotor::Motor::DJI::M3508;
using enum OneMotor::Motor::DJI::MotorMode;


int main()
{
    constexpr PID_Params<float> params{
        .Kp = 10.0,
        .Ki = 0.1,
        .Kd = 0.0,
        .MaxOutput = 10000,
        .Deadband = 10,
        .IntegralLimit = 1000,
    };
    CanDriver driver("can0");
    auto _ = driver.open();
    M3508<1, Angular> m1(driver, params);
    m1.setRef(100);
    _ = m1.enable();
    uint8_t x = 0;
    while (x < 10)
    {
        auto status = m1.getStatus();
        std::println("{}", status);

        OneMotor::thread::Othread::sleep_for(500 * 1000 * 1000);
        x++;
    }
    _ = driver.close();
}
