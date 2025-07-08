#include <iostream>
#include <thread>
#include <OneMotor/Control/PID.hpp>
#include <OneMotor/Util/DeltaT.hpp>
#include <OneMotor/Thread/Othread.hpp>

#include "OneMotor/Can/CanDriver.hpp"
#include "OneMotor/Can/CanFrame.hpp"
#include "OneMotor/Motor/DJI/M3508.hpp"
#include "OneMotor/Util/SpinLock.hpp"

using OneMotor::DeltaT;
using OneMotor::SpinLock;
using OneMotor::Control::PID_Params;
using OneMotor::Control::PIDController;
using OneMotor::Control::Positional;
using OneMotor::thread::Othread;
using OneMotor::Can::CanDriver;
using OneMotor::Motor::DJI::M3508;
using enum OneMotor::Motor::DJI::MotorMode;


int main()
{
    constexpr PID_Params<float> params{.Kp = 10.0, .Ki = 1.0, .Kd = 0.5};
    Othread thread([&params]
    {
        PIDController pid(params);
        pid.compute(1, 0.1);
    });
    SpinLock lock;
    lock.lock();
    CanDriver can_driver("can0");
    can_driver.open();
    lock.unlock();
    DeltaT deltat{};
    OneMotor::thread::sleep_for(std::chrono::milliseconds(500));
    printf("%f\n", deltat.getDeltaMS());
    thread.join();
    OneMotor::thread::sleep_for(std::chrono::seconds(3));
    can_driver.close();
    return 0;
}
