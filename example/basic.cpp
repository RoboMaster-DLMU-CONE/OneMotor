#include <iostream>
#include <thread>
#include <one-motor/control/PID.hpp>
#include <one-motor/util/DeltaT.hpp>
#include <one-motor/thread/Othread.hpp>

#include "one-motor/can/CanDriver.hpp"
#include "one-motor/can/CanFrame.hpp"
#include "one-motor/motor/DJI/M3508.hpp"
#include "one-motor/util/SpinLock.hpp"

using OneMotor::Util::DeltaT;
using OneMotor::Util::SpinLock;
using OneMotor::Control::PID_Params;
using OneMotor::Control::PIDController;
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
    auto _ = can_driver.open();
    lock.unlock();
    DeltaT deltat{};
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    printf("%f\n", deltat.getDeltaMS());
    thread.join();

    std::cout << "m3508" << std::endl;
    M3508<1, Angular> m3508_1(can_driver, params);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    _ = can_driver.close();
    return 0;
}
