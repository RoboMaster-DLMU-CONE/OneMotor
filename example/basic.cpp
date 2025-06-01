#include <thread>
#include <one-motor/control/PID.hpp>
#include <one-motor/util/DeltaT.hpp>
#include <one-motor/thread/Othread.hpp>

#include "one-motor/can/CanDriver.hpp"

using OneMotor::Util::DeltaT;
using OneMotor::Control::PID_Params;
using OneMotor::Control::PIDController;
using OneMotor::thread::Othread;
using OneMotor::Can::CanDriver;

int main()
{
    Othread thread([]
    {
        constexpr PID_Params params{.Kp = 10.0, .Ki = 1.0, .Kd = 0.5};
        PIDController pid(params);
        pid.compute(1, 0.1);
    });
    CanDriver can_driver("can0");

    DeltaT deltat{};
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    printf("%f", deltat.getDeltaMS());
    thread.join();
    return 0;
}
