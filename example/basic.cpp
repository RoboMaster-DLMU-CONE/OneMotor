#include <thread>
#include <one-motor/control/PID.hpp>
#include <one-motor/util/DeltaT.hpp>
#include <one-motor/thread/Othread.hpp>

#include "one-motor/can/CanDriver.hpp"
#include "one-motor/util/SpinLock.hpp"

using OneMotor::Util::DeltaT;
using OneMotor::Util::SpinLock;
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
    SpinLock lock;
    lock.lock();
    CanDriver can_driver("can0");
    lock.unlock();
    DeltaT deltat{};
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    printf("%f", deltat.getDeltaMS());
    thread.join();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    return 0;
}
