#include <thread>
#include <one-motor/control/PID.hpp>
#include <one-motor/util/DeltaT.hpp>
#include <one-motor/thread/Othread.hpp>

using OneMotor::Util::DeltaT;
using OneMotor::Control::PID_Params;
using OneMotor::Control::PIDController;
using OneMotor::thread::Othread;

int main()
{
    Othread thread([]
    {
        constexpr PID_Params params{.Kp = 10.0, .Ki = 1.0, .Kd = 0.5};
        PIDController pid(params);
        pid.compute(1, 0.1);
    });

    DeltaT deltat{};
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    printf("%f", deltat.getDeltaMS());
    thread.join();
    return 0;
}
