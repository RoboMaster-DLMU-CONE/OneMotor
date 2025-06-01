#ifndef DELTAT_HPP
#define DELTAT_HPP

#ifdef ONE_MOTOR_LINUX
#include <chrono>
#endif

namespace OneMotor::Util
{
    class DeltaT
    {
    public:
        DeltaT();
        double getDeltaMS();
        void reset();
    private:
#ifdef ONE_MOTOR_LINUX
        std::chrono::time_point<std::chrono::steady_clock> last_time;
#endif
    };


}

#endif //DELTAT_HPP
