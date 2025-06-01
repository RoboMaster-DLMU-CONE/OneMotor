#ifndef DELTAT_HPP
#define DELTAT_HPP

#ifdef ONE_MOTOR_LINUX
#include <chrono>
#endif

#include "Arithmetic.hpp"

namespace OneMotor::Util
{
    template<Arithmetic T = float>
    class DeltaT
    {
    public:
        DeltaT();
        T getDeltaMS();
        void reset();
    private:
#ifdef ONE_MOTOR_LINUX
        std::chrono::time_point<std::chrono::steady_clock> last_time;
#endif
    };
}

#endif //DELTAT_HPP
