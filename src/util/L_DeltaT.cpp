#include <one-motor/util/DeltaT.hpp>

namespace OneMotor::Util
{
    DeltaT::DeltaT()
    {
        last_time = std::chrono::steady_clock::now();
    }

    double DeltaT::getDeltaMS()
    {
        const auto now = std::chrono::steady_clock::now();
        const std::chrono::duration<double, std::milli> delta = now - last_time;
        last_time = now;
        return delta.count();
    }

    void DeltaT::reset()
    {
        last_time = std::chrono::steady_clock::now();
    }
}