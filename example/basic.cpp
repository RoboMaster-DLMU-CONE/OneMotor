#include <thread>
#include <one-motor/util/DeltaT.hpp>

using OneMotor::Util::DeltaT;

int main()
{
    DeltaT deltat;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    printf("%f", deltat.getDeltaMS());
    return 0;
}