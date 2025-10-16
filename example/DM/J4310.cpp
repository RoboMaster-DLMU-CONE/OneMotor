#include "OneMotor/Motor/DM/J4310.hpp"

#include <iostream>

#include "OneMotor/Thread/Othread.hpp"
#include "OneMotor/Util/Panic.hpp"

constexpr float cycle = 2 * std::numbers::pi;

int main()
{
    OneMotor::Can::CanDriver driver("can0");
    // Remember to Change the ID
    OneMotor::Motor::DM::J4310 j4310(driver, 0x15, 0x05);

    (void)j4310.enable();
    OneMotor::Thread::sleep_for(std::chrono::seconds(2));

    (void)j4310.setZeroPosition();
    OneMotor::Thread::sleep_for(std::chrono::seconds(2));
    // Position Velocity mode
    // (void)j4310.posVelControl(2 * cycle, cycle);
    // OneMotor::Thread::sleep_for(std::chrono::seconds(2));

    // MIT mode
    // No-load params
    (void)j4310.MITControl(2 * cycle, cycle / 4.0, 0.01, 0.01f, 0.05f);

    std::cout << j4310.getStatus().format() << std::endl;
    OneMotor::Thread::sleep_for(std::chrono::seconds(2));

    std::cout << j4310.getNewStatus(1000).value().format() << std::endl;

    (void)j4310.disable();
    OneMotor::Thread::sleep_for(std::chrono::seconds(2));

    return 0;
}
