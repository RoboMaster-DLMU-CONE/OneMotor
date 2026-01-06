#include <OneMotor/Motor/DM/DmMotor.hpp>
#include <iostream>

#include "OneMotor/Motor/DM/DmPolicy.hpp"
#include "OneMotor/Motor/DM/DmTraits.hpp"
#include "OneMotor/Thread/Othread.hpp"

using namespace OneMotor::Units::literals;
using OneMotor::Motor::DM::J4310;
using OneMotor::Motor::DM::J4310_PosVel;

constexpr float cycle = 2 * std::numbers::pi;

int main() {
    OneMotor::Can::CanDriver driver("can0");
    // Remember to Change the ID
    J4310_PosVel j4310(driver, 0x53, 0x43);
    (void)j4310.setPidParams(0.01f, 0, 0.05f);

    (void)j4310.enable();
    OneMotor::Thread::sleep_for(std::chrono::seconds(4));

    (void)j4310.setZeroPosition();
    OneMotor::Thread::sleep_for(std::chrono::seconds(2));
    // Position Velocity mode

    // MIT mode
    (void)j4310.setRefs(360 * deg, 2 * rad / s, 0.01 * N * m);

    std::cout << j4310.getStatus().value().format() << std::endl;
    OneMotor::Thread::sleep_for(std::chrono::seconds(2));
    std::cout << j4310.getStatus().value().format() << std::endl;
    OneMotor::Thread::sleep_for(std::chrono::seconds(2));

    (void)j4310.disable();
    OneMotor::Thread::sleep_for(std::chrono::seconds(2));

    return 0;
}
