#include <OneMotor/Motor/DM/DmMotor.hpp>
#include <iostream>

#include "OneMotor/Motor/DM/DmPolicy.hpp"
#include "OneMotor/Motor/DM/DmTraits.hpp"
#include "OneMotor/Thread/Othread.hpp"

using namespace OneMotor::Units::literals;
using OneMotor::Motor::DM::J4310;
using OneMotor::Motor::DM::MITPolicy;

constexpr float cycle = 2 * std::numbers::pi;

int main() {
    OneMotor::Can::CanDriver driver("can0");
    // Remember to Change the ID
    J4310 j4310(driver, 0x53, 0x43);
    // ,MITPolicy<OneMotor::Motor::DM::J4310Traits>(0.01f, 0.05f));
    (void)j4310.setPidParams(0.01f, 0, 0.05f);

    (void)j4310.enable();
    OneMotor::Thread::sleep_for(std::chrono::seconds(4));

    (void)j4310.setZeroPosition();
    OneMotor::Thread::sleep_for(std::chrono::seconds(2));
    // Position Velocity mode

    // MIT mode
    (void)j4310.setTorRef(0.01 * N * m);
    (void)j4310.setAngRef(1 * rad / s);
    (void)j4310.setPosRef(360 * deg);

    std::cout << j4310.getStatus().value().format() << std::endl;
    OneMotor::Thread::sleep_for(std::chrono::seconds(2));
    std::cout << j4310.getStatus().value().format() << std::endl;

    (void)j4310.disable();
    OneMotor::Thread::sleep_for(std::chrono::seconds(2));

    return 0;
}
