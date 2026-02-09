#include "one/thread/Othread.hpp"

#include <chrono>
#include <iostream>
#include <one/motor/dm/DmMotor.hpp>
#include <thread>

using namespace one::units::literals;
using one::motor::dm::J4310;

constexpr float cycle = 2 * std::numbers::pi;

int main() {
    one::can::CanDriver driver("can0");
    // Remember to Change the ID
    J4310 j4310(driver, {0x01, 0x11, one::motor::dm::MITMode{0.5f, 1.0f}});
    one::thread::sleep_for(std::chrono::seconds(4));

    (void)j4310.enable();
    one::thread::sleep_for(std::chrono::seconds(4));

    (void)j4310.setZeroPosition();
    one::thread::sleep_for(std::chrono::seconds(2));
    // Position Velocity mode

    // MIT mode
    (void)j4310.setUnitRefs(0 * deg, 0 * rad / s, 0 * N * m);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::cout << j4310.getStatus().format() << std::endl;
    one::thread::sleep_for(std::chrono::seconds(2));
    std::cout << j4310.getStatus().format() << std::endl;
    one::thread::sleep_for(std::chrono::seconds(2));

    (void)j4310.disable();
    one::thread::sleep_for(std::chrono::seconds(2));

    return 0;
}
