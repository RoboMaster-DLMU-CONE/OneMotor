#include "OneMotor/Motor/DM/J4310.hpp"

#include <iostream>

#include "OneMotor/Thread/Othread.hpp"

int main()
{
    OneMotor::Can::CanDriver driver("can0");
    OneMotor::Motor::DM::J4310 j4310(driver, 0x51, 0x41);

    auto _ = j4310.enable();
    OneMotor::thread::Othread::sleep_for(2 * 1000 * 1000 * 1000);

    _ = j4310.setZeroPosition();
    OneMotor::thread::Othread::sleep_for(2 * 1000 * 1000 * 1000);

    _ = j4310.posVelControl(3.14, 3.14);

    if (auto result = j4310.getStatus(); result.has_value()) std::cout << result.value().format() << std::endl;
    else std::cerr << result.error() << std::endl;
    OneMotor::thread::Othread::sleep_for(2 * 1000 * 1000 * 1000);

    _ = j4310.disable();
    OneMotor::thread::Othread::sleep_for(2 * 1000 * 1000 * 1000);

    return 0;
}
