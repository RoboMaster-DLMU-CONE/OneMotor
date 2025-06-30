#include "OneMotor/Motor/DM/J4310.hpp"

#include <iostream>

#include "OneMotor/Thread/Othread.hpp"

int main()
{
    OneMotor::Can::CanDriver driver("can0");
    auto _ = driver.open();
    OneMotor::Motor::DM::J4310 j4310(driver, 0x53, 0x43);

    _ = j4310.enable();
    OneMotor::thread::sleep_for(std::chrono::seconds(2));

    _ = j4310.setZeroPosition();
    OneMotor::thread::sleep_for(std::chrono::seconds(2));

    _ = j4310.posVelControl(12.56, 6.28);
    OneMotor::thread::sleep_for(std::chrono::seconds(2));

    if (auto result = j4310.getStatus(); result.has_value()) std::cout << result.value().format() << std::endl;
    else std::cerr << result.error() << std::endl;
    OneMotor::thread::sleep_for(std::chrono::seconds(2));

    _ = j4310.disable();
    OneMotor::thread::sleep_for(std::chrono::seconds(2));

    return 0;
}
