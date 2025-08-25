#include "OneMotor/Motor/DM/J4310.hpp"

#include <iostream>

#include "OneMotor/Thread/Othread.hpp"
#include "OneMotor/Util/Panic.hpp"

int main()
{
    OneMotor::Can::CanDriver driver("can0");
    (void)driver.open().or_else([](const auto& e)
    {
        OneMotor::panic(std::format("Can't Open 'can0' port: {}", e.message));
    });
    OneMotor::Motor::DM::J4310 j4310(driver, 0x53, 0x43);

    (void)j4310.enable();
    OneMotor::Thread::sleep_for(std::chrono::seconds(2));

    (void)j4310.setZeroPosition();
    OneMotor::Thread::sleep_for(std::chrono::seconds(2));

    (void)j4310.posVelControl(12.56, 6.28);
    OneMotor::Thread::sleep_for(std::chrono::seconds(2));

    if (auto result = j4310.getStatus(); result) std::cout << result.value().format() << std::endl;
    else std::cerr << result.error().message << std::endl;
    OneMotor::Thread::sleep_for(std::chrono::seconds(2));

    (void)j4310.disable();
    OneMotor::Thread::sleep_for(std::chrono::seconds(2));

    return 0;
}
