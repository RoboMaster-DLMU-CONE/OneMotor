#include <chrono>
#include <iostream>
#include <one/can/CanDriver.hpp>
#include <one/motor/lk/LkMotor.hpp>
#include <thread>

using namespace one::motor::units::literals;
using one::can::CanDriver;
using one::motor::lk::LkMotor;

int main() {

    CanDriver driver("can0");
    LkMotor<1> motor;

    if (auto init_result = motor.init(driver); !init_result) {
        std::cerr << "Failed to initialize M2006: "
                  << init_result.error().message << std::endl;
        return 1;
    }

    (void)motor.enable();
    motor.setPosUnitRef(2 * rev);

    while (true) {

        std::cout << "LK angular: " << motor.getStatus().angular << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}
