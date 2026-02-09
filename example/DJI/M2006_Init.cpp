#include <chrono>
#include <iostream>
#include <one/PID/PidChain.hpp>
#include <one/PID/PidConfig.hpp>
#include <one/PID/PidParams.hpp>
#include <one/can/CanDriver.hpp>
#include <one/motor/dji/DjiMotor.hpp>
#include <one/units/Units.hpp>
#include <thread>
#include <utility>

using namespace one::units::literals;
using one::can::CanDriver;
using one::motor::dji::M2006;
using one::pid::PidChain;
using one::pid::PidConfig;
using one::pid::PidParams;

static constexpr PidParams<> POS_DEFAULT_PARAMS{
    .Kp = 8.1f,
    .Ki = 0.04f,
    .Kd = 0.5f,
    .MaxOutput = 20000,
    .Deadband = 50,
    .IntegralLimit = 2000,
};
static constexpr PidParams<> ANG_DEFAULT_PARAMS{
    .Kp = 2.5f,
    .Ki = 0.05f,
    .Kd = 0.5f,
    .MaxOutput = 8000,
    .Deadband = 10,
    .IntegralLimit = 100,
};

int main() {

    CanDriver driver("can0");
    M2006 motor;

    if (auto init_result = motor.init(
            driver, {2, one::motor::dji::PosAngMode{POS_DEFAULT_PARAMS,
                                                    ANG_DEFAULT_PARAMS}});
        !init_result) {
        std::cerr << "Failed to initialize M2006: "
                  << init_result.error().message << std::endl;
        return 1;
    }

    (void)motor.enable();
    motor.setPosUnitRef(2 * rev);

    for (int i = 0; i < 5; ++i) {

        std::cout << "M2006 reduced angle: " << motor.getStatus().reduced_angle
                  << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}
