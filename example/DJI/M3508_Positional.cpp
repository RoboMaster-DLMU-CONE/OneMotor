#include <iostream>
#include <one/PID/PidChain.hpp>
#include <one/PID/PidConfig.hpp>
#include <one/PID/PidParams.hpp>
#include <string>

#include <one/motor/dji/DjiMotor.hpp>

using one::can::CanDriver;
using one::motor::dji::M3508;
using one::pid::PidChain;
using one::pid::PidConfig;
using one::pid::PidParams;

static constexpr PidParams<> POS_DEFAULT_PARAMS{
    .Kp = 1,
    .Ki = 0.05,
    .Kd = 0,
    .MaxOutput = 20000,
    .Deadband = 50,
    .IntegralLimit = 1000,
};
static constexpr PidParams<> ANG_DEFAULT_PARAMS{
    .Kp = 0.1,
    .Ki = 0.05,
    .Kd = 0.01,
    .MaxOutput = 8000,
    .Deadband = 10,
    .IntegralLimit = 100,
};

using namespace one::motor::units::literals;
int main() {

    CanDriver driver("can0");

    M3508 m1(driver, {1, one::motor::dji::PosAngMode{POS_DEFAULT_PARAMS,
                                                     ANG_DEFAULT_PARAMS}});

    m1.setPosUnitRef(2 * rev);
    (void)m1.enable();

    std::thread thread([&] {
        while (true) {
            std::cout << m1.getStatus().reduced_angle << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });
    thread.detach();

    while (true) {
        std::string param_to_change;
        std::cout << "Enter parameter to change ('ref') or 'exit': ";
        std::cin >> param_to_change;

        if (param_to_change == "ref") {
            float ref;
            std::cout << "Enter new ref value: ";
            std::cin >> ref;
            m1.setPosUnitRef(ref * deg);
        } else if (param_to_change == "exit") {
            break;
        } else {
            std::cout << "Invalid parameter." << std::endl;
        }
    }
    return 0;
}
