
#include <iostream>
#include <one/PID/PidChain.hpp>
#include <one/PID/PidConfig.hpp>
#include <one/PID/PidParams.hpp>
#include <string>

#include <one/motor/dji/DjiMotor.hpp>

using one::can::CanDriver;
using one::motor::dji::GM6020_Voltage;
using one::motor::dji::M2006;
using one::motor::dji::PIDFeatures;
using one::motor::dji::PosAngMode;
using one::pid::PidChain;
using one::pid::PidConfig;
using one::pid::PidParams;

static constexpr PidParams<> POS_DEFAULT_PARAMS{
    .Kp = 8.1f,
    .Ki = 0.0f,
    .Kd = 1.0f,
    .MaxOutput = 20000,
    .Deadband = 0,
    .IntegralLimit = 5000,
};
static constexpr PidParams<> ANG_DEFAULT_PARAMS{
    .Kp = 6.0f,
    .Ki = 0.5f,
    .Kd = 0.0f,
    .MaxOutput = 20000,
    .Deadband = 0,
    .IntegralLimit = 8000,
};

using namespace one::units::literals;
int main() {

    CanDriver driver("can0");
    GM6020_Voltage m1(
        driver,
        {.id = 4, .mode = PosAngMode{POS_DEFAULT_PARAMS, ANG_DEFAULT_PARAMS}});

    m1.setPosUnitRef(0 * rad);
    m1.setAngUnitRef(2 * rad / s);
    (void)m1.enable();

    std::thread thread([&] {
        while (true) {
            std::cout << m1.getStatus().total_angle << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(700));
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
