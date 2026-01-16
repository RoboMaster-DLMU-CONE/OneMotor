#include <OneMotor/Can/CanDriver.hpp>
#include <iostream>
#include <one/PID/PidChain.hpp>
#include <one/PID/PidConfig.hpp>
#include <one/PID/PidParams.hpp>
#include <string>

#include <OneMotor/Motor/DJI/DjiMotor.hpp>

using one::pid::PidChain;
using one::pid::PidConfig;
using one::pid::PidParams;
using OneMotor::Can::CanDriver;
using OneMotor::Motor::DJI::GM6020_Voltage;
using OneMotor::Motor::DJI::M2006;
using OneMotor::Motor::DJI::PIDFeatures;

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

int main() {
    constexpr auto conf1 =
        PidConfig<one::pid::Positional, float, PIDFeatures>(POS_DEFAULT_PARAMS);
    constexpr auto conf2 =
        PidConfig<one::pid::Positional, float, PIDFeatures>(ANG_DEFAULT_PARAMS);
    auto pid_chain = PidChain(conf1, conf2);

    CanDriver driver("can0");
    GM6020_Voltage<4, decltype(pid_chain)> m1(driver, {pid_chain});

    (void)m1.setPosRef(0 * deg);
    (void)m1.setAngRef(2 * rad / s);
    (void)m1.enable();

    std::thread thread([&] {
        while (true) {
            std::cout << m1.getStatus().value().total_angle << std::endl;
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
            (void)m1.setPosRef(ref * deg);
        } else if (param_to_change == "exit") {
            break;
        } else {
            std::cout << "Invalid parameter." << std::endl;
        }
    }
    return 0;
}
