#include <OneMotor/Can/CanDriver.hpp>
#include <iostream>
#include <one/PID/PidChain.hpp>
#include <one/PID/PidConfig.hpp>
#include <one/PID/PidParams.hpp>
#include <string>

#include "OneMotor/Motor/DJI/DjiMotor.hpp"

using one::pid::PidChain;
using one::pid::PidConfig;
using one::pid::PidParams;
using OneMotor::Can::CanDriver;
using OneMotor::Motor::DJI::M3508;
using OneMotor::Motor::DJI::makeM3508;
using OneMotor::Motor::DJI::PIDFeatures;

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

int main() {
    constexpr auto conf1 =
        PidConfig<one::pid::Positional, float, PIDFeatures>(POS_DEFAULT_PARAMS);
    constexpr auto conf2 =
        PidConfig<one::pid::Positional, float, PIDFeatures>(ANG_DEFAULT_PARAMS);
    const auto pid_chain = PidChain(conf1, conf2);

    CanDriver driver("can0");
    // 最简单的 helper function
    auto m1 = makeM3508<1>(driver, pid_chain);
    // 你也可以直接用别名：
    // M3508<1, decltype(pid_chain)> m1(driver, {pid_chain});

    (void)m1.setPosRef(2 * rev);
    (void)m1.enable();
    (void)m1.setPidParams(1, 0.05, 0);

    std::thread thread([&] {
        while (true) {
            std::cout << m1.getStatus().value().reduced_angle << std::endl;
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
            (void)m1.setPosRef(ref * deg);
        } else if (param_to_change == "exit") {
            break;
        } else {
            std::cout << "Invalid parameter." << std::endl;
        }
    }
    return 0;
}
