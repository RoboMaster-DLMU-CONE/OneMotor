#include <OneMotor/Can/CanDriver.hpp>
#include <OneMotor/Control/PIDChain.hpp>

#include <iostream>
#include <string>

#include "OneMotor/Motor/DJI/DjiMotor.hpp"

using OneMotor::Can::CanDriver;
using OneMotor::Control::createPIDChain;
using OneMotor::Control::PID_Params;
using OneMotor::Control::PIDChain;
using OneMotor::Control::Positional;
using OneMotor::Motor::DJI::createDjiMotor;
using OneMotor::Motor::DJI::DjiMotor;
using OneMotor::Motor::DJI::GM6020CurrentTraits;
using OneMotor::Motor::DJI::GM6020VoltageTraits;
using OneMotor::Motor::DJI::PIDFeatures;

static constexpr PID_Params<> POS_DEFAULT_PARAMS{
    .Kp = 8.1,
    .Ki = 0.04,
    .Kd = 0.5,
    .Deadband = 50,
    .IntegralLimit = 2000,
};
static constexpr PID_Params<> ANG_DEFAULT_PARAMS{
    .Kp = 2.5,
    .Ki = 0.05,
    .Kd = 0.5,
    .Deadband = 10,
    .IntegralLimit = 100,
};

int main() {
    auto p = PIDChain(POS_DEFAULT_PARAMS, ANG_DEFAULT_PARAMS);
    auto pid_chain =
        createPIDChain()
            .add<Positional, float, PIDFeatures>(POS_DEFAULT_PARAMS)
            .add<Positional, float, PIDFeatures>(ANG_DEFAULT_PARAMS)
            .build();
    CanDriver driver("can0");
    auto m1 = createDjiMotor<GM6020VoltageTraits, 1>(driver, pid_chain);
    auto m2 =
        createDjiMotor<OneMotor::Motor::DJI::M2006Traits, 5>(driver, pid_chain);
    m1.setPosRef(1000);
    m1.setAngRef(100);
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
            m1.setPosRef(ref);
        } else if (param_to_change == "exit") {
            break;
        } else {
            std::cout << "Invalid parameter." << std::endl;
        }
    }
    return 0;
}
