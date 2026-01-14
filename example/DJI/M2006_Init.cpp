#include <OneMotor/Can/CanDriver.hpp>
#include <OneMotor/Motor/DJI/DjiMotor.hpp>
#include <OneMotor/Units/Units.hpp>
#include <chrono>
#include <iostream>
#include <one/PID/PidChain.hpp>
#include <one/PID/PidConfig.hpp>
#include <one/PID/PidParams.hpp>
#include <thread>
#include <utility>

using namespace OneMotor::Units::literals;
using one::pid::PidChain;
using one::pid::PidConfig;
using one::pid::PidParams;
using OneMotor::Can::CanDriver;
using OneMotor::Motor::DJI::DjiPolicy;
using OneMotor::Motor::DJI::M2006;
using OneMotor::Motor::DJI::M2006Traits;
using OneMotor::Motor::DJI::PIDFeatures;

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
    constexpr auto pos_config =
        PidConfig<one::pid::Positional, float, PIDFeatures>(POS_DEFAULT_PARAMS);
    constexpr auto ang_config =
        PidConfig<one::pid::Positional, float, PIDFeatures>(ANG_DEFAULT_PARAMS);
    auto pid_chain = PidChain(pos_config, ang_config);

    CanDriver driver("can0");
    M2006<2, decltype(pid_chain)> motor;
    auto policy = DjiPolicy<M2006Traits<2>, decltype(pid_chain)>{pid_chain};

    if (auto init_result = motor.init(driver, policy); !init_result) {
        std::cerr << "Failed to initialize M2006: "
                  << init_result.error().message << std::endl;
        return 1;
    }

    (void)motor.enable();
    (void)motor.setPosRef(2 * rev);

    for (int i = 0; i < 5; ++i) {
        if (auto status = motor.getStatus(); status) {
            std::cout << "M2006 reduced angle: " << status->reduced_angle
                      << std::endl;
        } else {
            std::cerr << "Failed to read status: "
                      << status.error().message << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}
