#include <OneMotor/Can/CanDriver.hpp>
#include <OneMotor/Motor/DJI/DjiMotor.hpp>
#include <OneMotor/Motor/DM/DmMotor.hpp>
#include <OneMotor/Motor/IMotor.hpp>
#include <OneMotor/Units/Units.hpp>
#include <array>
#include <cstddef>
#include <iostream>
#include <limits>
#include <memory>
#include <one/PID/PidChain.hpp>
#include <one/PID/PidConfig.hpp>
#include <one/PID/PidParams.hpp>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

namespace DM = OneMotor::Motor::DM;
namespace DJI = OneMotor::Motor::DJI;

using namespace OneMotor::Units::literals;
using one::pid::PidChain;
using one::pid::PidConfig;
using one::pid::PidParams;
using OneMotor::Can::CanDriver;
using OneMotor::Motor::IMotor;

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
        PidConfig<one::pid::Positional, float, DJI::PIDFeatures>(
            POS_DEFAULT_PARAMS);
    constexpr auto ang_config =
        PidConfig<one::pid::Positional, float, DJI::PIDFeatures>(
            ANG_DEFAULT_PARAMS);
    auto pid_chain = PidChain(pos_config, ang_config);

    CanDriver driver("can0");
    auto dm_motor = std::make_unique<DM::J4310_MIT>(driver, 0x01, 0x11);
    dm_motor->setPidParams(5, 0, 0.5);
    (void)dm_motor->setZeroPosition();
    (void)dm_motor->enable();

    auto dji_policy =
        DJI::DjiPolicy<DJI::M2006Traits<2>, decltype(pid_chain)>{pid_chain};
    auto dji_motor = std::make_unique<DJI::M2006<2, decltype(pid_chain)>>(
        driver, dji_policy);
    (void)dji_motor->setPosRef(1 * rev);
    (void)dji_motor->enable();

    std::array<std::unique_ptr<IMotor>, 2> motors;
    motors[0] = std::move(dm_motor);
    motors[1] = std::move(dji_motor);

    auto print_status = [&](size_t index) {
        if (!motors[index]) {
            return;
        }
        if (auto status = motors[index]->getStatusVariant(); status) {
            std::cout << "Motor j" << (index + 1) << " status:\n";
            std::visit(
                [](const auto &payload) {
                    using StatusType = std::decay_t<decltype(payload)>;
                    if constexpr (std::is_same_v<StatusType, DM::DmStatus>) {
                        std::cout << "  DM ID: " << int(payload.ID)
                                  << ", position: " << payload.position << '\n';
                    } else {
                        std::cout << "  DJI angular: " << payload.angular
                                  << '\n';
                    }
                },
                *status);
        } else {
            std::cerr << "Failed to read j" << (index + 1)
                      << " status: " << status.error().message << '\n';
        }
    };

    std::cout << "Commands: j1/j2/status/exit\n";
    std::string command;
    while (true) {
        std::cout << "> ";
        if (!(std::cin >> command)) {
            break;
        }
        if (command == "exit") {
            break;
        }
        if (command == "status") {
            for (size_t i = 0; i < motors.size(); ++i) {
                print_status(i);
            }
            continue;
        }
        if (command == "j1" || command == "j2") {
            const size_t index = command == "j1" ? 0 : 1;
            float rotations = 0.0f;
            std::cout << "New ref (rev): ";
            if (!(std::cin >> rotations)) {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(),
                                '\n');
                continue;
            }
            if (auto result = motors[index]->setPosRef(rotations * rev);
                !result) {
                std::cerr << "Failed to update " << command << ": "
                          << result.error().message << '\n';
            }
            continue;
        }
        std::cout << "Unknown command.\n";
    }

    return 0;
}
