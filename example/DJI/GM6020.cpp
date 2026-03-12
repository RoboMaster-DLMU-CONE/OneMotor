
#include <iostream>
#include <one/PID/PidChain.hpp>
#include <one/PID/PidConfig.hpp>
#include <one/PID/PidParams.hpp>
#include <string>

#include <one/motor/dji/DjiMotor.hpp>

using one::can::CanDriver;
using one::motor::dji::GM6020_Current;
using one::motor::dji::GM6020_Voltage;
using one::motor::dji::M2006;
using one::motor::dji::PIDFeatures;
using one::motor::dji::PosAngMode;
using one::pid::PidChain;
using one::pid::PidConfig;
using one::pid::PidParams;

using namespace one::motor::units::literals;
int main() {

    CanDriver driver("can0");
    GM6020_Voltage m1(driver,
                      {.id = 4, .mode = one::motor::dji::MITMode{2000, 10}});

    m1.setPosUnitRef(1 * rev);
    m1.setAngUnitRef(0.5 * rad / s);
    m1.setTorRef(15);
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
