#include <iostream>
#include <one/motor/dji/DjiMotor.hpp>
#include <string>

using one::can::CanDriver;
using one::motor::dji::M3508;

using namespace one::motor::units::literals;
int main() {

    CanDriver driver("can0");

    M3508 m1(driver, {1, one::motor::dji::MITMode{80, 3}});

    m1.setPosUnitRef(2 * rev);
    m1.setTorRef(1.5);
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
