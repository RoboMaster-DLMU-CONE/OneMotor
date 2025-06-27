#ifndef J4310_HPP
#define J4310_HPP
#include "J4310Frame.hpp"
#include "OneMotor/Can/CanDriver.hpp"
#include "OneMotor/Util/SpinLock.hpp"

namespace OneMotor::Motor::DM
{
    class J4310
    {
        using Result = std::expected<void, std::string>;

    public:
        J4310() = delete;
        explicit J4310(Can::CanDriver& driver, uint16_t canId, uint16_t masterId);
        Result enable();
        Result disable();
        Result setZeroPosition();
        Result cleanError();
        Result MITControl(float position, float velocity, float torque, float kp, float kd);
        Result posVelControl(float position, float velocity);
        Result velControl(float velocity);
        std::expected<J4310Status, std::string> getStatus();

    private:
        J4310Status status_{};
        Util::SpinLock lock_;
        Can::CanDriver& driver_;
        uint16_t canId_;
        uint16_t masterId_;
    };
}

#endif //J4310_HPP
