#include <utility>

#include "one-motor/can/CanDriver.hpp"
#include "one-motor/can/CanFrame.hpp"

namespace OneMotor::Can
{
    CanDriver::CanDriver(string interface_name): interface_name(std::move(interface_name)),
                                                 interface(this->interface_name)
    {
    }

    CanDriver::~CanDriver() = default;

    bool CanDriver::open()
    {
        interface.up();
        return true;
    }

    bool CanDriver::close()
    {
        interface.down();
        return true;
    }

    bool CanDriver::send(const CanFrame& frame)
    {
        interface.send(frame);
        return true;
    }

    void CanDriver::registerCallback(const set<size_t>& can_ids, const CallbackFunc& func)
    {
        interface.tryRegisterCallback<CanFrame>(can_ids, func);
    }
}
