#include <utility>

#include "one-motor/can/CanDriver.hpp"
#include "one-motor/can/CanFrame.hpp"

namespace OneMotor::Can
{
    CanDriver::CanDriver(std::string interface_name): interface_name(std::move(interface_name)),
                                                      interface(this->interface_name)
    {
    }

    CanDriver::~CanDriver() = default;

    CanDriver::Result CanDriver::open()
    {
        return interface.up();
    }

    CanDriver::Result CanDriver::close()
    {
        return interface.down();
    }

    CanDriver::Result CanDriver::send(const CanFrame& frame)
    {
        return interface.send(frame);
    }

    CanDriver::Result CanDriver::registerCallback(const std::set<size_t>& can_ids, const CallbackFunc& func)
    {
        return interface.tryRegisterCallback<CanFrame>(can_ids, func);
    }
}
