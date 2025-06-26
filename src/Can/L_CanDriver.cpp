#include <utility>

#include "OneMotor/Can/CanDriver.hpp"
#include "OneMotor/Can/CanFrame.hpp"

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
        return interface.send(reinterpret_cast<const can_frame&>(frame));
    }

    CanDriver::Result CanDriver::registerCallback(const std::set<size_t>& can_ids, const CallbackFunc& func)
    {
        return interface.tryRegisterCallback<can_frame>(can_ids, [func](can_frame&& frame)
                                                        {
                                                            func(std::move(reinterpret_cast<CanFrame&>(frame)));
                                                        }
        );
    }
}
