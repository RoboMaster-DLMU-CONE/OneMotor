#include "autoconf.h"
#include "OneMotor/Can/CanDriver.hpp"
#include "OneMotor/Util/Panic.hpp"

using tl::unexpected;
using enum OneMotor::ErrorCode;

namespace OneMotor::Can
{
    void CanDriver::rx_callback_entry(const device* dev, can_frame* frame, void* func)
    {
        if (func && *func)
        {
            func(std::move(*frame));
        }
    }

    CanDriver::CanDriver(device* device): can_dev(device)
    {
        if (!device_is_ready(can_dev))
        {
            panic("CAN Device not ready.");
        }
    }

    CanDriver::~CanDriver() = default;

    tl::expected<void, Error> CanDriver::open()
    {
        if (const auto ret = can_start(can_dev); ret == -EIO)
        {
            return tl::unexpected(Error{CanDriverInternalError, strerror(ret)});
        }
        return {};
    }

    tl::expected<void, Error> CanDriver::close()
    {
        if (const auto ret = can_stop(can_dev); ret == -EIO)
        {
            return unexpected(Error{CanDriverInternalError, strerror(ret)});
        }
        return {};
    }

    tl::expected<void, Error> CanDriver::send(const CanFrame& frame)
    {
        if (const auto ret = can_send(can_dev, reinterpret_cast<const can_frame*>(&frame), K_MSEC(1), nullptr, nullptr);
            ret != 0)
        {
            return unexpected(Error{CanDriverInternalError, strerror(ret)});
        }
    }

    tl::expected<void, Error> CanDriver::registerCallback(const std::set<size_t>& can_ids, const CallbackFunc& func)
    {
        for (const auto& id : can_ids)
        {
        }
    }
}
