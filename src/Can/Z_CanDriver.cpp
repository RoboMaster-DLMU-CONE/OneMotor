#include "autoconf.h"
#include "OneMotor/Can/CanDriver.hpp"
#include "OneMotor/Util/Panic.hpp"

using tl::unexpected;
using enum OneMotor::ErrorCode;

namespace OneMotor::Can
{
    void rx_callback_entry(const device* dev, can_frame* frame, void* user_data)
    {
        if (const auto callback = reinterpret_cast<CanDriver::CallbackFunc*>(user_data))
        {
            (*callback)(std::move(*reinterpret_cast<CanFrame*>(frame)));
        }
    }

    CanDriver::CanDriver(const device* device): can_dev(device)
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
            return unexpected(Error{CanDriverInternalError, strerror(ret)});
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
        return {};
    }

    tl::expected<void, Error> CanDriver::registerCallback(const std::set<size_t>& can_ids, CallbackFunc func)
    {
        for (const auto& id : can_ids)
        {
            if (id > 2048) return unexpected(Error{CanDriverInternalError, "specified CAN ID has exceeded 2048."});
            if (auto it = filters.find(id); it != filters.end())
            {
                can_remove_rx_filter(can_dev, it->second.second);
                filters.erase(it);
                callbacks.erase(id);
            }
            callbacks[id] = func;
            const auto filter = can_filter{
                .id = id,
                .mask = CAN_STD_ID_MASK,
                .flags = 0U,
            };
            int filter_id = can_add_rx_filter(can_dev, rx_callback_entry, &func, &filter);
            if (filter_id < 0)
            {
                return unexpected(Error{CanDriverInternalError, strerror(-filter_id)});
            }
            filters[id] = {filter, filter_id};
        }
        return {};
    }
}
