#include <ankerl/unordered_dense.h>
#include <OneMotor/Can/CanDriver.hpp>
#include <OneMotor/Util/CCM.h>
#include <OneMotor/Util/Panic.hpp>
#include <OneMotor/Util/DtcmAllocator.hpp>
#include <memory>
using tl::unexpected;
using enum OneMotor::ErrorCode;

namespace OneMotor::Can
{
    namespace
    {
        void tx_complete(const device*, int, void*)
        {
        }
    }

    template <typename T>
    using Hash = std::hash<T>;
    template <typename K, typename V>
    using FastMap = std::unordered_map<K, V, Hash<K>, std::equal_to<K>, DtcmAllocator<std::pair<const K, V>>>;

    using CallbackFunc = std::function<void(CanFrame)>;
    using Callbacks = FastMap<uint16_t, CallbackFunc>;
    using Filters = FastMap<uint16_t, std::pair<can_filter, int>>;
    OM_CCM_ATTR FastMap<const device*, Callbacks> g_callbacks;
    OM_CCM_ATTR FastMap<const device*, Filters> g_filters;

    void rx_callback_entry(const device* dev, can_frame* frame, void* user_data)
    {
        if (const auto callback = static_cast<CallbackFunc*>(user_data))
        {
            (*callback)(std::bit_cast<CanFrame>(*frame));
        }
    }

    CanDriver::CanDriver(const device* device)
    {
        if (auto result = init(device); !result)
        {
            panic(result.error().message);
        }
    }

    CanDriver::~CanDriver() = default;

    tl::expected<void, Error> CanDriver::init(const device* device)
    {
        if (m_initialized)
        {
            return unexpected(
                Error{CanDriverAlreadyInitialized, "CanDriver already initialized"});
        }
        if (device == nullptr || !device_is_ready(device))
        {
            return unexpected(Error{
                CanDriverInternalError,
                "CAN Device not ready"
            });
        }
        can_dev = device;
        g_callbacks[can_dev] = {};
        g_filters[can_dev] = {};
        m_initialized = true;
        return {};
    }

    tl::expected<void, Error> CanDriver::open()
    {
        if (auto guard = ensureInitialized(); !guard)
        {
            return unexpected(guard.error());
        }
        if (const auto ret = can_start(can_dev); ret == -EIO)
        {
            return unexpected(Error{CanDriverInternalError, strerror(ret)});
        }
        return {};
    }

    tl::expected<void, Error> CanDriver::close()
    {
        if (auto guard = ensureInitialized(); !guard)
        {
            return unexpected(guard.error());
        }
        if (const auto ret = can_stop(can_dev); ret == -EIO)
        {
            return unexpected(Error{CanDriverInternalError, strerror(ret)});
        }
        return {};
    }

    tl::expected<void, Error> CanDriver::send(const CanFrame& frame)
    {
        if (auto guard = ensureInitialized(); !guard)
        {
            return unexpected(guard.error());
        }
        if (const auto ret =
                can_send(can_dev, reinterpret_cast<const can_frame*>(&frame),
                         K_MSEC(1), tx_complete, nullptr);
            ret != 0)
        {
            return unexpected(Error{CanDriverInternalError, strerror(ret)});
        }
        return {};
    }

    tl::expected<void, Error> CanDriver::registerCallbackImpl(
        const std::set<size_t>& can_ids,
        const std::function<void(CanFrame)>& func) const
    {
        if (auto guard = ensureInitialized(); !guard)
        {
            return unexpected(guard.error());
        }
        for (const auto& id : can_ids)
        {
            if (id > 2048)
                return unexpected(Error{
                    CanDriverInternalError,
                    "specified CAN ID has exceeded 2048."
                });
            auto& callbacks = g_callbacks.at(can_dev);
            auto& filters = g_filters.at(can_dev);
            if (auto it = filters.find(id); it != filters.end())
            {
                can_remove_rx_filter(can_dev, it->second.second);
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
            int filter_id = can_add_rx_filter(can_dev, rx_callback_entry,
                                              &callbacks[id], &filter);
            if (filter_id < 0)
            {
                callbacks.erase(id);
                return unexpected(
                    Error{CanDriverInternalError, strerror(-filter_id)});
            }
            filters[id] = {filter, filter_id};
        }
        return {};
    }
} // namespace OneMotor::Can
