#include <one/motor/Error.hpp>
#include <utility>

#include <one/can/CanDriver.hpp>
#include <one/can/CanFrame.hpp>
#include <one/utils/Panic.hpp>

using enum one::motor::ErrorCode;
using one::motor::Error;
namespace one::can {
CanDriver::CanDriver(std::string interface_name,
                     const std::optional<uint8_t> &cpu_core_opt) {
    if (auto result = init(std::move(interface_name), cpu_core_opt); !result) {
        motor::panic(result.error().message);
    }
}

CanDriver::~CanDriver() = default;
tl::expected<void, Error>
CanDriver::init(std::string interface_name,
                const std::optional<uint8_t> &cpu_core_opt) {
    if (m_initialized) {
        return tl::make_unexpected(Error{CanDriverAlreadyInitialized,
                                         "CanDriver already initialized"});
    }
    this->interface_name = std::move(interface_name);
    interface = std::make_unique<HyCAN::CANInterface>(this->interface_name,
                                                      cpu_core_opt);
    m_initialized = true;
    return {};
}

tl::expected<bool, Error> CanDriver::is_open() {
    if (auto guard = ensureInitialized(); !guard) {
        return tl::make_unexpected(guard.error());
    }
    return interface->is_up().map_error([&](const auto &) {
        return Error{CanDriverInternalError, "HyCAN interface is_up failed"};
    });
}

tl::expected<void, Error> CanDriver::open() {
    if (auto guard = ensureInitialized(); !guard) {
        return tl::make_unexpected(guard.error());
    }
    return interface->up().map_error([&](const auto &) {
        return Error{CanDriverInternalError, "HyCAN interface up failed"};
    });
}

tl::expected<void, Error> CanDriver::close() {
    if (auto guard = ensureInitialized(); !guard) {
        return tl::make_unexpected(guard.error());
    }
    return interface->down().map_error([&](const auto &) {
        return Error{CanDriverInternalError, "HyCAN interface down failed"};
    });
}

tl::expected<void, Error> CanDriver::send(const CanFrame &frame) {
    if (auto guard = ensureInitialized(); !guard) {
        return tl::make_unexpected(guard.error());
    }
    return interface->send(reinterpret_cast<const can_frame &>(frame))
        .map_error([&](const auto &) {
            return Error{CanDriverInternalError, "HyCAN send failed"};
        });
}

} // namespace one::can
