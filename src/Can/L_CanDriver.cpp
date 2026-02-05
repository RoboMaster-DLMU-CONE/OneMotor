#include <bit>
#include <type_traits>
#include <utility>

#include <OneMotor/Can/CanDriver.hpp>
#include <OneMotor/Can/CanFrame.hpp>
#include <OneMotor/Util/Panic.hpp>
#include <concepts>

using enum OneMotor::ErrorCode;

namespace OneMotor::Can {
CanDriver::CanDriver(std::string interface_name,
                     const std::optional<uint8_t> &cpu_core_opt) {
    if (auto result = init(std::move(interface_name), cpu_core_opt); !result) {
        panic(result.error().message);
    }
}

CanDriver::~CanDriver() = default;
tl::expected<void, Error>
CanDriver::init(std::string interface_name,
                const std::optional<uint8_t> &cpu_core_opt) {
    if (m_initialized) {
        return tl::make_unexpected(Error{ErrorCode::CanDriverAlreadyInitialized,
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

} // namespace OneMotor::Can
