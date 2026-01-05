#include <bit>
#include <type_traits>
#include <utility>

#include <OneMotor/Can/CanDriver.hpp>
#include <OneMotor/Can/CanFrame.hpp>
#include <concepts>

using tl::unexpected;
using enum OneMotor::ErrorCode;

namespace OneMotor::Can {
CanDriver::CanDriver(std::string interface_name)
    : interface_name(std::move(interface_name)),
      interface(this->interface_name) {}

CanDriver::~CanDriver() = default;

tl::expected<bool, Error> CanDriver::is_open() {
    return interface.is_up().map_error([&](const auto &e) {
        return Error({CanDriverInternalError, e.message});
    });
}

tl::expected<void, Error> CanDriver::open() {
    return interface.up().map_error([&](const auto &e) {
        return Error({CanDriverInternalError, e.message});
    });
}

tl::expected<void, Error> CanDriver::close() {
    return interface.down().map_error([&](const auto &e) {
        return Error({CanDriverInternalError, e.message});
    });
}

tl::expected<void, Error> CanDriver::send(const CanFrame &frame) {
    return interface.send(reinterpret_cast<const can_frame &>(frame))
        .map_error([&](const auto &e) {
            return Error({CanDriverInternalError, e.message});
        });
}

} // namespace OneMotor::Can
