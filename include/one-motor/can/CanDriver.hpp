#ifndef CANDRIVER_HPP
#define CANDRIVER_HPP

#include <string>
#include <functional>
#include <set>
#include <expected>
#include "CanFrame.hpp"

#ifdef ONE_MOTOR_LINUX
#include <HyCAN/Interface/Interface.hpp>
#endif


namespace OneMotor::Can
{
    class CanDriver
    {
    public:
        using CallbackFunc = std::function<void(CanFrame&&)>;
        using Result = std::expected<void, std::string>;
        explicit CanDriver(std::string interface_name);
        ~CanDriver();
        CanDriver(const CanDriver&) = delete;
        CanDriver(const CanDriver&&) = delete;
        CanDriver& operator=(const CanDriver&) = delete;
        CanDriver& operator=(CanDriver&&) = delete;
        Result open();
        Result close();
        Result send(const CanFrame& frame);
        Result registerCallback(const std::set<size_t>& can_ids, const CallbackFunc& func);

    private:
        std::string interface_name;
#ifdef ONE_MOTOR_LINUX
        HyCAN::Interface interface;
#endif
    };
}


#endif //CANDRIVER_HPP
