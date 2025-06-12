#ifndef CANDRIVER_HPP
#define CANDRIVER_HPP

#include <string>
#include <functional>
#include <set>

#ifdef ONE_MOTOR_LINUX
#include <HyCAN/Interface/Interface.hpp>
#endif

using std::string, std::set, std::function;
using HyCAN::Interface;

namespace OneMotor::Can
{
    class CanFrame;

    class CanDriver
    {
    public:
        using CallbackFunc = function<void(CanFrame&&)>;
        explicit CanDriver(string interface_name);
        ~CanDriver();
        CanDriver(const CanDriver&) = delete;
        CanDriver(const CanDriver&&) = delete;
        CanDriver& operator=(const CanDriver&) = delete;
        CanDriver& operator=(CanDriver&&) = delete;
        bool open();
        bool close();
        bool send(const CanFrame& frame);
        void registerCallback(const set<size_t>& can_ids, const CallbackFunc& func);

    private:
        string interface_name;
#ifdef ONE_MOTOR_LINUX
        Interface interface;
#endif
    };
}


#endif //CANDRIVER_HPP
