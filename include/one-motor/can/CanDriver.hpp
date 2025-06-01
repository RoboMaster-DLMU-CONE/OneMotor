#ifndef CANDRIVER_HPP
#define CANDRIVER_HPP

#include <string>

#ifdef ONE_MOTOR_LINUX
#include <hycan/Interface/Interface.hpp>
#endif
#include "CanFrame.hpp"

using std::string;
using HyCAN::Interface;

namespace OneMotor::Can
{
    class CanDriver
    {
    public:
        explicit CanDriver(const string& interface_name);
        ~CanDriver();
        CanDriver(const CanDriver&) = delete;
        CanDriver(const CanDriver&&) = delete;
        CanDriver& operator=(const CanDriver&) = delete;
        CanDriver& operator=(CanDriver&&) = delete;
        bool open();
        bool close();
        bool send(const CanFrame& frame);

    private:
        string interface_name;
#ifdef ONE_MOTOR_LINUX
        Interface interface;
#endif
    };
}


#endif //CANDRIVER_HPP
