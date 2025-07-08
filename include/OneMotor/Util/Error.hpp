#ifndef ERROR_HPP
#define ERROR_HPP

#include <string>
#include <utility>

namespace OneMotor
{
    enum class ErrorCode
    {
        CanDriverInternalError,

        DJIMotorManagerError,
    };

    struct Error
    {
        Error(const ErrorCode c, std::string msg): message(std::move(msg)), code(c)
        {
        }

        std::string message;
        ErrorCode code;
    };
}

#endif //ERROR_HPP
