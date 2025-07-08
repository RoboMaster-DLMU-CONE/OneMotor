#include <stdexcept>

#include "OneMotor/Util/Panic.hpp"

namespace OneMotor
{
    void panic(const std::string&& message)
    {
        throw std::runtime_error(message);
    }
}
