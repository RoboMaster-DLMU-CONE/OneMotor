#include <stdexcept>

#include "OneMotor/Util/Panic.hpp"

namespace OneMotor
{
    void panic(const char *message)
    {
        throw std::runtime_error(message);
    }
}
