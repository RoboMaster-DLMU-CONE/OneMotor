#include <stdexcept>

#include "OneMotor/Util/Panic.hpp"

namespace OneMotor::Util
{
    void om_panic(const std::string&& message)
    {
        throw std::runtime_error(message);
    }
}
