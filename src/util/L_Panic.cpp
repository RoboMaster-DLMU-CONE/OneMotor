#include <stdexcept>

#include "one-motor/util/Panic.hpp"

namespace OneMotor::Util
{
    void om_panic(const std::string&& message)
    {
        throw std::runtime_error(message);
    }
}
