#include "OneMotor/Util/Panic.hpp"
#include <zephyr/kernel.h>

namespace OneMotor
{
    void panic(const std::string&& message)
    {
        k_panic();
    }
}