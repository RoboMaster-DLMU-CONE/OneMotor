#include <ostream>

#include "one/utils/Panic.hpp"
#include <zephyr/kernel.h>

#include "zephyr/logging/log.h"
LOG_MODULE_REGISTER(OneMotor, CONFIG_LOG_DEFAULT_LEVEL);

namespace one::motor
{
    void panic(const char* message)
    {
        LOG_ERR("%s", message);
        k_panic();
    }
}
