#include <ostream>

#include "OneMotor/Util/Panic.hpp"
#include <zephyr/kernel.h>

#include "zephyr/logging/log.h"
LOG_MODULE_REGISTER(OneMotor, CONFIG_LOG_DEFAULT_LEVEL);

namespace OneMotor
{
    void panic(const char *message)
    {
        LOG_ERR("%s", message);
        k_panic();
    }
}
