#include <OneMotor/Motor/DJI/DjiMotor.hpp>
#include <zephyr/logging/log.h>
#include <zephyr/debug/cpu_load.h>
#include <one/PID/PidChain.hpp>
#include <one/PID/PidConfig.hpp>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);
const device* can_device = DEVICE_DT_GET(DT_NODELABEL(can1));
using OneMotor::Motor::DJI::PIDFeatures;
using OneMotor::Motor::DJI::M2006;
using OneMotor::Can::CanDriver;

using one::pid::PidParams;
using one::pid::PidConfig;
using one::pid::PidChain;

static constexpr PidParams<> POS_DEFAULT_PARAMS{
    .Kp = 3,
    .Ki = 0.1,
    .Kd = 0,
    .MaxOutput = 20000,
    .Deadband = 50,
    .IntegralLimit = 1000,
};
static constexpr PidParams<> ANG_DEFAULT_PARAMS{
    .Kp = 0.8,
    .Ki = 0.05,
    .Kd = 0.1,
    .MaxOutput = 8000,
    .Deadband = 10,
    .IntegralLimit = 100,
};

int main()
{
    LOG_INF("Starting...");
    LOG_INF("Init CanDriver...");
    CanDriver driver(can_device);
    LOG_INF("Creating motor instance...");
    constexpr auto conf1 =
        PidConfig<one::pid::Positional, float, PIDFeatures>(POS_DEFAULT_PARAMS);
    constexpr auto conf2 =
        PidConfig<one::pid::Positional, float, PIDFeatures>(ANG_DEFAULT_PARAMS);
    auto pid_chain = PidChain(conf1, conf2);

    M2006<1, decltype(pid_chain)> m1(driver, pid_chain);
    (void)m1.setPosRef(2 * rev);

    (void)m1.enable();

    while (true)
    {
        uint32_t load = cpu_load_get(false);
        LOG_INF("cpu: %u.%u%%", load / 10, load % 10);
        k_sleep(K_MSEC(500));
    }
    return 0;
}
