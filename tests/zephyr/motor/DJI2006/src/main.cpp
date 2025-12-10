#include <OneMotor/Motor/DJI/DjiMotor.hpp>
#include <zephyr/logging/log.h>
#include <zephyr/debug/cpu_load.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);
const device* can_device = DEVICE_DT_GET(DT_NODELABEL(can1));
using OneMotor::Control::PID_Params;
using OneMotor::Control::PIDChain;
using OneMotor::Control::createPIDChain;
using OneMotor::Motor::DJI::PIDFeatures;
using OneMotor::Motor::DJI::DjiMotor;
using OneMotor::Motor::DJI::createDjiMotor;
using OneMotor::Control::Positional;
using OneMotor::Can::CanDriver;
using OneMotor::Motor::DJI::M2006Traits;

static constexpr PID_Params<float> POS_DEFAULT_PARAMS{
    .Kp = 3,
    .Ki = 0.1,
    .Kd = 0,
    .MaxOutput = 20000,
    .Deadband = 50,
    .IntegralLimit = 1000,
};
static constexpr PID_Params<float> ANG_DEFAULT_PARAMS{
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
    const auto pid_chain = createPIDChain()
                          .add<Positional, float, PIDFeatures>(POS_DEFAULT_PARAMS)
                          .add<Positional, float, PIDFeatures>(ANG_DEFAULT_PARAMS)
                          .build();
    auto m1 = createDjiMotor<M2006Traits, 1>(driver, pid_chain);
    m1.setPosRef(10000);
    m1.setAngRef(100);

    (void)m1.enable();

    while (true)
    {
        uint32_t load = cpu_load_get(false);
        LOG_INF("cpu: %u.%u%%", load / 10, load % 10);
        k_sleep(K_MSEC(500));
    }
    return 0;
}
