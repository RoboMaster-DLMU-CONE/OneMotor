#include <OF/drivers/output/status_leds.h>
#include <OneMotor/Motor/DJI/M3508.hpp>
#include <zephyr/logging/log.h>
#include <zephyr/debug/cpu_load.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);
const device* can_device = DEVICE_DT_GET(DT_NODELABEL(can1));
using OneMotor::Control::PID_Params;
using OneMotor::Motor::DJI::PIDController;
using OneMotor::Control::Positional;
using OneMotor::Can::CanDriver;
using OneMotor::Motor::DJI::M3508;
using enum OneMotor::Motor::DJI::MotorMode;

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
    const device* status_led_dev = DEVICE_DT_GET(DT_NODELABEL(status_leds));
    if (!device_is_ready(status_led_dev))
    {
        LOG_ERR("LED not ready\n");
        return -1;
    }
    const auto led_api = static_cast<const status_leds_api*>(status_led_dev->api);
    led_api->set_heartbeat(status_led_dev);

    LOG_INF("Starting...");
    LOG_INF("Init CanDriver...");
    CanDriver driver(can_device);
    LOG_INF("Creating motor instance...");
    M3508<1, Position> m3508(driver, POS_DEFAULT_PARAMS, ANG_DEFAULT_PARAMS);
    m3508.setPosRef(10000);
    m3508.setAngRef(100);

    (void)m3508.enable();

    while (true)
    {
        uint32_t load = cpu_load_get(false);
        LOG_INF("cpu: %u.%u%%", load / 10, load % 10);
        k_sleep(K_MSEC(500));
    }
    return 0;
}
