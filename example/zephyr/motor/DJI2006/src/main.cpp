#include <one/motor/dji/DjiMotor.hpp>
#include <zephyr/logging/log.h>
#include <zephyr/debug/cpu_load.h>
#include <one/PID/PidParams.hpp>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);
const device* can_device = DEVICE_DT_GET(DT_NODELABEL(can1));
using one::motor::dji::M2006;
using one::motor::dji::M3508;
using one::can::CanDriver;

using one::pid::PidParams;

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
using namespace one::motor::units::literals;

int main()
{
    LOG_INF("Starting...");
    LOG_INF("Init CanDriver...");
    CanDriver driver(can_device);
    LOG_INF("Creating motor instance...");
    M2006 m1(driver, {1, one::motor::dji::PosAngMode{POS_DEFAULT_PARAMS, ANG_DEFAULT_PARAMS}});
    M2006 m2(driver, {2, one::motor::dji::PosAngMode{POS_DEFAULT_PARAMS, ANG_DEFAULT_PARAMS}});
    M2006 m3(driver, {3, one::motor::dji::PosAngMode{POS_DEFAULT_PARAMS, ANG_DEFAULT_PARAMS}});
    M2006 m4(driver, {4, one::motor::dji::PosAngMode{POS_DEFAULT_PARAMS, ANG_DEFAULT_PARAMS}});
    (void)m1.setPosUnitRef(0 * rev);
    (void)m2.setPosUnitRef(0 * rev);
    (void)m3.setPosUnitRef(0 * rev);
    (void)m4.setPosUnitRef(0 * rev);

    (void)m1.enable();
    (void)m2.enable();
    (void)m3.enable();
    (void)m4.enable();

    while (true)
    {
        uint32_t load = cpu_load_get(false);
        LOG_INF("cpu: %u.%u%%", load / 10, load % 10);
        k_sleep(K_MSEC(500));
    }
    return 0;
}
