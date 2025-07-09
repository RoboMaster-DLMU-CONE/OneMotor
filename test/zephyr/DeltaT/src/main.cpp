#include <vector>
#include <zephyr/kernel.h>
#include <OneMotor/Util/DeltaT.hpp>
#include <zephyr/ztest.h>
ZTEST(delta_t, test_basic_timing)
{
    OneMotor::DeltaT<> timer;
    const float delta1 = timer.getDeltaMS();
    zassert_true(delta1 >= static_cast<float>(0.0), "DeltaT time should not be negative.");
    k_sleep(K_MSEC(100));
    const float delta2 = timer.getDeltaMS();
    zassert_true(delta2 >= static_cast<float>(100.0), "DeltaT time should >= 100ms.");
}

ZTEST(delta_t, test_reset)
{
    OneMotor::DeltaT<> timer;

    k_sleep(K_MSEC(100));
    timer.reset();

    const double delta2 = timer.getDeltaMS();
    zassert_true(delta2 < 2.0, "DeltaT time should be small after resetting.");
}

ZTEST(delta_t, test_different_types)
{
    {
        OneMotor::DeltaT<double> timer;
        k_sleep(K_MSEC(100));
        const double delta = timer.getDeltaMS();
        zassert_true(delta >= 100.0, "Double type DeltaT time should >= 100ms");
    }

    {
        OneMotor::DeltaT<int> timer;
        k_sleep(K_MSEC(100));
        const int delta = timer.getDeltaMS();
        zassert_true(delta >= 100, "Int type DeltaT should >= 100ms");
    }
}

ZTEST(delta_t, test_consecutive_calls)
{
    OneMotor::DeltaT<> timer;
    float total = 0.0;

    for (int i = 0; i < 4; i++)
    {
        k_sleep(K_MSEC(25));
        total += timer.getDeltaMS();
    }

    zassert_true(total >= 100.0f, "four times 25ms delay should >= 100ms.");
}

// 测试非零保证
ZTEST(delta_t, test_nonzero_guarantee)
{
    OneMotor::DeltaT<> timer;
    OneMotor::DeltaT<uint32_t> timer1;

    // 连续多次快速调用，验证返回值始终大于0
    std::vector<double> t_vector;
    std::vector<uint32_t> t_vector1;
    for (int i = 0; i < 10; i++)
    {
        t_vector.push_back(timer.getDeltaMS());
        t_vector1.push_back(timer1.getDeltaMS());
    }
    for (int i = 0; i < 10; i++)
    {
        const auto time = t_vector[i];
        const auto time1 = t_vector1[i];
        TC_PRINT("%f ms %d ms\n", time, time1);
        zassert_true(time != 0 && time1 != 0, "DeltaT time should >0.");
    }
}

ZTEST_SUITE(delta_t, NULL, NULL, NULL, NULL, NULL);
