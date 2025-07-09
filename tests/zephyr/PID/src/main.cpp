#include <OneMotor/Control/PID.hpp>
#include <cmath>
#include <zephyr/tc_util.h>
#include <zephyr/ztest.h>
#include <zephyr/kernel.h>

using namespace OneMotor::Control;

// Test basic positional PID controller
ZTEST(pid_controller, test_positional_basic)
{
    // Create a simple positional PID controller
    PID_Params<float> params;
    params.Kp = 1.0f;
    params.Ki = 0.1f;
    params.Kd = 0.05f;

    PIDController pid(params);

    float measurement = 0.0f;
    float output = pid.compute(10.0f, measurement);
    zassert_true(output > 0, "Positional PID should output positive value");

    // Perform more iterations to observe if it eventually approaches target value
    for (int i = 0; i < 50; i++)
    {
        measurement += output * 0.1f; // Simple system response
        output = pid.compute(10.0f, measurement);
        TC_PRINT("measure: %f, output: %f\n", static_cast<double>(measurement), static_cast<double>(output));
    }

    // Check if final result is close to target value
    zassert_true(std::abs(measurement - 10.0f) < 5.0f, "PID controller should bring system close to target value");

    // Check if final output is stable
    const float last_output = output;
    measurement += output * 0.1f;
    output = pid.compute(10.0f, measurement);
    zassert_true(std::abs(output - last_output) < 0.5f, "PID controller should eventually stabilize");
}

// Test derivative on measurement feature
ZTEST(pid_controller, test_derivative_on_measurement)
{
    // Create PID controller with derivative on measurement
    PID_Params<float> params;
    params.Kp = 1.0f;
    params.Ki = 0.0f;
    params.Kd = 1.0f;

    PIDController<Positional, float, WithDerivativeOnMeasurement> pid(params);

    TC_PRINT("first output: %f\n", static_cast<double>(pid.compute(0.0f, 0.0f)));
    k_sleep(K_MSEC(1));

    // Setpoint sudden change
    const float output = pid.compute(10.0f, 0.0f);
    TC_PRINT("second output: %f\n", static_cast<double>(output));

    // Compare with regular PID
    PIDController regular_pid(params);
    TC_PRINT("regular first output: %f\n", static_cast<double>(regular_pid.compute(0.0f, 0.0f)));
    k_sleep(K_MSEC(1));
    const float regular_output = regular_pid.compute(10.0f, 0.0f);
    TC_PRINT("regular second output: %f\n", static_cast<double>(regular_output));

    // Derivative on measurement should produce smaller output jump
    zassert_true(std::abs(output) < std::abs(regular_output), "Derivative on measurement should reduce output spikes");
}

// Test integer type PID
ZTEST(pid_controller, test_integer_pid)
{
    // Create PID controller using integer type
    PID_Params<int> params;
    params.Kp = 1;
    params.Ki = 0;
    params.Kd = 0;

    PIDController pid(params);

    int output = pid.compute(100, 0);
    zassert_equal(output, 100, "Integer type PID calculation error");

    output = pid.compute(-50, 0);
    zassert_equal(output, -50, "Integer type PID calculation error");
}

// Test PID with output filter
ZTEST(pid_controller, test_output_filter)
{
    // Create PID controller with output filter
    PID_Params<float> params;
    params.Kp = 1.0f;
    params.Ki = 0.0f;
    params.Kd = 0.0f;
    params.OutputFilterRC = 0.1f;

    PIDController<Positional, float, WithOutputFilter> pid(params);

    // Step change test
    const float output1 = pid.compute(10.0f, 0.0f);
    k_sleep(K_MSEC(10));
    const float output2 = pid.compute(10.0f, 0.0f);

    // Filter effect should make second output closer to setpoint
    zassert_true(std::abs(output2 - 10.0f) < std::abs(output1 - 10.0f), "Filter should smooth output");
}

// Test basic incremental PID controller
ZTEST(pid_controller, test_incremental_basic)
{
    // Create a simple incremental PID controller
    PID_Params<float> params;
    params.Kp = 1.0f;
    params.Ki = 0.1f;
    params.Kd = 0.05f;

    PIDController<Incremental, float> pid(params);

    float measurement = 0.0f;
    float output = pid.compute(10.0f, measurement);
    zassert_true(output > 0, "Incremental PID should output positive value");

    // Perform more iterations to observe if it eventually approaches target value
    for (int i = 0; i < 50; i++)
    {
        measurement += output * 0.1f; // Simple system response
        output = pid.compute(10.0f, measurement);
        TC_PRINT("Incremental PID: measure: %f, output: %f\n", static_cast<double>(measurement),
                 static_cast<double>(output));
    }

    // Check if final result is close to target value
    zassert_true(std::abs(measurement - 10.0f) < 5.0f, "Incremental PID controller should bring system close to target value");
}

// Compare positional vs incremental PID response
ZTEST(pid_controller, test_positional_vs_incremental)
{
    PID_Params<float> params;
    params.Kp = 1.0f;
    params.Ki = 0.1f;
    params.Kd = 0.05f;

    PIDController pos_pid(params);
    PIDController<Incremental, float> inc_pid(params);

    float pos_measure = 0.0f;
    float inc_measure = 0.0f;

    for (int i = 0; i < 20; i++)
    {
        const float pos_output = pos_pid.compute(10.0f, pos_measure);
        const float inc_output = inc_pid.compute(10.0f, inc_measure);
        pos_measure += pos_output * 0.1f;
        inc_measure += inc_output * 0.1f;
        TC_PRINT("step %d - positional: %f, incremental: %f\n", i, static_cast<double>(pos_measure),
                 static_cast<double>(inc_measure));
    }
    // Incremental usually has smoother output changes
    zassert_true(true, "Only for displaying comparison results of two algorithms");
}

// Test incremental PID with output limit
ZTEST(pid_controller, test_incremental_with_output_limit)
{
    // Create incremental PID controller with output limit
    PID_Params<float> params;
    params.Kp = 10.0f;
    params.Ki = 0.0f;
    params.Kd = 0.0f;
    params.MaxOutput = 5.0f;

    PIDController<Incremental, float, WithOutputLimit> pid(params);

    // Set large error, should trigger limiting
    const float output = pid.compute(10.0f, 0.0f);

    // Output should not exceed limit
    zassert_true(output <= 5.0f, "Incremental PID should respect output limit");
    zassert_equal(output, 5.0f, "For large error, output should reach upper limit");
}

// Test PID controller with deadband
ZTEST(pid_controller, test_deadband)
{
    // Create PID controller with deadband, deadband value set to 1.0
    PID_Params<float> params;
    params.Kp = 1.0f;
    params.Ki = 0.1f;
    params.Kd = 0.05f;
    params.Deadband = 1.0f;

    PIDController<Positional, float, WithDeadband> pid(params);

    // Test 1: error smaller than deadband
    float output = pid.compute(0.5f, 0.0f); // error is 0.5, smaller than deadband 1.0
    zassert_equal(output, 0.0f, "When error is smaller than deadband, output should be 0");

    // Test 2: error equal to deadband
    auto prev_output = output;
    output = pid.compute(1.0f, 0.0f); // error is 1.0, equal to deadband
    zassert_equal(output, prev_output, "When error equals deadband, output should be previous output");

    // Test 3: error larger than deadband
    output = pid.compute(2.0f, 0.0f); // error is 2.0, larger than deadband 1.0
    zassert_true(output > 0.0f, "When error is larger than deadband, should have non-zero output");
    TC_PRINT("%f\n", static_cast<double>(output));

    // Test 4: negative error, smaller than deadband
    prev_output = output;
    output = pid.compute(-0.5f, 0.0f); // error is -0.5, absolute value smaller than deadband
    zassert_equal(output, prev_output, "When negative error is smaller than deadband, output should be previous output");

    // Test 5: negative error, larger than deadband
    output = pid.compute(-2.0f, 0.0f); // error is -2.0, absolute value larger than deadband
    zassert_true(output < 0.0f, "When negative error is larger than deadband, should have non-zero negative output");
    TC_PRINT("%f\n", static_cast<double>(output));

    // Test 6: error change from inside deadband to outside deadband
    pid.compute(0.5f, 0.0f); // first set a value inside deadband
    output = pid.compute(5.0f, 0.0f); // then suddenly change to outside deadband
    zassert_true(output > 0.0f, "Should have correct response when changing from inside to outside deadband");
    TC_PRINT("%f\n", static_cast<double>(output));
}

// Test PID with integral limit
ZTEST(pid_controller, test_integral_limit)
{
    // Create PID controller with integral limit
    PID_Params<float> params;
    params.Kp = 0.1f;
    params.Ki = 1.0f;
    params.Kd = 0.0f;
    params.IntegralLimit = 5.0f;

    PIDController<Positional, float, WithIntegralLimit> pid(params);

    // Apply constant error to build up integral
    float measurement = 0.0f;
    for (int i = 0; i < 100; i++)
    {
        const float output = pid.compute(10.0f, measurement);
        measurement += output * 0.01f; // Very small system response to maintain error
        if (i % 10 == 0)
        {
            TC_PRINT("step %d: measure: %f, output: %f\n", i, static_cast<double>(measurement), static_cast<double>(output));
        }
    }

    zassert_true(true, "Integral limit test completed");
}

// Test PID reset functionality
ZTEST(pid_controller, test_reset)
{
    // Create PID controller
    PID_Params<float> params;
    params.Kp = 1.0f;
    params.Ki = 0.1f;
    params.Kd = 0.05f;

    PIDController pid(params);

    // Run several calculations to accumulate state
    float measurement = 0.0f;
    for (int i = 0; i < 5; i++)
    {
        const float output = pid.compute(10.0f, measurement);
        measurement += output * 0.1f;
    }

    // Record current output
    const float before_reset = pid.compute(10.0f, measurement);

    // Reset controller
    pid.reset();

    // Should get different output after reset
    const float after_reset = pid.compute(10.0f, measurement);
    TC_PRINT("before: %f, after: %f\n",
             static_cast<double>(before_reset), static_cast<double>(after_reset));

    // After reset, the output should be different (usually larger for first computation)
    zassert_true(after_reset != before_reset, "Output should be different after reset");
}

// Test latency for PID compute with all optional features
ZTEST(pid_controller, test_latency_all_features)
{
    // Setup PID parameters with all features enabled
    PID_Params<float> params;
    params.Kp = 1.0f;
    params.Ki = 0.1f;
    params.Kd = 0.05f;
    params.Deadband = 0.1f;
    params.IntegralLimit = 10.0f;
    params.DerivativeFilterRC = 0.01f;
    params.OutputFilterRC = 0.05f;
    params.MaxOutput = 100.0f;

    PIDController<Positional, float,
                   WithDeadband,
                   WithIntegralLimit,
                   WithDerivativeOnMeasurement,
                   WithDerivativeFilter,
                   WithOutputFilter,
                   WithOutputLimit> pid(params);

    constexpr int iterations = 1000;
    // Warm up
    pid.compute(10.0f, 5.0f);

    const uint32_t start = k_cycle_get_32();
    for (int i = 0; i < iterations; i++) {
        pid.compute(10.0f, 5.0f);
    }
    const uint32_t end = k_cycle_get_32();
    const uint32_t total = end - start;
    const auto avg_cycles = static_cast<double>(total) / iterations;
    TC_PRINT("Total cycles: %u, avg cycles per compute: %f\n", total, avg_cycles);
}

ZTEST_SUITE(pid_controller, nullptr, NULL, NULL, NULL, NULL);

