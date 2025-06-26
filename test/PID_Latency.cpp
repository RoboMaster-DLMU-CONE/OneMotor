#include <iostream>
#include <vector>
#include <numeric>
#include <chrono>
#include "OneMotor/Control/PID.hpp"

using PID_Params = OneMotor::Control::PID_Params<float>;
using PIDController = OneMotor::Control::PIDController<
    OneMotor::Control::Positional,
    float,
    OneMotor::Control::WithDeadband,
    OneMotor::Control::WithIntegralLimit,
    OneMotor::Control::WithDerivativeOnMeasurement,
    OneMotor::Control::WithDerivativeFilter,
    OneMotor::Control::WithOutputFilter,
    OneMotor::Control::WithOutputLimit
>;

int main()
{
    // 1. 設定 PID 控制器
    PID_Params params{};
    params.Kp = 1.2f;
    params.Ki = 0.5f;
    params.Kd = 0.1f;
    params.MaxOutput = 1000.0f;
    params.IntegralLimit = 500.0f;
    params.Deadband = 0.01f;

    PIDController pid(params);

    constexpr int num_iterations = 10000;
    std::vector<double> durations_us;
    durations_us.reserve(num_iterations);

    float measure = 0.0f;

    std::cout << "Running PID compute latency test for " << num_iterations << " iterations..." << std::endl;

    for (int i = 0; i < num_iterations; ++i)
    {
        constexpr float ref = 100.0f;
        measure += 0.1f;

        auto start = std::chrono::high_resolution_clock::now();
        pid.compute(ref, measure);
        auto end = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::micro> duration = end - start;
        durations_us.push_back(duration.count());
    }

    // 3. 計算並輸出平均耗時
    if (!durations_us.empty())
    {
        const double total_duration = std::accumulate(durations_us.begin(), durations_us.end(), 0.0);
        const double average_duration = total_duration / durations_us.size();

        std::cout << "Average PID::compute() execution time: " << average_duration << " microseconds." << std::endl;
    }
    else
    {
        std::cout << "No test iterations were run." << std::endl;
    }

    return 0;
}
