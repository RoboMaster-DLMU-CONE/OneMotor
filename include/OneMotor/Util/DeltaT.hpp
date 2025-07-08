/**
 * @file DeltaT.hpp
 * @brief 提供一个工具类来计算两次调用之间的时间差。
 * @author MoonFeather
 * @date 2025-06-26
 */
#ifndef DELTAT_HPP
#define DELTAT_HPP

#ifdef ONE_MOTOR_LINUX
#include <chrono>
#endif

#include <cstdint>

#include "Arithmetic.hpp"

namespace OneMotor
{
    /**
     * @class DeltaT
     * @brief 一个用于计算时间增量 (delta time) 的工具类。
     * @tparam T 计算结果的数据类型，必须是算术类型。
     * @details
     * 每次调用 `getDeltaMS()` 方法时，它会返回自上次调用以来经过的时间（以毫秒为单位）。
     * 这在需要基于时间间隔进行计算的场景（如PID控制器）中非常有用。
     */
    template <Arithmetic T = float>
    class DeltaT
    {
    public:
        /**
         * @brief 构造函数，初始化计时器。
         */
        DeltaT();

        /**
         * @brief 获取自上次调用以来经过的时间。
         * @return T类型的值，表示经过的毫秒数。
         */
        T getDeltaMS();

        /**
         * @brief 重置计时器。
         * @details 将上一次记录的时间点更新为当前时间。
         */
        void reset();

    private:
#ifdef ONE_MOTOR_LINUX
        std::chrono::time_point<std::chrono::steady_clock> last_time; ///< 记录上一次调用的时间点
#else
#ifdef CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER
        uint64_t last_time_cycles;
#else
        uint32_t last_time_cycles;
#endif
#endif
    };
}

#endif //DELTAT_HPP
