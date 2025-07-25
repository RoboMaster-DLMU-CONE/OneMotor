#include "OneMotor/Util/DeltaT.hpp"
#include <zephyr/kernel.h>

static constexpr uint64_t CYCLES_PER_SEC = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;

namespace OneMotor
{
    template <Arithmetic T>
    DeltaT<T>::DeltaT(): last_time_cycles(k_cycle_get_32())
    {
    }

    template <Arithmetic T>
    T DeltaT<T>::getDeltaMS()
    {
#ifdef CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER
        const uint64_t current_cycles = k_cycle_get_64();
#else
        const uint32_t current_cycles = k_cycle_get_32();
#endif
        const auto delta_cycles = current_cycles - last_time_cycles;
        last_time_cycles = current_cycles;

        double ms_value = delta_cycles * 1000.0 / CYCLES_PER_SEC;

        if constexpr (std::is_integral_v<T>)
        {
            return ms_value < 1.0 ? (delta_cycles > 0 ? 1 : 0) : static_cast<T>(ms_value);
        }
        else
        {
            return static_cast<T>(ms_value);
        }
    }

    template <Arithmetic T>
    void DeltaT<T>::reset()
    {
#ifdef CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER
        last_time_cycles = k_cycle_get_64();
#else
        last_time_cycles = k_cycle_get_32();
#endif
    }

    template class DeltaT<float>;
    template class DeltaT<double>;
    template class DeltaT<int>;
    template class DeltaT<uint32_t>;
}
