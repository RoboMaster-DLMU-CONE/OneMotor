#include "OneMotor/Util/SpinLock.hpp"

namespace OneMotor::Util
{
    void SpinLock::lock() noexcept
    {
        for (;;)
        {
            if (!lock_.exchange(true, std::memory_order_acquire))
            {
                return;
            }
            while (lock_.load(std::memory_order_relaxed))
            {
                __builtin_ia32_pause();
            }
        }
    }

    bool SpinLock::try_lock() noexcept
    {
        return !lock_.load(std::memory_order_relaxed) && !lock_.exchange(true, std::memory_order_acquire);
    }

    void SpinLock::unlock() noexcept
    {
        lock_.store(false, std::memory_order_release);
    }
}
