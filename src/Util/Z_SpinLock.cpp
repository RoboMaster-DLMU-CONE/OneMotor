#include "OneMotor/Util/SpinLock.hpp"

namespace OneMotor
{
    extern Z_THREAD_LOCAL k_spinlock_key_t g_spinlock_key;
    void SpinLock::lock() noexcept
    {
        g_spinlock_key = k_spin_lock(&lock_);
    }

    bool SpinLock::try_lock() noexcept
    {
        return k_spin_trylock(&lock_, &g_spinlock_key) == 0;
    }

    void SpinLock::unlock() noexcept
    {
        k_spin_unlock(&lock_, g_spinlock_key);
    }



}