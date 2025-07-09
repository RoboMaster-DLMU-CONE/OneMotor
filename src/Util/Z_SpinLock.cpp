#include "OneMotor/Util/SpinLock.hpp"


namespace OneMotor
{
    Z_THREAD_LOCAL k_spinlock_key_t g_spinlock_key;

    void SpinLock::lock() noexcept
    {
        g_spinlock_key = k_spin_lock(&lock_);
    }

    bool SpinLock::try_lock() noexcept
    {
        k_spinlock_key_t temp_key;
        if (const int result = k_spin_trylock(&lock_, &temp_key); result == 0) {
            g_spinlock_key = temp_key;
            return true;
        }
        return false;
    }

    void SpinLock::unlock() noexcept
    {
        k_spin_unlock(&lock_, g_spinlock_key);
    }



}