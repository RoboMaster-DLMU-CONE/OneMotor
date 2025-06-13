#ifndef SPINLOCK_HPP
#define SPINLOCK_HPP

#ifdef ONE_MOTOR_LINUX
#include <atomic>
#endif

namespace OneMotor::Util
{
    class SpinLock
    {
    public:
        void lock() noexcept;
        bool try_lock() noexcept;
        void unlock() noexcept;

    private:
#ifdef ONE_MOTOR_LINUX
        std::atomic<bool> lock_ = {false};
#endif
    };
}

#endif //SPINLOCK_HPP
