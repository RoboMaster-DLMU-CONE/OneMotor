#ifndef ONE_MOTOR_SPINLOCK_HPP
#define ONE_MOTOR_SPINLOCK_HPP

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

#endif //ONE_MOTOR_SPINLOCK_HPP
