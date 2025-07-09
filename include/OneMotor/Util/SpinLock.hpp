/**
 * @file SpinLock.hpp
 * @brief 提供一个简单的自旋锁实现。
 * @author MoonFeather
 * @date 2025-06-26
 */
#ifndef ONE_MOTOR_SPINLOCK_HPP
#define ONE_MOTOR_SPINLOCK_HPP

#ifdef ONE_MOTOR_LINUX
#include <atomic>
#else
#include <zephyr/spinlock.h>
#endif

namespace OneMotor
{
    /**
     * @class SpinLock
     * @brief 一个基础的、非递归的自旋锁。
     * @details
     * 自旋锁是一种忙等待锁，线程在获取锁时会持续循环检查锁的状态，直到成功获取。
     * 它适用于锁持有时间极短的场景，可以避免线程上下文切换的开销。
     * @warning 使用自旋锁时应非常小心，长时间持有锁会导致CPU资源浪费。
     */
    class SpinLock
    {
    public:
        /**
         * @brief 获取锁。
         * @details 如果锁已被其他线程持有，则当前线程将在此处忙等待，直到获取锁为止。
         */
        void lock() noexcept;

        /**
         * @brief 尝试获取锁。
         * @return 如果成功获取锁，返回 `true`；如果锁已被持有，立即返回 `false`，不进行等待。
         */
        bool try_lock() noexcept;

        /**
         * @brief 释放锁。
         * @details 调用此方法的线程必须已经持有了该锁。
         */
        void unlock() noexcept;

    private:
#ifdef ONE_MOTOR_LINUX
        std::atomic<bool> lock_ = {false}; ///< 使用原子布尔值作为锁的状态标志
#else
        k_spinlock lock_{};
#endif
    };
}

#endif //ONE_MOTOR_SPINLOCK_HPP
