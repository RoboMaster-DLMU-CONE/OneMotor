#include <OneMotor/Util/SpinLock.hpp>
#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <atomic>

using OneMotor::SpinLock;

static void thread_try_lock(void* p1, void* p2, void* p3)
{
    auto* lk = static_cast<SpinLock*>(p1);
    auto* cnt = static_cast<std::atomic<int>*>(p2);
    if (lk->try_lock()) {
        cnt->fetch_add(1);
        lk->unlock();
    }
}

static void thread_hold(void* p1, void* p2, void* p3)
{
    auto* lk = reinterpret_cast<SpinLock*>(p1);
    auto* flag = reinterpret_cast<std::atomic<bool>*>(p2);
    lk->lock();
    flag->store(true);
    k_sleep(K_MSEC(50));
    lk->unlock();
    flag->store(false);
}

ZTEST(spinlock, test_try_lock_and_unlock)
{
    SpinLock lock;
    zassert_true(lock.try_lock(), "Lock should acquire when free");
    lock.unlock();
    zassert_true(lock.try_lock(), "Lock should acquire after unlock");
    lock.unlock();
}

ZTEST(spinlock, test_multithread_try_lock)
{
    SpinLock lock;
    std::atomic<bool> held{false};
    std::atomic<int> success_count{0};

    static K_THREAD_STACK_DEFINE(stack_hold, 1024);
    static struct k_thread t_hold;
    k_thread_create(&t_hold, stack_hold, K_THREAD_STACK_SIZEOF(stack_hold),
                    thread_hold, &lock, &held, nullptr,
                    K_PRIO_PREEMPT(2), 0, K_NO_WAIT);

    /* Wait until lock is held */
    for (int i = 0; i < 1000 && !held.load(); i++) {
        k_sleep(K_MSEC(1));
    }
    zassert_true(held.load(), "Lock should be held by thread_hold");

    static K_THREAD_STACK_DEFINE(stack_try1, 1024);
    static struct k_thread t_try1;
    k_thread_create(&t_try1, stack_try1, K_THREAD_STACK_SIZEOF(stack_try1),
                    thread_try_lock, &lock, &success_count, nullptr,
                    K_PRIO_PREEMPT(3), 0, K_NO_WAIT);
    k_thread_join(&t_try1, K_MSEC(100));
    zassert_equal(success_count.load(), 0, "try_lock should fail when lock is held by another thread");

    /* Wait until lock is released */
    for (int i = 0; i < 1000 && held.load(); i++) {
        k_sleep(K_MSEC(1));
    }
    zassert_false(held.load(), "Lock should be released by thread_hold");

    /* Try to acquire after release: should succeed */
    static K_THREAD_STACK_DEFINE(stack_try2, 1024);
    static struct k_thread t_try2;
    k_thread_create(&t_try2, stack_try2, K_THREAD_STACK_SIZEOF(stack_try2),
                    thread_try_lock, &lock, &success_count, nullptr,
                    K_PRIO_PREEMPT(3), 0, K_NO_WAIT);
    k_thread_join(&t_try2, K_MSEC(100));
    zassert_equal(success_count.load(), 1, "try_lock should succeed after lock is released");

    k_thread_join(&t_hold, K_MSEC(200));
}

ZTEST_SUITE(spinlock, NULL, NULL, NULL, NULL, NULL);
