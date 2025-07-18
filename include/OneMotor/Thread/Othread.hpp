/**
 * @file Othread.hpp
 * @brief 提供一个封装了不同平台线程操作的线程类。
 * @author MoonFeather
 * @date 2025-06-26
 */
#ifndef OTHREAD_HPP
#define OTHREAD_HPP

#include <functional>
#include <chrono>

#ifdef ONE_MOTOR_LINUX
#include <thread>
#else
#include <zephyr/kernel.h>
#endif

namespace OneMotor::Thread
{
    /**
     * @brief 使当前线程休眠指定的时间。
     * @tparam Rep `std::chrono::duration` 的表示类型。
     * @tparam Period `std::chrono::duration` 的周期类型。
     * @param duration 要休眠的时间段。
     */
    template <typename Rep, typename Period>
    void sleep_for(const std::chrono::duration<Rep, Period>& duration) noexcept
    {
#ifdef ONE_MOTOR_LINUX
        std::this_thread::sleep_for(duration);
#endif
    }

    /**
     * @class Othread
     * @brief 一个对底层线程实现（如 std::thread）的封装器。
     * @details
     * 此类提供了一个基本的线程接口，用于启动、加入(join)或分离(detach)线程。
     * 它旨在简化线程管理，并确保在析构时线程能够被正确处理（默认尝试加入）。
     * 此类是不可拷贝和不可移动的，以防止对底层线程句柄的意外操作。
     */
    class Othread
    {
    public:
        /**
         * @brief 线程执行函数的类型定义。
         */
        using ThreadFunc = std::function<void()>;

        /**
         * @brief 构造函数，创建一个新的线程并立即开始执行。
         * @param func 要在线程中执行的函数。
         */
        explicit Othread(ThreadFunc& func) noexcept;

        /**
         * @brief 默认构造函数，创建一个空的线程对象，不开始执行。
         */
        Othread() noexcept;

        /**
         * @brief 析构函数。
         * @details 如果线程是可加入的(joinable)，则会自动尝试加入该线程。
         * 如果加入失败，为了防止程序终止，它将尝试分离线程。
         */
        ~Othread();

        Othread(const Othread&) = delete;
        Othread(Othread&&) = delete;
        Othread& operator=(const Othread&) = delete;
        Othread& operator=(Othread&&) = delete;

        /**
         * @brief 启动线程执行。
         * @param func 要在线程中执行的函数。
         * @return 如果成功启动线程，返回 `true`；如果线程已启动或函数无效，返回 `false`。
         */
        bool start(ThreadFunc& func) noexcept;

        /**
         * @brief 等待线程执行完成。
         * @return 如果成功加入线程，返回 `true`；如果线程未启动、已加入或已分离，返回 `false`。
         */
        bool join() noexcept;

        /**
         * @brief 将线程从当前对象中分离。
         * @details 分离后，线程将在后台独立运行，Othread对象不再拥有该线程。
         * @note 在Zephyr RTOS上，该函数无实际作用。
         * @return 如果成功分离线程，返回 `true`；如果线程未启动、已加入或已分离，返回 `false`。
         */
        bool detach() noexcept;

        /**
         * @brief 检查线程是否可加入。
         * @return 如果线程正在执行且尚未被加入或分离，返回 `true`。
         */
        [[nodiscard]] bool joinable() const noexcept;

    private:
        bool started{false}; ///< 标志线程是否已启动
        bool joined{false}; ///< 标志线程是否已被加入
        bool detached{false}; ///< 标志线程是否已被分离
#ifdef ONE_MOTOR_LINUX
        ThreadFunc thread_func{nullptr}; ///< 存储线程要执行的函数
        std::thread native_handle{}; ///< 底层的 std::thread 对象
#else
        mutable k_thread k_thread_handle{};
        k_thread_stack_t k_thread_stack{};
        k_tid_t k_tid{};
#endif
    };
}

#endif //OTHREAD_HPP
