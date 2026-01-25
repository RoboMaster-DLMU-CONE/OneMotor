/**
 * @file Othread.hpp
 * @brief 提供一个封装了不同平台线程操作的线程类。
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
         *
         * 该函数提供跨平台的线程休眠功能，
         * 在Linux上使用std::this_thread::sleep_for，
         * 在Zephyr上使用k_usleep。
         */
        template <typename Rep, typename Period>
        void sleep_for(const std::chrono::duration<Rep, Period>& duration) noexcept
        {
#ifdef ONE_MOTOR_LINUX
                std::this_thread::sleep_for(duration);
#else
                auto us = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
                k_usleep(us);

#endif
        }

        /**
         * @class Othread
         * @brief 一个对底层线程实现（如 std::thread 或 Zephyr 线程）的封装器。
         * @details
         * 此类提供了一个基本的线程接口，用于启动、加入(join)或分离(detach)线程。
         * 它旨在简化线程管理，并确保在析构时线程能够被正确处理（默认尝试加入）。
         * 此类是不可拷贝和不可移动的，以防止对底层线程句柄的意外操作。
         *
         * 该类支持跨平台操作，在Linux上使用std::thread，
         * 在Zephyr RTOS上使用Zephyr的线程API。
         */
        class Othread
        {
        public:
                /**
                 * @typedef ThreadFunc
                 * @brief 线程执行函数的类型定义。
                 *
                 * 定义了在线程中执行的函数类型，无参数无返回值。
                 */
                using ThreadFunc = std::function<void()>;

                /**
                 * @brief 构造函数，创建一个新的线程并立即开始执行。
                 * @param func 要在线程中执行的函数。
                 *
                 * 构造Othread实例并立即启动线程执行指定的函数。
                 * 在Linux和Zephyr平台上有不同的实现。
                 */
#ifdef ONE_MOTOR_LINUX
                explicit Othread(const ThreadFunc& func) noexcept;
#else
                explicit Othread(ThreadFunc func) noexcept;
#endif

                /**
                 * @brief 默认构造函数，创建一个空的线程对象，不开始执行。
                 *
                 * 创建一个未启动的Othread实例。
                 * 需要通过start方法来启动线程。
                 */
                Othread() noexcept;

                /**
                 * @brief 析构函数。
                 * @details 如果线程是可加入的(joinable)，则会自动尝试加入该线程。
                 * 如果加入失败，为了防止程序终止，它将尝试分离线程。
                 *
                 * 确保线程资源得到正确清理。
                 */
                ~Othread();

                /**
                 * @brief 删除拷贝构造函数
                 *
                 * Othread不可拷贝，以避免线程所有权问题。
                 */
                Othread(const Othread&) = delete;

                /**
                 * @brief 删除移动构造函数
                 *
                 * Othread不可移动，以避免线程所有权问题。
                 */
                Othread(Othread&&) = delete;

                /**
                 * @brief 删除赋值运算符
                 *
                 * Othread不可赋值，以避免线程所有权问题。
                 */
                Othread& operator=(const Othread&) = delete;

                /**
                 * @brief 删除移动赋值运算符
                 *
                 * Othread不可移动赋值，以避免线程所有权问题。
                 */
                Othread& operator=(Othread&&) = delete;

                /**
                 * @brief 启动线程执行。
                 * @param func 要在线程中执行的函数。
                 * @return 如果成功启动线程，返回 `true`；如果线程已启动或函数无效，返回 `false`。
                 *
                 * 启动线程执行指定的函数。
                 * 在Linux和Zephyr平台上有不同的实现。
                 */
#ifdef ONE_MOTOR_LINUX
                bool start(const ThreadFunc& func) noexcept;
#else
                bool start(ThreadFunc& func) noexcept;
#endif


                /**
                 * @brief 等待线程执行完成。
                 * @return 如果成功加入线程，返回 `true`；如果线程未启动、已加入或已分离，返回 `false`。
                 *
                 * 阻塞当前线程直到此线程执行完成。
                 */
                bool join() noexcept;

                /**
                 * @brief 将线程从当前对象中分离。
                 * @details 分离后，线程将在后台独立运行，Othread对象不再拥有该线程。
                 * @note 在Zephyr RTOS上，该函数无实际作用。
                 * @return 如果成功分离线程，返回 `true`；如果线程未启动、已加入或已分离，返回 `false`。
                 *
                 * 将线程设置为分离状态，线程完成后自动回收资源。
                 */
                bool detach() noexcept;

                /**
                 * @brief 检查线程是否可加入。
                 * @return 如果线程正在执行且尚未被加入或分离，返回 `true`。
                 *
                 * 判断线程是否处于可join状态。
                 */
                [[nodiscard]] bool joinable() const noexcept;

        private:
                /**
                 * @var started
                 * @brief 标志线程是否已启动
                 *
                 * 记录线程是否已经被启动。
                 */
                bool started{false};

                /**
                 * @var joined
                 * @brief 标志线程是否已被加入
                 *
                 * 记录线程是否已经被join。
                 */
                bool joined{false};

                /**
                 * @var detached
                 * @brief 标志线程是否已被分离
                 *
                 * 记录线程是否已经被detach。
                 */
                bool detached{false};

#ifdef ONE_MOTOR_LINUX
                /**
                 * @var thread_func
                 * @brief 存储线程要执行的函数
                 *
                 * 保存要在新线程中执行的函数对象。
                 */
                ThreadFunc thread_func{nullptr};

                /**
                 * @var native_handle
                 * @brief 底层的 std::thread 对象
                 *
                 * 保存底层的std::thread对象。
                 */
                std::thread native_handle{};
#else
                /**
                 * @var StackSize
                 * @brief 线程栈大小
                 *
                 * 定义Zephyr线程的栈大小。
                 */
                static constexpr size_t StackSize = 1024;

                /**
                 * @var stack_mem
                 * @brief 线程栈内存
                 *
                 * 为Zephyr线程分配的栈内存。
                 */
                K_KERNEL_STACK_MEMBER(stack_mem, StackSize);

                /**
                 * @var k_thread_handle
                 * @brief Zephyr线程控制块
                 *
                 * Zephyr RTOS的线程控制块。
                 */
                mutable k_thread k_thread_handle{};

                /**
                 * @var k_tid
                 * @brief Zephyr线程ID
                 *
                 * Zephyr RTOS的线程标识符。
                 */
                k_tid_t k_tid{};

                /**
                 * @var func_
                 * @brief 线程执行函数
                 *
                 * 保存要在Zephyr线程中执行的函数对象。
                 */
                ThreadFunc func_;
#endif
        };
}

#endif //OTHREAD_HPP
