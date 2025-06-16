#ifndef OTHREAD_HPP
#define OTHREAD_HPP

#include <functional>

#ifdef ONE_MOTOR_LINUX
#include <thread>
#endif

namespace OneMotor::thread
{
    class Othread
    {
    public:
        using ThreadFunc = std::function<void()>;
        explicit Othread(const ThreadFunc& func) noexcept;
        Othread() noexcept;
        ~Othread();
        Othread(const Othread&) = delete;
        Othread(Othread&&) = delete;
        Othread& operator=(const Othread&) = delete;
        Othread& operator=(Othread&&) = delete;

        bool start(const ThreadFunc& func) noexcept;
        bool join() noexcept;
        bool detach() noexcept;
        [[nodiscard]] bool joinable() const noexcept;
        static void sleep_for(uint64_t ns) noexcept;

    private:
        ThreadFunc thread_func{nullptr};
        bool started{false};
        bool joined{false};
        bool detached{false};
#ifdef ONE_MOTOR_LINUX
        std::thread native_handle{};
#endif
    };
}

#endif //OTHREAD_HPP
