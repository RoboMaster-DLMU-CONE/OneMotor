#include <OneMotor/Thread/Othread.hpp>

namespace OneMotor::Thread
{
    static void thread_entry(void* func, void*, void*)
    {
        if (func && *func)
        {
            (*func)();
        }
    }

    Othread::Othread(ThreadFunc& func) noexcept
    {
        start(func);
    }

    Othread::Othread() noexcept = default;

    Othread::~Othread()
    {
        if (this->joinable())
        {
            this->join();
        }
    }

    bool Othread::join() noexcept
    {
        return k_thread_join(&k_thread_handle, K_FOREVER) == 0;
    }

    bool Othread::detach() noexcept
    {
        if (detached)
        {
            return true;
        }
        if (!started && joined)
        {
            detached = true;
            return true;
        }
        return false;
    }

    bool Othread::joinable() const noexcept
    {
        return k_thread_join(&k_thread_handle, K_NO_WAIT) == 0;
    }


    bool Othread::start(ThreadFunc& func) noexcept
    {
        if (started || !func)
        {
            return false;
        }

        k_tid = k_thread_create(&k_thread_handle, &k_thread_stack, 1024, thread_entry, &func, nullptr, nullptr, 0,
                                0,
                                K_NO_WAIT);
        return true;
    }
}
