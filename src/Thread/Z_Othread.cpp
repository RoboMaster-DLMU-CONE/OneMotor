#include <OneMotor/Thread/Othread.hpp>
#include <utility>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(Othread, CONFIG_LOG_DEFAULT_LEVEL);

namespace OneMotor::Thread
{
    static void thread_entry(void* func, void*, void*)
    {
        LOG_INF("entering thread");
        if (const auto f = static_cast<Othread::ThreadFunc*>(func); f)
        {
            LOG_INF("casted function");
            (*f)();
        }
    }

    Othread::Othread(ThreadFunc func) noexcept : func_(std::move(func))
    {
        start(func_);
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
        LOG_INF("try starting a thread");
        if (started || !func)
        {
            LOG_ERR("func invalid");
            return false;
        }

        k_tid = k_thread_create(&k_thread_handle, stack_mem, K_KERNEL_STACK_SIZEOF(stack_mem), thread_entry, &func,
                                nullptr, nullptr, 0,
                                0,
                                K_NO_WAIT);
        LOG_INF("successfully creaate thread");
        return true;
    }
}
