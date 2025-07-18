#include "OneMotor/Thread/Othread.hpp"


namespace OneMotor::thread
{
    Othread::Othread(ThreadFunc& func) noexcept
    {
        start(func);
    }

    Othread::Othread() noexcept = default;

    Othread::~Othread()
    {
        if (this->joinable())
        {
            if (!this->join())
            {
                if (native_handle.joinable())
                {
                    this->detach();
                }
            }
        }
    }

    bool Othread::start(ThreadFunc& func) noexcept
    {
        if (started || !func)
        {
            return false;
        }
        if (native_handle.joinable())
        {
            return false;
        }
        try
        {
            this->thread_func = func;
            native_handle = std::thread(thread_func);
        }
        catch (...)
        {
            thread_func = nullptr;
            return false;
        }
        started = true;
        joined = false;
        detached = false;
        return true;
    }

    bool Othread::joinable() const noexcept
    {
        return native_handle.joinable();
    }

    bool Othread::join() noexcept
    {
        if (!started || joined || detached)
        {
            return false;
        }
        if (!native_handle.joinable())
        {
            return false;
        }
        try
        {
            native_handle.join();
        }
        catch (...)
        {
            return false;
        }
        joined = true;
        return true;
    }

    bool Othread::detach() noexcept
    {
        if (!started || joined || detached)
        {
            return false;
        }
        if (!native_handle.joinable())
        {
            return false;
        }
        try
        {
            native_handle.detach();
        }
        catch (...)
        {
            return false;
        }

        detached = true;
        return true;
    }
}
