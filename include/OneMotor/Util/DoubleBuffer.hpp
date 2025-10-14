#ifndef ONEMOTOR_DOUBLEBUFFER_HPP
#define ONEMOTOR_DOUBLEBUFFER_HPP

#include <cstring>
#include <atomic>

/**
 * @file DoubleBuffer.hpp
 * @author MoonFeather
 * @brief 定义 DoubleBuffer 类，用于在多线程模型下保证电机状态信息的安全更新
 * @date 2025-10-14
 *
 */


namespace OneMotor
{
    template <typename T>
    class DoubleBuffer
    {
    public:
        template <typename... Args>
        explicit DoubleBuffer(Args&&... args)
        {
            new(&m_Buffers[0]) T(std::forward<Args>(args)...);
            new(&m_Buffers[1])T(std::forward<Args>(args)...);
            m_CurrentRead.store(&m_Buffers[0], std::memory_order_release);
            m_CurrentWrite = &m_Buffers[1];
        }

        ~DoubleBuffer() noexcept
        {
            T* r = &m_Buffers[0];
            T* w = &m_Buffers[1];
            r->~T();
            w->~T();
        }

        DoubleBuffer(const DoubleBuffer&) = delete;
        DoubleBuffer& operator=(const DoubleBuffer&) = delete;

        const T& readView() noexcept
        {
            return *m_CurrentRead.load(std::memory_order_acquire);
        }

        T readCopy() noexcept
        {
            return *m_CurrentRead.load(std::memory_order_acquire);
        }

        T& write() noexcept
        {
            return *m_CurrentWrite;
        }

        void swap() noexcept
        {
            T* old_read = m_CurrentRead.exchange(m_CurrentWrite, std::memory_order_acq_rel);
            m_CurrentWrite = old_read;
        }

        void push(const T& frame) noexcept
        {
            std::memcpy(m_CurrentWrite, &frame, sizeof(T));
            swap();
        }

    private:
        union
        {
            T m_Buffers[2];
        };

        std::atomic<T*> m_CurrentRead{nullptr};
        T* m_CurrentWrite{nullptr};
    };
}

#endif //ONEMOTOR_DOUBLEBUFFER_HPP
