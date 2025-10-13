#ifndef ONEMOTOR_DOUBLEBUFFER_HPP
#define ONEMOTOR_DOUBLEBUFFER_HPP

#include <concepts>

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

        const T& read() noexcept
        {
            return *m_CurrentRead.load(std::memory_order_acquire);
        }

        T& write() noexcept
        {
            return *m_CurrentWrite;
        }

        void snapshot(T& out) const noexcept
        {
            auto* p = m_CurrentRead.load(std::memory_order_acquire);
            out = *p;
        }

        void swap() noexcept
        {
            T* old_read = m_CurrentRead.exchange(m_CurrentWrite, std::memory_order_acq_rel);
            m_CurrentWrite = old_read;
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
