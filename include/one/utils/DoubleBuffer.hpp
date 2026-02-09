#ifndef ONEMOTOR_DOUBLEBUFFER_HPP
#define ONEMOTOR_DOUBLEBUFFER_HPP

#include <array>
#include <atomic>
#include <cstring>
#include <utility>

/**
 * @file DoubleBuffer.hpp
 * @brief 定义 DoubleBuffer 类，用于在多线程模型下保证电机状态信息的安全更新
 */

namespace one {
template <typename T> class DoubleBuffer {
  public:
    DoubleBuffer() noexcept {
        static_assert(
            std::is_default_constructible_v<T>,
            "T must be default constructible for DoubleBuffer default ctor");
        m_Buffers[0] = T();
        m_Buffers[1] = T();
        m_CurrentRead.store(&m_Buffers[0], std::memory_order_release);
        m_CurrentWrite = &m_Buffers[1];
    }

    template <typename... Args,
              typename = std::enable_if_t<std::is_constructible_v<T, Args...>>>
    explicit DoubleBuffer(Args &&...args) {
        m_Buffers = std::array<T, 2>{T(std::forward<Args>(args)...),
                                     T(std::forward<Args>(args)...)};
        m_CurrentRead.store(&m_Buffers[0], std::memory_order_release);
        m_CurrentWrite = &m_Buffers[1];
    }

    // 禁用拷贝语义
    DoubleBuffer(const DoubleBuffer &) = delete;
    DoubleBuffer &operator=(const DoubleBuffer &) = delete;

    // 移动语义，保证内部原子指针指向新对象的缓冲区
    DoubleBuffer(DoubleBuffer &&other) noexcept {
        m_Buffers = std::move(other.m_Buffers);

        T *other_read = other.m_CurrentRead.load(std::memory_order_acquire);
        std::size_t idx = 0;
        if (other_read == &other.m_Buffers[0])
            idx = 0;
        else if (other_read == &other.m_Buffers[1])
            idx = 1;
        else
            idx = 0; // 保守处理（应当不会发生）

        m_CurrentRead.store(&m_Buffers[idx], std::memory_order_release);
        m_CurrentWrite = &m_Buffers[1 - idx];

        // 留下被移动对象为有效状态
        other.m_CurrentRead.store(&other.m_Buffers[0],
                                  std::memory_order_release);
        other.m_CurrentWrite = &other.m_Buffers[1];
    }

    DoubleBuffer &operator=(DoubleBuffer &&other) noexcept {
        if (this == &other)
            return *this;

        m_Buffers = std::move(other.m_Buffers);

        T *other_read = other.m_CurrentRead.load(std::memory_order_acquire);
        std::size_t idx = 0;
        if (other_read == &other.m_Buffers[0])
            idx = 0;
        else if (other_read == &other.m_Buffers[1])
            idx = 1;
        else
            idx = 0;

        m_CurrentRead.store(&m_Buffers[idx], std::memory_order_release);
        m_CurrentWrite = &m_Buffers[1 - idx];

        other.m_CurrentRead.store(&other.m_Buffers[0],
                                  std::memory_order_release);
        other.m_CurrentWrite = &other.m_Buffers[1];

        return *this;
    }

    const T &readView() noexcept {
        return *m_CurrentRead.load(std::memory_order_acquire);
    }

    T readCopy() noexcept {
        return *m_CurrentRead.load(std::memory_order_acquire);
    }

    T &write() noexcept { return *m_CurrentWrite; }

    void swap() noexcept {
        T *old_read =
            m_CurrentRead.exchange(m_CurrentWrite, std::memory_order_acq_rel);
        m_CurrentWrite = old_read;
    }

    void push(const T &frame) noexcept {
        std::memcpy(m_CurrentWrite, &frame, sizeof(T));
        swap();
    }

  private:
    std::array<T, 2> m_Buffers;

    std::atomic<T *> m_CurrentRead{nullptr};
    T *m_CurrentWrite{nullptr};
};
} // namespace one

#endif // ONEMOTOR_DOUBLEBUFFER_HPP
