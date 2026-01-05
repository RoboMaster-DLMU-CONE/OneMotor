#ifndef ONEMOTOR_DTCMALLOCATOR_HPP
#define ONEMOTOR_DTCMALLOCATOR_HPP
#include <memory>
#ifdef __ZEPHYR__
#include <zephyr/kernel.h>

extern k_heap g_dtcm_heap;
#endif

namespace OneMotor {

#ifdef __ZEPHYR__

namespace detail {
void dtcm_heap_init();
}

template <typename T> class DtcmAllocator {
  public:
    using value_type = T;
    using pointer = T *;
    using const_pointer = const T *;
    using size_type = size_t;

    DtcmAllocator() noexcept { detail::dtcm_heap_init(); }

    template <typename U> DtcmAllocator(const DtcmAllocator<U> &) noexcept {}

  public:
    T *allocate(size_t n) {
        if (auto res = k_heap_alloc(&g_dtcm_heap, n * sizeof(T), K_NO_WAIT);
            !res) {
            return nullptr;
        } else {
            return static_cast<T *>(res);
        }
    }

    void deallocate(T *ptr, size_t n) noexcept {
        (void)n;
        k_heap_free(&g_dtcm_heap, ptr);
    }

    template <typename U>
    bool operator==(const DtcmAllocator<U> &) const noexcept {
        return true;
    }

    template <typename U>
    bool operator!=(const DtcmAllocator<U> &) const noexcept {
        return false;
    }
};

#else
template <typename T> using DtcmAllocator = std::allocator<T>;

#endif
} // namespace OneMotor
#endif // ONEMOTOR_DTCMALLOCATOR_HPP
