#include <one-motor/util/DeltaT.hpp>

namespace OneMotor::Util
{
    template <Arithmetic T>
    DeltaT<T>::DeltaT()
    {
        last_time = std::chrono::steady_clock::now();
    }

    template <Arithmetic T>
    T DeltaT<T>::getDeltaMS()
    {
        const auto now = std::chrono::steady_clock::now();
        const std::chrono::duration<T, std::milli> delta = now - last_time;
        last_time = now;
        return delta.count();
    }

    template <Arithmetic T>
    void DeltaT<T>::reset()
    {
        last_time = std::chrono::steady_clock::now();
    }

    template class DeltaT<float>;
    template class DeltaT<double>;
}