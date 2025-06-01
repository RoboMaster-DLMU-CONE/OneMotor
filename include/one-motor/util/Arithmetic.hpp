#ifndef ARITHMETIC_HPP
#define ARITHMETIC_HPP

#include <concepts>

namespace OneMotor::Util
{
    template <typename T>
    concept Arithmetic = std::integral<T> || std::floating_point<T>;
}

#endif //ARITHMETIC_HPP
