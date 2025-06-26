/**
 * @file Arithmetic.hpp
 * @brief 定义了一个C++20概念，用于约束模板参数必须为算术类型。
 * @author MoonFeather
 * @date 2025-06-26
 */
#ifndef ARITHMETIC_HPP
#define ARITHMETIC_HPP

#include <concepts>

namespace OneMotor::Util
{
    /**
     * @concept Arithmetic
     * @brief 约束一个类型 `T` 必须是整数类型或浮点数类型。
     * @tparam T 要检查的类型。
     */
    template <typename T>
    concept Arithmetic = std::integral<T> || std::floating_point<T>;
}

#endif //ARITHMETIC_HPP
