/**
 * @file Panic.hpp
 * @brief 提供一个用于处理严重错误的全局函数。
 * @author MoonFeather
 * @date 2025-06-26
 */
#ifndef PANIC_HPP
#define PANIC_HPP

#include <string>

namespace OneMotor::Util
{
    /**
     * @brief 触发一个严重错误（panic）。
     * @details
     * 当程序遇到无法恢复的错误时，应调用此函数。
     * 它会打印一条错误消息到标准错误流，然后立即终止程序。
     * @param message 要打印的错误消息。
     */
    void om_panic(const std::string&& message);
}

#endif //PANIC_HPP
