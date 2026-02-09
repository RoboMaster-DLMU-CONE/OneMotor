#ifndef ONE_MOTOR_ERROR_HPP
#define ONE_MOTOR_ERROR_HPP

/**
 * @file Error.hpp
 * @brief 定义 Error 结构体和 ErrorCode 枚举，用于表示操作错误并与 tl::expected
 * 一起使用。
 */

#include <utility>

namespace one::motor {
/**
 * @brief 错误码枚举
 *
 * 定义可能的错误类型。
 */
enum class ErrorCode {
    /// CAN 驱动内部错误
    CanDriverInternalError,

    /// CAN 驱动未初始化
    CanDriverNotInitialized,

    /// CAN 驱动已初始化
    CanDriverAlreadyInitialized,

    /// DJI 电机管理器错误
    DJIMotorManagerError,

    /// 电机未初始化
    MotorNotInitialized,

    /// 电机已初始化
    MotorAlreadyInitialized,

    /// 电机未失能
    MotorNotDisabled,
};

/**
 * @brief 错误信息结构体
 *
 * 封装错误码和错误消息，可与 tl::expected<T, Error> 一起使用。
 */
struct Error {
    Error(const ErrorCode c, const char *msg) : message(msg), code(c) {}

    /// 错误消息文本
    const char *message;
    /// 错误码
    ErrorCode code;
};
} // namespace one::motor

#endif // ONE_MOTOR_ERROR_HPP
