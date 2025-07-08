#ifndef ONE_MOTOR_ERROR_HPP
#define ONE_MOTOR_ERROR_HPP

/**
 * @file Error.hpp
 * @author MoonFeather
 * @brief 定义 Error 结构体和 ErrorCode 枚举，用于表示操作错误并与 tl::expected 一起使用。
 * @date 2025-07-08
 *
 */

#include <string>
#include <utility>

namespace OneMotor
{
    /**
     * @brief 错误码枚举
     *
     * 定义可能的错误类型。
     */
    enum class ErrorCode
    {
        /// CAN 驱动内部错误
        CanDriverInternalError,

        /// DJI 电机管理器错误
        DJIMotorManagerError,
    };

    /**
     * @brief 错误信息结构体
     *
     * 封装错误码和错误消息，可与 tl::expected<T, Error> 一起使用。
     */
    struct Error
    {
        /**
         * @brief 构造函数
         * @param c 错误码
         * @param msg 错误消息
         */
        Error(const ErrorCode c, std::string msg): message(std::move(msg)), code(c)
        {
        }

        /// 错误消息文本
        std::string message;
        /// 错误码
        ErrorCode code;
    };
}

#endif //ONE_MOTOR_ERROR_HPP
