/**
 * @file M3508Frames.hpp
 * @brief 定义了与大疆M3508/M2006电机相关的CAN数据帧结构体。
 * @author MoonFeather
 * @date 2025-06-26
 */

#ifndef M3508FRAMES_HPP
#define M3508FRAMES_HPP
#include <string>

#include "OneMotor/Can/CanDriver.hpp"

namespace OneMotor::Motor::DJI
{
    /**
     * @struct M3508RawStatusFrame
     * @brief M3508电机原始状态反馈的CAN帧的直接映射。
     * @details
     * 此结构体用于解析从M3508电机接收到的CAN报文，或构建要发送到电机的CAN报文。
     * 它提供了与 `Can::CanFrame` 相互转换的功能。
     */
    struct M3508RawStatusFrame
    {
        /**
         * @brief 将此结构体转换为通用的 `Can::CanFrame`。
         */
        explicit operator Can::CanFrame() const;

        /**
         * @brief 从通用的 `Can::CanFrame` 构造此结构体。
         * @param frame 从CAN总线接收到的原始帧。
         */
        explicit M3508RawStatusFrame(Can::CanFrame frame);

        /**
         * @brief 将帧内容格式化为可读字符串。
         * @return 包含帧所有字段信息的字符串。
         */
        [[nodiscard]] std::string format() const;

        uint16_t canId; ///< CAN ID
        uint16_t ecd; ///< 编码器原始值 (0~8191)
        int16_t rpm; ///< 转速 (RPM)
        int16_t current; ///< 电机实际电流 (mA)
        uint8_t temperature; ///< 电机温度 (°C)
    };

    /**
     * @struct M3508Status
     * @brief 经过处理和计算后的M3508电机状态。
     * @details
     * 此结构体包含了从原始反馈数据中解析出的更有用的信息，
     * 例如处理过的角度、角速度和累计圈数等。
     */
    struct M3508Status
    {
        uint16_t last_ecd; ///< 上一次的编码器值
        uint16_t ecd; ///< 当前编码器值
        float angle_single_round; ///< 单圈角度 (0-360°)
        float angular; ///< 角速度 (度/秒)
        int16_t real_current; ///< 实际电流 (mA)
        uint8_t temperature; ///< 温度 (°C)
        float total_angle; ///< 累计总角度 (°)，带方向
        int32_t total_round; ///< 累计总圈数，带方向
        int16_t output_current; ///< PID计算后输出给电机的电流值
    };
}

#endif //M3508FRAMES_HPP
