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
        /**
         * @brief 将电机状态格式化为可读字符串。
         * @return 包含状态所有字段信息的字符串。
         */
        [[nodiscard]] std::string format() const;
    };
}

#endif //M3508FRAMES_HPP
