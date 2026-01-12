#ifndef ONEMOTOR_TRAITS_HPP
#define ONEMOTOR_TRAITS_HPP
/**
 * @file DjiTraits.hpp
 * @brief 大疆电机特性定义
 */

#include "DjiFrame.hpp"
#include <cstdint>

namespace OneMotor::Motor::DJI {

/**
 * @brief M3508电机特性
 * @tparam motor_id 电机ID
 */
template <uint8_t motor_id> struct M3508Traits {
    static constexpr auto name = "M3508"; ///< 电机名称
    using StatusType = MotorStatusPlain; ///< 状态类型
    using UserStatusType = MotorStatus; ///< 用户状态类型
    static constexpr uint16_t max_current = 16384; ///< 最大电流
    static constexpr uint16_t encoder_resolution = 8192; ///< 编码器分辨率
    static constexpr bool has_gearbox = true; ///< 是否有减速箱
    static constexpr uint8_t reduction_ratio = 19; ///< 减速比
    static constexpr uint8_t max_id = 8; ///< 最大ID
    static_assert(motor_id > 0 && motor_id < max_id, "Invalid motor_id");

    /**
     * @brief 将编码器值转换为角度
     * @param ecd 编码器值
     * @return 角度值
     */
    static constexpr float ecd_to_angle(const float ecd) {
        return ecd * 360.f / encoder_resolution;
    }

    /**
     * @brief 获取控制ID
     * @return 控制ID
     */
    static consteval uint16_t control_id() {
        if constexpr (motor_id <= 4) {
            return 0x200;
        } else {
            return 0x1FF;
        }
    }

    /**
     * @brief 获取控制偏移
     * @return 控制偏移
     */
    static consteval uint16_t control_offset() {
        if constexpr (motor_id <= 4) {
            return (motor_id - 1) * 2;
        } else {
            return (motor_id - 4 - 1) * 2;
        }
    }

    /**
     * @brief 获取反馈ID
     * @return 反馈ID
     */
    static consteval uint16_t feedback_id() { return motor_id + 0x200; }
};

/**
 * @brief M2006电机特性
 * @tparam motor_id 电机ID
 */
template <uint8_t motor_id> struct M2006Traits {
    static constexpr auto name = "M2006"; ///< 电机名称
    using StatusType = MotorStatusPlain; ///< 状态类型
    using UserStatusType = MotorStatus; ///< 用户状态类型
    static constexpr uint16_t max_current = 16384; ///< 最大电流
    static constexpr uint16_t encoder_resolution = 8192; ///< 编码器分辨率
    static constexpr bool has_gearbox = true; ///< 是否有减速箱
    static constexpr uint8_t reduction_ratio = 36; ///< 减速比
    static constexpr uint8_t max_id = 8; ///< 最大ID
    static_assert(motor_id > 0 && motor_id < max_id, "Invalid motor_id");

    /**
     * @brief 将编码器值转换为角度
     * @param ecd 编码器值
     * @return 角度值
     */
    static constexpr float ecd_to_angle(const float ecd) {
        return ecd * 360.f / encoder_resolution;
    }

    /**
     * @brief 获取控制ID
     * @return 控制ID
     */
    static consteval uint16_t control_id() {
        if constexpr (motor_id <= 4) {
            return 0x200;
        } else {
            return 0x1FF;
        }
    }

    /**
     * @brief 获取控制偏移
     * @return 控制偏移
     */
    static consteval uint16_t control_offset() {
        if constexpr (motor_id <= 4) {
            return (motor_id - 1) * 2;
        } else {
            return (motor_id - 4 - 1) * 2;
        }
    }

    /**
     * @brief 获取反馈ID
     * @return 反馈ID
     */
    static consteval uint16_t feedback_id() { return motor_id + 0x200; }
};

/**
 * @brief GM6020电压模式电机特性
 * @tparam motor_id 电机ID
 */
template <uint8_t motor_id> struct GM6020VoltageTraits {
    static constexpr auto name = "GM6020"; ///< 电机名称
    using StatusType = MotorStatusPlain; ///< 状态类型
    using UserStatusType = MotorStatus; ///< 用户状态类型
    static constexpr uint16_t max_current = 16384; ///< 最大电流
    static constexpr uint16_t encoder_resolution = 8192; ///< 编码器分辨率
    static constexpr bool has_gearbox = false; ///< 是否有减速箱
    static constexpr uint8_t max_id = 7; ///< 最大ID
    static_assert(motor_id > 0 && motor_id < max_id, "Invalid motor_id");

    /**
     * @brief 将编码器值转换为角度
     * @param ecd 编码器值
     * @return 角度值
     */
    static constexpr float ecd_to_angle(const float ecd) {
        return ecd * 360.f / encoder_resolution;
    }

    /**
     * @brief 获取控制ID
     * @return 控制ID
     */
    static consteval uint16_t control_id() {
        if constexpr (motor_id <= 4) {
            return 0x1FF;
        } else {
            return 0x2FF;
        }
    }

    /**
     * @brief 获取控制偏移
     * @return 控制偏移
     */
    static consteval uint16_t control_offset() {
        if constexpr (motor_id <= 4) {
            return (motor_id - 1) * 2;
        } else {
            return (motor_id - 4 - 1) * 2;
        }
    }

    /**
     * @brief 获取反馈ID
     * @return 反馈ID
     */
    static consteval uint16_t feedback_id() { return motor_id + 0x204; }
};

/**
 * @brief GM6020电流模式电机特性
 * @tparam motor_id 电机ID
 */
template <uint8_t motor_id> struct GM6020CurrentTraits {
    static constexpr auto name = "GM6020"; ///< 电机名称
    using StatusType = MotorStatusPlain; ///< 状态类型
    using UserStatusType = MotorStatus; ///< 用户状态类型
    static constexpr uint16_t max_output = 25000; ///< 最大输出
    static constexpr uint16_t encoder_resolution = 8192; ///< 编码器分辨率
    static constexpr bool has_gearbox = false; ///< 是否有减速箱
    static constexpr uint8_t max_id = 7; ///< 最大ID
    static_assert(motor_id > 0 && motor_id < max_id, "Invalid motor_id");

    /**
     * @brief 将编码器值转换为角度
     * @param ecd 编码器值
     * @return 角度值
     */
    static constexpr float ecd_to_angle(const float ecd) {
        return ecd * 360.f / encoder_resolution;
    }

    /**
     * @brief 获取控制ID
     * @return 控制ID
     */
    static consteval uint16_t control_id() {
        if constexpr (motor_id <= 4) {
            return 0x1FE;
        } else {
            return 0x2FE;
        }
    }

    /**
     * @brief 获取控制偏移
     * @return 控制偏移
     */
    static consteval uint16_t control_offset() {
        if constexpr (motor_id <= 4) {
            return (motor_id - 1) * 2;
        } else {
            return (motor_id - 4 - 1) * 2;
        }
    }

    /**
     * @brief 获取反馈ID
     * @return 反馈ID
     */
    static consteval uint16_t feedback_id() { return motor_id + 0x204; }
};
} // namespace OneMotor::Motor::DJI

#endif // ONEMOTOR_TRAITS_HPP
