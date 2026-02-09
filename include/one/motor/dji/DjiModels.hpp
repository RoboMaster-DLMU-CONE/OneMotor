#ifndef ONEMOTOR_DJIMODELS_HPP
#define ONEMOTOR_DJIMODELS_HPP
#include <cstdint>

namespace one::motor::dji {
struct M3508Model {
    static constexpr uint16_t max_current = 16384;       ///< 最大电流
    static constexpr uint16_t encoder_resolution = 8192; ///< 编码器分辨率
    static constexpr bool has_gearbox = true;            ///< 是否有减速箱
    static constexpr uint8_t reduction_ratio = 19;       ///< 减速比
    static constexpr uint8_t max_id = 8;                 ///< 最大ID
    static constexpr float kt = 0.3; ///< 转矩常数 (N * m / A)

    /**
     * @brief 将编码器值转换为角度
     * @param ecd 编码器值
     * @return 角度值 (rad)
     */
    static constexpr float ecd_to_angle(const float ecd) {
        return ecd * 2.0f * std::numbers::pi_v<float> /
               static_cast<float>(encoder_resolution);
    }

    /**
     * @brief 获取控制ID
     * @return 控制ID
     */
    static uint16_t control_id(const uint8_t id) {
        if (id <= 4) {
            return 0x200;
        }
        return 0x1FF;
    }

    /**
     * @brief 获取控制偏移
     * @return 控制偏移
     */
    static uint8_t control_offset(const uint8_t id) {
        if (id <= 4) {
            return (id - 1) * 2;
        }
        return (id - 4 - 1) * 2;
    }

    /**
     * @brief 获取反馈ID
     * @return 反馈ID
     */
    static uint16_t feedback_id(const uint8_t id) { return id + 0x200; }
};

struct M2006Model {
    static constexpr uint16_t max_current = 16384;       ///< 最大电流
    static constexpr uint16_t encoder_resolution = 8192; ///< 编码器分辨率
    static constexpr bool has_gearbox = true;            ///< 是否有减速箱
    static constexpr uint8_t reduction_ratio = 36;       ///< 减速比
    static constexpr uint8_t max_id = 8;                 ///< 最大ID
    static constexpr float kt = 0.18; ///< 转矩常数 (N * m / A)

    /**
     * @brief 将编码器值转换为角度
     * @param ecd 编码器值
     * @return 角度值
     */
    static constexpr float ecd_to_angle(const float ecd) {
        return ecd * 2.0f * std::numbers::pi_v<float> /
               static_cast<float>(encoder_resolution);
    }

    /**
     * @brief 获取控制ID
     * @return 控制ID
     */
    static uint16_t control_id(const uint8_t id) {
        if (id <= 4) {
            return 0x200;
        }
        return 0x1FF;
    }

    /**
     * @brief 获取控制偏移
     * @return 控制偏移
     */
    static uint8_t control_offset(const uint8_t id) {
        if (id <= 4) {
            return (id - 1) * 2;
        }
        return (id - 4 - 1) * 2;
    }

    /**
     * @brief 获取反馈ID
     * @return 反馈ID
     */
    static uint16_t feedback_id(const uint8_t id) { return id + 0x200; }
};

struct GM6020VoltageModel {
    static constexpr uint16_t max_current = 16384;       ///< 最大电流
    static constexpr uint16_t encoder_resolution = 8192; ///< 编码器分辨率
    static constexpr bool has_gearbox = false;           ///< 是否有减速箱
    static constexpr uint8_t max_id = 7;                 ///< 最大ID
    static constexpr float kt = 0.741; ///< 转矩常数 (N * m / A)

    /**
     * @brief 将编码器值转换为角度
     * @param ecd 编码器值
     * @return 角度值
     */
    static constexpr float ecd_to_angle(const float ecd) {
        return ecd * 2.0f * std::numbers::pi_v<float> /
               static_cast<float>(encoder_resolution);
    }

    /**
     * @brief 获取控制ID
     * @return 控制ID
     */
    static uint16_t control_id(const uint8_t id) {
        if (id <= 4) {
            return 0x1FF;
        }
        return 0x2FF;
    }

    /**
     * @brief 获取控制偏移
     * @return 控制偏移
     */
    static uint8_t control_offset(const uint8_t id) {
        if (id <= 4) {
            return (id - 1) * 2;
        }
        return (id - 4 - 1) * 2;
    }

    /**
     * @brief 获取反馈ID
     * @return 反馈ID
     */
    static uint16_t feedback_id(const uint8_t id) { return id + 0x204; }
};

struct GM6020CurrentModel {
    static constexpr uint16_t max_output = 25000;        ///< 最大输出
    static constexpr uint16_t encoder_resolution = 8192; ///< 编码器分辨率
    static constexpr bool has_gearbox = false;           ///< 是否有减速箱
    static constexpr uint8_t max_id = 7;                 ///< 最大ID
    static constexpr float kt = 0.741; ///< 转矩常数 (N * m / A)

    /**
     * @brief 将编码器值转换为角度
     * @param ecd 编码器值
     * @return 角度值
     */
    static constexpr float ecd_to_angle(const float ecd) {
        return ecd * 2.0f * std::numbers::pi_v<float> /
               static_cast<float>(encoder_resolution);
    }

    /**
     * @brief 获取控制ID
     * @return 控制ID
     */
    static uint16_t control_id(const uint8_t id) {
        if (id <= 4) {
            return 0x1FE;
        }
        return 0x2FE;
    }

    /**
     * @brief 获取控制偏移
     * @return 控制偏移
     */
    static uint8_t control_offset(const uint8_t id) {
        if (id <= 4) {
            return (id - 1) * 2;
        }
        return (id - 4 - 1) * 2;
    }

    /**
     * @brief 获取反馈ID
     * @return 反馈ID
     */
    static uint16_t feedback_id(const uint8_t id) { return id + 0x204; }
};

} // namespace one::motor::dji

#endif // ONEMOTOR_DJIMODELS_HPP
