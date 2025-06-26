#ifndef M3508FRAMES_HPP
#define M3508FRAMES_HPP
#include <string>
#include <format>

#include "one-motor/can/CanDriver.hpp"

namespace OneMotor::Motor::DJI
{
    struct M3508RawStatusFrame
    {
        explicit operator Can::CanFrame() const;

        explicit M3508RawStatusFrame(Can::CanFrame frame);

        [[nodiscard]] std::string format() const;

        uint16_t canId;
        uint16_t ecd; // 减速前 0~8191
        int16_t rpm;
        int16_t current; //mA
        uint8_t temperature;
    };

    struct M3508Status
    {
        uint16_t last_ecd;
        uint16_t ecd;
        float angle_single_round; // 单圈角度
        float angular; // 角速度,单位为:度/秒
        int16_t real_current; // 实际电流
        uint8_t temperature; // 温度 Celsius
        float total_angle; // 总角度,注意方向
        int32_t total_round; // 总圈数,注意方向
        int16_t output_current; //PID计算完后输出电流
    };
}

template <>
struct std::formatter<OneMotor::Motor::DJI::M3508Status>
{
    constexpr auto parse(std::format_parse_context& ctx)
    {
        return ctx.begin();
    }

    // The format function defines how the object is converted to text.
    template <typename FormatContext>
    auto format(const OneMotor::Motor::DJI::M3508Status& status, FormatContext& ctx) const
    {
        return std::format_to(
            ctx.out(),
            "M3508Status:\n"
            "  ECD (last/current): {} / {}\n"
            "  Angle (single round): {:.2f} deg\n"
            "  Angular Velocity: {:.2f} deg/s\n"
            "  Total Angle: {:.2f} deg\n"
            "  Total Rounds: {}\n"
            "  Current (real/output): {} / {} mA\n"
            "  Temperature: {} C",
            status.last_ecd, status.ecd,
            status.angle_single_round,
            status.angular,
            status.total_angle,
            status.total_round,
            status.real_current, status.output_current,
            status.temperature
        );
    }
};

#endif //M3508FRAMES_HPP
