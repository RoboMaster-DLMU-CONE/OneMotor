#include "one-motor/motor/DJI/M3508.hpp"
#include "one-motor/motor/DJI/M3508Frames.hpp"
#include "one-motor/motor/DJI/MotorManager.hpp"
#include "one-motor/util/Panic.hpp"

constexpr float ECD_TO_ANGLE_DJI = 0.043945f;
constexpr float RPM_2_ANGLE_PER_SEC = 6.0f;

void trMsgToStatus(const OneMotor::Motor::DJI::M3508RawStatusFrame& frame,
                   OneMotor::Motor::DJI::M3508Status& status)
{
    status.ecd = frame.ecd;
    status.real_current = frame.current;
    status.temperature = frame.temperature;
    status.angle_single_round = ECD_TO_ANGLE_DJI * static_cast<float>(frame.ecd);
    status.angular = RPM_2_ANGLE_PER_SEC * static_cast<float>(frame.rpm);

    if (frame.ecd - status.last_ecd > 4096)
    {
        status.total_round--;
    }
    else if (frame.ecd - status.last_ecd < -4096)
    {
        status.total_round++;
    }
    status.total_angle = static_cast<float>(status.total_round) * 360 + status.angle_single_round;
}

namespace OneMotor::Motor::DJI
{
    template <uint8_t id>
    M3508<id>::M3508(Can::CanDriver& driver): driver_(driver), status_()
    {
        static_assert(id >= 1 && id <= 8, "M3508 Only support 1 <= id <= 8.");
        MotorManager& manager = MotorManager::getInstance();
        if (auto result = manager.registerMotor(driver_, canId_); !result)
        {
            Util::om_panic(std::move(result.error()));
        }
        manager.pushOutput<id>(driver_, 0, 0);
    }

    template <uint8_t id>
    M3508Status M3508<id>::getStatus() noexcept
    {
        status_lock_.lock();
        const auto status = status_;
        status_lock_.unlock();
        return status;
    }

    template <uint8_t id>
    M3508<id>::~M3508()
    {
        MotorManager& manager = MotorManager::getInstance();
        if (auto result = manager.deregisterMotor(driver_, canId_); !result)
        {
            Util::om_panic(std::move(result.error()));
        }
    }

    template <uint8_t id>
    void M3508<id>::disabled_func_(Can::CanFrame&& frame)
    {
        const auto msg = static_cast<M3508RawStatusFrame>(frame);
        status_lock_.lock();
        trMsgToStatus(msg, status_);
        status_lock_.unlock();
    }

    template class M3508<1>;
    template class M3508<2>;
    template class M3508<3>;
    template class M3508<4>;
    template class M3508<5>;
    template class M3508<6>;
    template class M3508<7>;
    template class M3508<8>;
}
