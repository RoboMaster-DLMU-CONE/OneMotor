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
    M3508_Base<id>::M3508_Base(Can::CanDriver& driver): driver_(driver), status_()

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
    M3508_Base<id>::~M3508_Base()
    {
        MotorManager& manager = MotorManager::getInstance();
        if (auto result = manager.deregisterMotor(driver_, canId_); !result)
        {
            Util::om_panic(std::move(result.error()));
        }
    }

    template <uint8_t id>
    M3508Status M3508_Base<id>::getStatus() noexcept
    {
        status_lock_.lock();
        const auto status = status_;
        status_lock_.unlock();
        return status;
    }


    template <uint8_t id>
    M3508<id, MotorMode::Angular>::M3508(Can::CanDriver& driver,
                                         const Control::PID_Params<float>& ang_params): M3508_Base<id>(driver)
    {
        ang_pid_ = std::make_unique<Control::PIDController<Control::Positional, float>>(ang_params);
    }

    template <uint8_t id>
    void M3508<id, MotorMode::Angular>::setRef(const float ref) noexcept
    {
        ang_ref_.store(ref, std::memory_order_release);
    }

    template <uint8_t id>
    void M3508<id, MotorMode::Angular>::disabled_func_(Can::CanFrame&& frame)
    {
        const auto msg = static_cast<M3508RawStatusFrame>(frame);
        this->status_lock_.lock();
        trMsgToStatus(msg, this->status_);
        this->status_lock_.unlock();
    }

    template <uint8_t id>
    void M3508<id, MotorMode::Angular>::enabled_func_(Can::CanFrame&& frame)
    {
        const auto msg = static_cast<M3508RawStatusFrame>(frame);
        this->status_lock_.lock();
        trMsgToStatus(msg, this->status_);
        auto ang_result = ang_pid_->compute(ang_ref_.load(std::memory_order_acquire), this->status_.angular);
        const auto output_current = static_cast<int16_t>(ang_result);
        const uint8_t hi_byte = output_current >> 8;
        const uint8_t lo_byte = output_current & 0xFF;
        MotorManager::getInstance().pushOutput<id>(this->driver_, lo_byte, hi_byte);
        this->status_lock_.unlock();
    }

    template <uint8_t id>
    M3508<id, MotorMode::Position>::M3508(Can::CanDriver& driver, const Control::PID_Params<float>& pos_params,
                                          const Control::PID_Params<float>& ang_params) : M3508_Base<id>(driver)
    {
        pos_pid_ = std::make_unique<Control::PIDController<Control::Positional, float>>(pos_params);
        ang_pid_ = std::make_unique<Control::PIDController<Control::Positional, float>>(ang_params);
    }

    template <uint8_t id>
    void M3508<id, MotorMode::Position>::setAngRef(const float ang_ref) noexcept
    {
        ang_ref_.store(ang_ref, std::memory_order_release);
    }

    template <uint8_t id>
    void M3508<id, MotorMode::Position>::setPosRef(const float pos_ref) noexcept
    {
        pos_ref_.store(pos_ref, std::memory_order_release);
    }

    template <uint8_t id>
    void M3508<id, MotorMode::Position>::disabled_func_(Can::CanFrame&& frame)
    {
        const auto msg = static_cast<M3508RawStatusFrame>(frame);
        this->status_lock_.lock();
        trMsgToStatus(msg, this->status_);
        this->status_lock_.unlock();
    }

    template <uint8_t id>
    void M3508<id, MotorMode::Position>::enabled_func_(Can::CanFrame&& frame)
    {
        const auto msg = static_cast<M3508RawStatusFrame>(frame);
        ang_pid_->MaxOutputVal = ang_ref_.load(std::memory_order_acquire);


        this->status_lock_.lock();
        trMsgToStatus(msg, this->status_);
        const auto pos_result = pos_pid_->compute(pos_ref_.load(std::memory_order_acquire), this->status_.total_angle);
        auto ang_result = ang_pid_->compute(pos_result, this->status_.angular);
        const auto output_current = static_cast<int16_t>(ang_result);
        const uint8_t hi_byte = output_current >> 8;
        const uint8_t lo_byte = output_current & 0xFF;
        MotorManager::getInstance().pushOutput<id>(this->driver_, lo_byte, hi_byte);
        this->status_lock_.unlock();
    }


    template class M3508<1, MotorMode::Angular>;
    template class M3508<2, MotorMode::Angular>;
    template class M3508<3, MotorMode::Angular>;
    template class M3508<4, MotorMode::Angular>;
    template class M3508<5, MotorMode::Angular>;
    template class M3508<6, MotorMode::Angular>;
    template class M3508<7, MotorMode::Angular>;
    template class M3508<8, MotorMode::Angular>;

    template class M3508<1, MotorMode::Position>;
    template class M3508<2, MotorMode::Position>;
    template class M3508<3, MotorMode::Position>;
    template class M3508<4, MotorMode::Position>;
    template class M3508<5, MotorMode::Position>;
    template class M3508<6, MotorMode::Position>;
    template class M3508<7, MotorMode::Position>;
    template class M3508<8, MotorMode::Position>;
}
