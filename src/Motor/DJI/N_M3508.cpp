#include <iostream>

#include "OneMotor/Motor/DJI/M3508.hpp"
#include "OneMotor/Motor/DJI/M3508Frames.hpp"
#include "OneMotor/Motor/DJI/MotorManager.hpp"
#include "OneMotor/Util/Panic.hpp"

static constexpr float ECD_TO_ANGLE_DJI = 0.043945f;
static constexpr float RPM_2_ANGLE_PER_SEC = 6.0f;
static constexpr float MAX_CURRENT_OUTPUT = 16384;


void trMsgToStatus(const OneMotor::Motor::DJI::M3508RawStatusFrame& frame,
                   OneMotor::Motor::DJI::M3508Status& status)
{
    auto& [last_ecd, ecd, angle_single_round, angular, real_current, temperature, total_angle, total_round,
        output_current] = status;

    ecd = frame.ecd;
    real_current = frame.current;
    temperature = frame.temperature;
    angle_single_round = ECD_TO_ANGLE_DJI * static_cast<float>(ecd);
    angular = RPM_2_ANGLE_PER_SEC * static_cast<float>(frame.rpm);

    if (ecd - last_ecd > 4096)
    {
        total_round--;
    }
    else if (ecd - last_ecd < -4096)
    {
        total_round++;
    }
    total_angle = static_cast<float>(total_round) * 360 + angle_single_round;
    last_ecd = ecd;
}

namespace OneMotor::Motor::DJI
{
    template <uint8_t id>
    M3508<id, MotorMode::Angular>::M3508(Can::CanDriver& driver,
                                         const Control::PID_Params<float>& ang_params): M3508Base<id>(driver)
    {
        ang_pid_ = std::make_unique<PIDController>(ang_params);
        (void)driver.registerCallback({this->canId_}, [this](Can::CanFrame&& frame)
        {
            this->disabled_func_(std::move(frame));
        }).or_else([](const auto& e)
        {
            panic(std::move(e.message));
        });
    }

    template <uint8_t id>
    void M3508<id, MotorMode::Angular>::setRef(const float ref) noexcept
    {
        ang_ref_.store(ref, std::memory_order_release);
    }

    template <uint8_t id>
    void M3508<id, MotorMode::Angular>::editAngPID(
        const std::function<void(PIDController*)>& func)
    {
        this->status_lock_.lock();
        func(ang_pid_.get());
        this->status_lock_.unlock();
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
        ang_result = std::clamp(ang_result, -MAX_CURRENT_OUTPUT, MAX_CURRENT_OUTPUT); // 顺便避免u16t整型溢出
        const auto output_current = static_cast<int16_t>(ang_result);
        this->status_.output_current = output_current;
        this->status_lock_.unlock();
        const uint8_t hi_byte = output_current >> 8;
        const uint8_t lo_byte = output_current & 0xFF;
        MotorManager::getInstance().pushOutput<id>(this->driver_, lo_byte, hi_byte);
    }

    template <uint8_t id>
    M3508<id, MotorMode::Position>::M3508(Can::CanDriver& driver, const Control::PID_Params<float>& pos_params,
                                          const Control::PID_Params<float>& ang_params) : M3508Base<id>(driver)
    {
        pos_pid_ = std::make_unique<PIDController>(pos_params);
        ang_pid_ = std::make_unique<PIDController>(ang_params);
        ang_pid_->MaxOutputVal = MAX_CURRENT_OUTPUT;
        (void)driver.registerCallback({this->canId_}, [this](Can::CanFrame&& frame)
        {
            this->disabled_func_(std::move(frame));
        }).or_else([](const auto& e)
        {
            panic(std::move(e.message));
        });
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
    void M3508<id, MotorMode::Position>::editPosPID(
        const std::function<void(PIDController*)>& func)
    {
        this->status_lock_.lock();
        func(pos_pid_.get());
        this->status_lock_.unlock();
    }

    template <uint8_t id>
    void M3508<id, MotorMode::Position>::editAngPID(
        const std::function<void(PIDController*)>& func)
    {
        this->status_lock_.lock();
        func(ang_pid_.get());
        this->status_lock_.unlock();
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

        this->status_lock_.lock();
        trMsgToStatus(msg, this->status_);
        auto pos_result = pos_pid_->compute(pos_ref_.load(std::memory_order_acquire), this->status_.total_angle);

        auto ang_result = ang_pid_->compute(pos_result, this->status_.angular);
        ang_result = std::clamp(ang_result, -MAX_CURRENT_OUTPUT, MAX_CURRENT_OUTPUT); // 顺便避免u16t整型溢出
        const auto output_current = static_cast<int16_t>(ang_result);

        // 调试用输出
        // std::cout << static_cast<int>(id) << " " << pos_result << " " << ang_result << " " << output_current <<
        //     std::endl;

        this->status_.output_current = output_current;
        this->status_lock_.unlock();
        const uint8_t hi_byte = output_current >> 8;
        const uint8_t lo_byte = output_current & 0xFF;
        MotorManager::getInstance().pushOutput<id>(this->driver_, lo_byte, hi_byte);
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
