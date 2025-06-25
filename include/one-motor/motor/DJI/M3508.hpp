#ifndef M3508_HPP
#define M3508_HPP
#include "M3508Frames.hpp"
#include "one-motor/can/CanDriver.hpp"
#include "one-motor/control/PID.hpp"
#include "one-motor/util/SpinLock.hpp"

namespace OneMotor::Motor::DJI
{
    enum class MotorMode
    {
        Position,
        Angular,
    };

    template <uint8_t id>
    class M3508_Base
    {
    public:
        virtual ~M3508_Base();
        M3508Status getStatus() noexcept;

    protected:
        explicit M3508_Base(Can::CanDriver& driver);
        Can::CanDriver& driver_;
        Util::SpinLock status_lock_;
        M3508Status status_;
        static constexpr uint16_t canId_ = id + 0x200;
    };

    template <uint8_t id, MotorMode mode>
    class M3508;

    template <uint8_t id>
    class M3508<id, MotorMode::Angular> : public M3508_Base<id>
    {
    public:
        explicit M3508(Can::CanDriver& driver, const Control::PID_Params<float>& ang_params);
        void setRef(float ref) noexcept;

    private:
        void disabled_func_(Can::CanFrame&& frame);
        void enabled_func_(Can::CanFrame&& frame);
        std::unique_ptr<Control::PIDController<Control::Positional, float>> ang_pid_;
        std::atomic<float> ang_ref_;
    };

    template <uint8_t id>
    class M3508<id, MotorMode::Position> : public M3508_Base<id>
    {
    public:
        explicit M3508(Can::CanDriver& driver,
                       const Control::PID_Params<float>& pos_params,
                       const Control::PID_Params<float>& ang_params);
        void setAngRef(float ang_ref) noexcept;
        void setPosRef(float pos_ref) noexcept;

    private:
        void disabled_func_(Can::CanFrame&& frame);
        void enabled_func_(Can::CanFrame&& frame);
        std::unique_ptr<Control::PIDController<Control::Positional, float>> pos_pid_;
        std::unique_ptr<Control::PIDController<Control::Positional, float>> ang_pid_;
        std::atomic<float> pos_ref_;
        std::atomic<float> ang_ref_;
    };
}

#endif //M3508_HPP
