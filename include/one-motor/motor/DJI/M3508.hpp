#ifndef M3508_HPP
#define M3508_HPP
#include "M3508Frames.hpp"
#include "one-motor/can/CanDriver.hpp"
#include "one-motor/util/SpinLock.hpp"

namespace OneMotor::Motor::DJI
{
    template <uint8_t id>
    class M3508
    {
    public:
        explicit M3508(Can::CanDriver& driver);
        M3508Status getStatus() noexcept;
        ~M3508();

    private:
        void disabled_func_(Can::CanFrame&& frame);

        Can::CanDriver& driver_;
        Util::SpinLock status_lock_;
        M3508Status status_;
        static constexpr uint16_t canId_ = id + 0x200;
    };
}

#endif //M3508_HPP
