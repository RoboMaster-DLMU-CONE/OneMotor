#ifndef M3508BASE_HPP
#define M3508BASE_HPP
#include <expected>

#include "M3508Frames.hpp"
#include "one-motor/util/SpinLock.hpp"

namespace OneMotor::Motor::DJI
{
    template <uint8_t id>
    class M3508Base
    {
        using Result = std::expected<void, std::string>;

    public:
        virtual ~M3508Base();
        M3508Status getStatus() noexcept;
        Result disable() noexcept;
        Result enable() noexcept;
        Result shutdown() noexcept;

    protected:
        explicit M3508Base(Can::CanDriver& driver);
        virtual void disabled_func_(Can::CanFrame&& frame) = 0;
        virtual void enabled_func_(Can::CanFrame&& frame) = 0;

        static void shutdown_func_([[maybe_unused]] Can::CanFrame&& frame)
        {
        };

        Can::CanDriver& driver_;
        Util::SpinLock status_lock_;
        M3508Status status_;
        static constexpr uint16_t canId_ = id + 0x200;
    };
}

#endif //M3508BASE_HPP
