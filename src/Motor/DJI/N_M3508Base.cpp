#include "OneMotor/Motor/DJI/M3508Base.hpp"
#include "OneMotor/Motor/DJI/MotorManager.hpp"
#include "OneMotor/Util/Panic.hpp"

namespace OneMotor::Motor::DJI
{
    template <uint8_t id>
    M3508Base<id>::M3508Base(Can::CanDriver& driver): driver_(driver), status_()

    {
        static_assert(id >= 1 && id <= 8, "M3508 Only support 1 <= id <= 8.");
        MotorManager& manager = MotorManager::getInstance();
        manager.registerMotor(driver_, canId_).or_else([](const auto& e)
        {
            Util::om_panic(std::move(e.message));
        });
        manager.pushOutput<id>(driver_, 0, 0);
    }

    template <uint8_t id>
    M3508Base<id>::~M3508Base()
    {
        MotorManager& manager = MotorManager::getInstance();
        manager.deregisterMotor(driver_, canId_).or_else([](const auto& e)
        {
            Util::om_panic(std::move(e.message));
        });
    }

    template <uint8_t id>
    M3508Status M3508Base<id>::getStatus() noexcept
    {
        status_lock_.lock();
        const auto status = status_;
        status_lock_.unlock();
        return status;
    }

    template <uint8_t id>
    tl::expected<void, Error> M3508Base<id>::disable() noexcept
    {
        return driver_.registerCallback({canId_}, [this](Can::CanFrame&& frame)
        {
            this->disabled_func_(std::move(frame));
        });
    }

    template <uint8_t id>
    tl::expected<void, Error> M3508Base<id>::enable() noexcept
    {
        return driver_.registerCallback({canId_}, [this](Can::CanFrame&& frame)
        {
            this->enabled_func_(std::move(frame));
        });
    }

    template <uint8_t id>
    tl::expected<void, Error> M3508Base<id>::shutdown() noexcept
    {
        return driver_.registerCallback({canId_}, shutdown_func_);
    }

    template class M3508Base<1>;
    template class M3508Base<2>;
    template class M3508Base<3>;
    template class M3508Base<4>;
    template class M3508Base<5>;
    template class M3508Base<6>;
    template class M3508Base<7>;
    template class M3508Base<8>;
}
