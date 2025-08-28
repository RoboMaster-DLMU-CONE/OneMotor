#include "OneMotor/Motor/DJI/M3508Base.hpp"
#include "OneMotor/Motor/DJI/MotorManager.hpp"
#include "OneMotor/Util/Panic.hpp"
#include "OneMotor/Util/Error.hpp"

namespace OneMotor::Motor::DJI
{
    template <uint8_t id>
    M3508Base<id>::M3508Base(Can::CanDriver& driver) : driver_(driver), status_buffers_{}

    {
        static_assert(id >= 1 && id <= 8, "M3508 Only support 1 <= id <= 8.");
        MotorManager& manager = MotorManager::getInstance();
        (void)manager.registerMotor(driver_, canId_).or_else([](const auto& e)
        {
            panic(std::move(e.message));
        });
        manager.pushOutput<id>(driver_, 0, 0);
    }

    template <uint8_t id>
    M3508Base<id>::~M3508Base()
    {
        MotorManager& manager = MotorManager::getInstance();
        (void)manager.deregisterMotor(driver_, canId_).or_else([](const auto& e)
        {
            panic(std::move(e.message));
        });
    }

    template <uint8_t id>
    M3508Status M3508Base<id>::getStatus() noexcept
    {
        // 仅需一次原子加载，获取当前读取缓冲区
        return *current_read_buffer_.load(std::memory_order_acquire);
    }

    // 添加一个新的保护方法，用于在PID解算完成后同步状态
    template <uint8_t id>
    void M3508Base<id>::swapBuffers() noexcept
    {
        // 原子交换读写缓冲区指针，这样下次外部读取就能看到最新状态
        auto* old_read = current_read_buffer_.exchange(current_write_buffer_, std::memory_order_acq_rel);
        current_write_buffer_ = old_read;
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
