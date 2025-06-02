#ifndef CANFRAMEAWAITER_HPP
#define CANFRAMEAWAITER_HPP

#include <coroutine>
#include <optional>
#include <set>
#include "one-motor/can/CanDriver.hpp"
#include "one-motor/can/CanFrame.hpp"

namespace OneMotor::thread
{
    class CanFrameAwaiter
    {
    public:
        CanFrameAwaiter(Can::CanDriver& driver, const std::set<size_t>& can_ids);
        [[nodiscard]] bool await_ready() const noexcept;
        void await_suspend(std::coroutine_handle<> coroutine_handle);
        Can::CanFrame await_resume() noexcept;

    private:
        void internal_callback_handler(Can::CanFrame&& frame);
        Can::CanDriver& driver;
        const std::set<size_t>& can_ids;
        std::optional<Can::CanFrame> received_frame;
        std::coroutine_handle<> handle_to_resume{nullptr};
    };
}


#endif //CANFRAMEAWAITER_HPP
