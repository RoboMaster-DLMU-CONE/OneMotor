#include "one-motor/thread/CanFrameAwaiter.hpp"
#include <utility>

namespace OneMotor::thread
{
    CanFrameAwaiter::CanFrameAwaiter(Can::CanDriver& driver, const std::set<size_t>& can_ids):
        driver(driver), can_ids(can_ids)
    {
        this->driver.registerCallback(this->can_ids, [this](Can::CanFrame&& frame)
        {
            this->internal_callback_handler(std::move(frame));
        });
    }

    void CanFrameAwaiter::internal_callback_handler(Can::CanFrame&& frame)
    {
        received_frame = frame;
        if (handle_to_resume)
        {
            const std::coroutine_handle<> h = handle_to_resume;
            handle_to_resume = nullptr;
            h.resume();
        }
    }

    bool CanFrameAwaiter::await_ready() const noexcept
    {
        return received_frame.has_value();
    }

    void CanFrameAwaiter::await_suspend(const std::coroutine_handle<> coroutine_handle)
    {
        handle_to_resume = coroutine_handle;
    }

    Can::CanFrame CanFrameAwaiter::await_resume() noexcept
    {
        const Can::CanFrame frame = *received_frame;
        received_frame.reset();
        return frame;
    }
}
