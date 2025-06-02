#include <exception>
#include "one-motor/thread/CoroTask.hpp"

namespace OneMotor::thread
{
    void CoroTask::promise_type::unhandled_exception()
    {
        std::terminate();
    }
}
