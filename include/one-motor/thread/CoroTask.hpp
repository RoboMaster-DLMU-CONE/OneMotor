#ifndef COROTASK_HPP
#define COROTASK_HPP
#include <coroutine>

namespace OneMotor::thread
{
    struct CoroTask
    {
        struct promise_type
        {
            CoroTask get_return_object() { return {}; }
            std::suspend_never initial_suspend() { return {}; }
            std::suspend_always final_suspend() { return {}; }
            void return_void() { return; };
            void unhandled_exception();
        };
    };
}
#endif //COROTASK_HPP
