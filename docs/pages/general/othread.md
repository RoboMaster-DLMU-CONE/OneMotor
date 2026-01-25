\page othread 跨平台线程封装

OneMotor::Thread::Othread 提供了一个跨平台的线程类，用于统一 Linux 和 Zephyr RTOS 的多线程开发体验。

## 核心特性

- 跨平台一致性：统一了 `std::thread` (Linux) 和 `k_thread` (Zephyr) 的接口差异。
- RAII 管理：析构函数会自动尝试 `join` 线程，避免因忘记 join 导致的资源泄漏或程序异常终止。
- 简化的 API：提供直观的 `start`、`join` 和 `sleep_for` 接口。

## 使用说明

### 1. 创建并启动线程

最简单的用法是在构造时直接传入要在线程中执行的函数，线程会立即启动。

```c++
#include <OneMotor/Thread/Othread.hpp>
#include <iostream>

using OneMotor::Thread::Othread;

void my_task() {
    // 线程执行的代码
    for (int i = 0; i < 5; ++i) {
        // 使用跨平台的 sleep_for
        OneMotor::Thread::sleep_for(std::chrono::seconds(1));
    }
}

int main() {
    // 创建并立即启动线程
    Othread t(my_task);

    // 主线程继续执行...
    
    // t 析构时会自动 join，等待线程结束
    return 0;
}
```

### 2. 延后启动

也可以先创建一个空的线程对象，稍后再启动它。

```c++
Othread t;

// ... 做一些准备工作 ...

// 启动线程
t.start([](){
    // Lambda 表达式作为线程函数
});
```

### 3. 线程休眠

为了避免直接使用平台相关的休眠函数（如 `sleep` 或 `k_sleep`），OneMotor 提供了跨平台的 `sleep_for` 模板函数。

```c++
using namespace std::chrono_literals;

// 休眠 500 毫秒
OneMotor::Thread::sleep_for(500ms);

// 休眠 1 秒
OneMotor::Thread::sleep_for(std::chrono::seconds(1));
```

## 平台差异与注意事项

虽然 Othread 提供了统一的接口，但在底层实现上仍有一些必要的差异需要注意。

### Zephyr RTOS
- 栈大小：在 Zephyr 上，每个 Othread 实例默认拥有 1024 字节 的栈空间。如果线程函数需要大量栈空间，可能会导致栈溢出。
- 优先级：线程以默认优先级创建。

### Linux
- 底层实现：基于 `std::thread`。
- 异常安全：析构函数中包含了保护逻辑，如果 join 失败会尝试 detach，防止因 `std::terminate` 导致整个程序崩溃。
