\page can_driver CAN驱动

OneMotor 提供了一个统一的 CAN 总线抽象层 OneMotor::Can::CanDriver，用于屏蔽不同操作系统（Linux 和 Zephyr）底层的 CAN 接口差异。

## 核心特性

- 跨平台统一接口：在 Linux 上基于 SocketCAN (HyCAN)，在 Zephyr 上基于原生 CAN 驱动。
- RAII 设计：支持通过构造函数自动初始化，也支持延后调用 init。
- 错误处理：所有关键操作均返回 tl::expected，提供强类型的错误检查。
- 回调机制：支持注册针对特定 CAN ID 的回调函数，方便处理接收到的数据。

## 初始化

用户可以根据平台选择合适的初始化方式。

### Linux 平台

在 Linux 平台上，驱动通过 CAN 接口名称（如 "can0"）进行初始化。

```c++
using OneMotor::Can::CanDriver;

int main() {
    // 方式 1: 构造时初始化
    CanDriver driver("can0");

    // 方式 2: 使用智能指针管理
    auto driver_ptr = std::make_unique<CanDriver>("can0");

    // 方式 3: 先声明后初始化
    CanDriver driver_delayed;
    auto result = driver_delayed.init("can0");
    if (!result) {
        // 处理初始化失败
    }
}
```

### Zephyr 平台

在 Zephyr RTOS 上，驱动通过设备树获取的 device 指针进行初始化。

```c++
#include <zephyr/drivers/can.h>
#include <zephyr/device.h>

using OneMotor::Can::CanDriver;

int main() {
    const device* can_dev = DEVICE_DT_GET(DT_NODELABEL(can1));

    if (!device_is_ready(can_dev)) {
        return -1;
    }

    // 构造时初始化
    CanDriver driver(can_dev);
}
```

## 数据收发

初始化完成后，可以使用统一的接口进行数据发送和接收回调注册。

### 发送数据

```c++
OneMotor::Can::CanFrame frame;
frame.can_id = 0x123;
frame.can_dlc = 8;
// ... 填充数据 ...

auto res = driver.send(frame);
if (!res) {
    // 发送失败处理
}
```

### 接收数据

可以通过 registerCallback 注册监听特定 ID 的回调函数。

```c++
std::set<size_t> ids_to_listen = {0x201, 0x202};

driver.registerCallback(ids_to_listen, [](OneMotor::Can::CanFrame frame) {
    // 处理接收到的帧
    // 注意：回调函数通常在独立的接收线程中执行，请注意线程安全
});
```

更多详细 API 说明请参考 Doxygen 生成的类文档。
