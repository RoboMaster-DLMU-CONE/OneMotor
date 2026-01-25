\page dji_motor 大疆电机

面向 DJI 系列电机的使用说明与参考。

## 支持范围

- M3508
- M2006
- GM6020

## 官方文档

在开发之前，建议参考官方文档了解相关信息。

- M3508
    - [M3508 减速直流电机套装](https://bbs.robomaster.com/wiki/20204847/809954?source=7)
    - [C620 无刷电机调速器](https://bbs.robomaster.com/wiki/20204847/817324?source=7)
- M2006
    - [M2006 动力系统](https://bbs.robomaster.com/wiki/20204847/809956?source=7)
    - [C610 无刷电机调速器](https://bbs.robomaster.com/wiki/20204847/817325?source=7)
- [GM6020](https://bbs.robomaster.com/wiki/20204847/809953?source=7)

## 使用方法

### 控制器

大疆电机没有自带的控制器，我们需要创建串级PID进行控制。

在`OneMotor`中，我们集成了`OnePID`库的`PIDChain`类，使用该类，你可以高效、优雅地创建任意的高性能PID串级控制器。以下是创建全局位置+速度串级PID控制器的示例代码

```c++

// 位置环、速度环的PID参数

static constexpr PidParams<> POS_DEFAULT_PARAMS{
    .Kp = 1,
    .Ki = 0.05,
    .Kd = 0,
    .MaxOutput = 20000,
    .Deadband = 50,
    .IntegralLimit = 1000,
};
static constexpr PidParams<> ANG_DEFAULT_PARAMS{
    .Kp = 0.1,
    .Ki = 0.05,
    .Kd = 0.01,
    .MaxOutput = 8000,
    .Deadband = 10,
    .IntegralLimit = 100,
};

using OneMotor::Motor::DJI::PIDFeatures; // 由 OneMotor 提供，包含了多种刚需PID特性

// 使用上述参数创建 PidConfig
static constexpr auto conf1 =
    PidConfig<one::pid::Positional, float, PIDFeatures>(POS_DEFAULT_PARAMS);
static constexpr auto conf2 =
    PidConfig<one::pid::Positional, float, PIDFeatures>(ANG_DEFAULT_PARAMS);

// 使用 PidConfig 初始化 PidChain 
static const auto pid_chain = PidChain(conf1, conf2);

```

如果想使用速度环PID控制器，只需要使用速度环的参数创建PidChain对象即可。

```c++

static constexpr PidParams<> ANG_DEFAULT_PARAMS{
    .Kp = 0.1,
    .Ki = 0.05,
    .Kd = 0.01,
    .MaxOutput = 8000,
    .Deadband = 10,
    .IntegralLimit = 100,
};

using OneMotor::Motor::DJI::PIDFeatures;

static constexpr auto conf1 =
    PidConfig<one::pid::Positional, float, PIDFeatures>(POS_DEFAULT_PARAMS);

static const auto pid_chain = PidChain(conf1);

```

### 初始化

#### 创建 CanDriver

首先，创建一个 `CanDriver` 实例用于 CAN 通信。详情请参考 \ref can_driver 。

```c++
#include <OneMotor/Can/CanDriver.hpp>
using OneMotor::Can::CanDriver;

// Linux
CanDriver driver("can0"); 

// Zephyr
// const device* can_dev = DEVICE_DT_GET(DT_NODELABEL(can1));
// CanDriver driver(can_dev);
```

#### 创建电机实例

使用 `makeM3508` 等辅助函数可以方便地创建电机实例。你需要指定电机的 ID（例如 `1` 对应电调 ID 1），并传入 `CanDriver` 和 `PidChain`。

```c++
#include <OneMotor/Motor/DJI/DjiMotor.hpp>

// 引入辅助函数
using OneMotor::Motor::DJI::makeM3508;
using OneMotor::Motor::DJI::makeM2006;
using OneMotor::Motor::DJI::makeGM6020;

// 创建一个 ID 为 1 的 M3508 电机实例
auto m3508_1 = makeM3508<1>(driver, pid_chain);

// 创建一个 ID 为 2 的 M2006 电机实例
auto m2006_2 = makeM2006<2>(driver, pid_chain);

// 创建一个 ID 为 3 的 GM6020 电机实例
auto gm6020_3 = makeGM6020<3>(driver, pid_chain);
```

### 控制电机

#### 使能与设置目标值

电机创建后，默认处于禁用状态。需要调用 `enable()` 开启闭环控制，并使用 `setPosRef` 或 `setVelRef` 设置目标值。

```c++
// 设置位置目标值（例如 2 圈）
// 注意：OneMotor 使用 mp-units 单位系统，确保类型安全
m3508_1.setPosRef(2 * rev); 

// 设置速度目标值（例如 100 rpm）
// m3508_1.setVelRef(100 * rpm);

// 使能电机
m3508_1.enable();
```

#### 动态调整 PID 参数

可以在运行时动态调整 PID 参数。

```c++
// 设置第一级 PID（位置环）的 Kp=1.0, Ki=0.05, Kd=0.0
m3508_1.setPidParams(1, 1.0, 0.05, 0.0);
```

#### 获取电机状态

通过 `getStatus()` 获取电机的实时反馈数据。

```c++
auto status = m3508_1.getStatus();
if (status) {
    std::cout << "Current Angle: " << status->reduced_angle << std::endl;
    std::cout << "Current Speed: " << status->angular_velocity << std::endl;
    std::cout << "Current Torque: " << status->torque << std::endl;
    std::cout << "Temperature: " << status->temperature << std::endl;
}
```

## 内部实现原理

OneMotor 的大疆电机驱动采用了一些独特的设计模式来实现高性能和易用性。

### 1. 模板化设计与 CRTP

`DjiMotor` 类通过 CRTP (Curiously Recurring Template Pattern) 继承自 `MotorBase`，实现了静态多态。这意味着电机类型在编译期就已经确定，避免了虚函数调用的开销。

### 2. 集中式管理 (MotorManager)

为了高效地处理 CAN 总线上的数据，OneMotor 使用 `MotorManager` 进行集中式管理：

- 统一接收：`MotorManager` 注册 CAN 回调，统一接收总线上的所有电机反馈帧（0x201-0x20B）。
- 统一发送：为了优化带宽，大疆电机的控制指令（0x200, 0x1FF, 0x2FF）通常一帧可以包含 4 个电机的控制数据。`MotorManager` 负责将多个电机的控制输出打包成一帧 CAN 消息进行发送。
- 线程模型：`MotorManager` 内部维护一个独立的线程，以固定的频率（通常为 1kHz）运行控制循环：
    1. 计算所有已注册电机的 PID 输出。
    2. 将输出打包成 CAN 帧。
    3. 调用 `CanDriver` 发送数据。

这种架构确保了控制回路的实时性，并将电机控制逻辑与底层的 CAN 通信细节解耦。
