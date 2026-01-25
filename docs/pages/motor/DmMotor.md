\page dm_motor 达妙电机

面向达妙（DM）系列电机的使用说明与参考。

## 支持范围
- J4310、J10010、J8009、J4340 等型号

## 快速指引
- 接线与初始化：参见达妙电机相关的策略与帧定义文档
- 示例：`example/` 目录下的达妙控制示例（若有）

## 官方文档

建议阅读达妙官方提供的 CAN 通讯手册，了解不同控制模式（MIT、位置速度、速度）的协议细节。

## 使用方法

### 初始化

达妙电机的初始化需要指定 Control Policy（控制策略）。达妙电机支持多种控制模式，OneMotor 提供了对应的策略类。

#### 1. 选择控制策略

- MITPolicy (默认): 也就是达妙的 MIT 模式，支持位置、速度、Kp、Kd、前馈扭矩的混合控制。性能最强，适合高动态场景。
- PosVelPolicy: 位置-速度模式，仅控制位置和速度限制。
- VelPolicy: 纯速度模式。

#### 2. 创建电机实例

```c++
#include <OneMotor/Motor/DM/DmMotor.hpp>
#include <OneMotor/Can/CanDriver.hpp>

using OneMotor::Can::CanDriver;
using namespace OneMotor::Motor::DM;

// 创建 CanDriver
CanDriver driver("can0");

// 定义电机 ID 配置
// slave_id: 电机作为 CAN 从站的发送 ID (Master -> Slave)
// master_id: 电机作为 CAN 主站的反馈 ID (Slave -> Master)
// 注意：达妙的 ID 配置比较灵活，需根据上位机软件设定确认
uint16_t slave_id = 0x01;  
uint16_t master_id = 0x00; 

// 方式 A: 使用 MIT 模式 (推荐)
// 只需要 CanDriver, slave_id, master_id
J4310_MIT motor_mit(driver, slave_id, master_id);

// 方式 B: 使用位置速度模式
J4310_PosVel motor_pv(driver, slave_id, master_id);

// 方式 C: 使用纯速度模式
J4310_Vel motor_v(driver, slave_id, master_id);
```

### 控制电机

#### 使能与设置零点

达妙电机在上电后通常需要使能，并可能需要校准零点。

```c++
// 使能电机
motor_mit.enable();

// 设置当前位置为零点 (慎用，通常在校准时使用)
motor_mit.setZeroPosition();

// 清除错误状态
motor_mit.clearError();
```

#### 设置控制目标

根据选择的策略不同，设置目标值的方法略有差异。

MIT 模式下：
可以同时设置位置、速度、前馈扭矩以及 PD 参数。

```c++
// 设置位置目标
motor_mit.setPosRef(3.14 * rad);

// 设置速度目标
motor_mit.setAngRef(10.0 * rad_per_s);

// 动态调整 MIT 模式下的 Kp 和 Kd
// Kp=5.0, Kd=1.0 (注意：这是达妙 MIT 协议中的参数，非 PID 参数)
motor_mit.setPidParams(0, 5.0, 1.0); 
```

位置速度模式下：

```c++
// 设置位置目标和最大速度
motor_mit.setPosRef(10 * rev);
motor_mit.setAngRef(100 * rpm); // 这里的 AngRef 通常作为速度限制
```

#### 获取反馈

```c++
auto status = motor_mit.getStatus();
if (status) {
    // 角度、速度、扭矩、温度 (部分型号支持)
    std::cout << "Angle: " << status->reduced_angle << std::endl;
    std::cout << "Velocity: " << status->angular_velocity << std::endl;
    std::cout << "Torque: " << status->torque << std::endl;
}
```

## 内部实现原理

与大疆电机不同，达妙电机通常不需要集中的 MotorManager 来进行打包发送（除非使用特定的组控制协议，目前 OneMotor 主要支持单电机协议）。

### 1. 策略模式 (Strategy Pattern)

`DmMotor` 使用策略模式 (`Policy` 模板参数) 来适配达妙电机的不同控制模式。
- 不同的 Policy 决定了 `update()` 函数如何将用户的参考值（Ref）转换为 CAN 报文的数据域。
- 例如，`MITPolicy` 会进行浮点数到 uint16/uint12 的线性映射与打包。

### 2. 即时通讯

达妙电机的控制通常是即时的。调用 `setPosRef` 或 `update` 时，驱动会直接计算并发送一帧 CAN 消息给电机，而不像大疆电机那样等待定时器打包。

### 3. 反馈处理

每个达妙电机实例在初始化时，会向 `CanDriver` 注册自己的 `master_id` 回调。当收到该 ID 的 CAN 帧时，`onFeedback` 函数会被触发，解析数据并存入双缓冲区。
