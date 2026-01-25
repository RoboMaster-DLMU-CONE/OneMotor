\page motors 电机文档

OneMotor 采用现代 C++ 的面向对象设计，将实体电机抽象为软件对象，提供了一套统一、高效且易于扩展的控制接口。

核心设计包含两个层面：
1.  高性能实现 (`MotorBase` + CRTP)：利用静态多态在编译期优化代码，确保零运行时开销。
2.  统一接口 (`IMotor`)：提供运行时多态能力，允许在同一个容器中管理不同类型的电机。

## 快速上手

以达妙 (Damiao) J4310 电机为例，创建一个电机对象并进行控制非常直观：

```cpp
#include <OneMotor/Motor/DM/DmMotor.hpp>
#include <OneMotor/Can/CanDriver.hpp>
#include <iostream>

using namespace OneMotor::Motor::DM;
using namespace OneMotor::Units;

// 1. 初始化 CAN 驱动
OneMotor::Can::CanDriver driver("can0");

// 2. 创建电机实例 (参数：驱动器，CAN ID，Master ID)
J4310_MIT j4310(driver, 0x01, 0x11);

// 3. 设置 PID 参数 (Kp, Ki, Kd)
(void)j4310.setPidParams(0.5f, 0, 1.0f);

// 4. 使能电机
(void)j4310.enable();

// 5. 设置控制目标 (位置, 速度, 前馈扭矩)
(void)j4310.setRefs(0 * deg, 0 * rad / s, 0 * N * m);

// 6. 获取并打印状态
if (auto status = j4310.getStatus()) {
    std::cout << "当前角度: " << status->reduced_angle << std::endl;
}
```

## 统一抽象与多态 (IMotor)

OneMotor 的强大之处在于其统一的抽象层。所有电机类（如 `DjiMotor`, `DmMotor`）最终都继承自 `IMotor` 接口。这意味着你可以使用基类指针统一管理不同品牌、不同型号的电机。

这在构建复杂的机器人系统（如六轴机械臂、轮足机器人）时尤为重要，因为你可以在一个容器中存储所有关节电机，并进行统一的初始化和控制。

### 示例：混合电机控制

```cpp
#include <vector>
#include <memory>
#include <OneMotor/Motor/IMotor.hpp>

// 创建一个存储 IMotor 指针的容器
std::vector<std::unique_ptr<IMotor>> robot_arm;

// 添加不同的电机
robot_arm.push_back(std::make_unique<J4310_MIT>(driver, 0x01, 0x11)); // 关节 1
// 假设这里还有其他类型的电机
// robot_arm.push_back(std::make_unique<M3508_FOC>(driver, 0x201));      // 关节 2

// 统一控制循环
for (auto& motor : robot_arm) {
    // 统一设置位置目标，无需关心具体电机类型
    motor->setPosRef(90 * deg);
}
```

> 性能说明：得益于现代编译器的过程间优化 (IPO/LTO) 和去虚拟化技术 (Devirtualization)，使用虚函数的运行时开销在大多数场景下是可以忽略不计的。

## 支持的电机系列

点击下方链接查看具体电机系列的详细文档与配置方法：

- \subpage dji_motor "大疆 (DJI) 电机"
  - M3508, M2006, GM6020 等
- \subpage dm_motor "达妙 (Damiao) 电机"
  - J4310, J8009, J10010 等
