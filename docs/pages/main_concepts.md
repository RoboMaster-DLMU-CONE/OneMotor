\page MainConcepts 概念介绍

本文概览 OneMotor 的设计理念与核心概念，旨在帮助开发者快速理解库的架构，并指引你进入更详细的文档。

OneMotor 在设计过程中始终权衡以下核心要素：

1. 安全性：利用现代 C++ 类型系统在编译期拦截错误（如单位不匹配、类型误用）。
2. 高性能：大量使用模板元编程与零开销抽象，避免运行时虚函数开销。
3. 易用性：提供直观、统一的 API，隐藏底层硬件差异。
4. 跨平台：无缝支持 Linux 和 Zephyr RTOS，一套代码到处运行。

## 多平台兼容特性

OneMotor 通过抽象层屏蔽了底层操作系统的差异：

- Linux：通过 `HyCAN` 库与标准 SocketCAN（如 `can0`）交互。
- Zephyr RTOS：直接复用 Zephyr 原生的 CAN 驱动与内核对象。
- 统一接口：上层应用逻辑完全一致，配合 `Othread`（线程封装）与 `DoubleBuffer`（双缓冲状态同步），确保代码极高的可移植性。

## 第三方库

为了提供最佳的开发体验并避免重复造轮子，OneMotor 集成了一些高质量的现代 C++ 库。

### tl::expected

[tl::expected](https://github.com/TartanLlama/expected) 提供了类似于 Rust `Result` 或 C++23 `std::expected` 的错误处理机制。

- 为什么使用它？ 在嵌入式或实时系统中，异常（Exceptions）往往因不可预测的开销而被禁用。
- 如何工作？ 函数返回 `tl::expected<T, E>`，既包含了成功时的返回值 `T`，也包含了失败时的错误码 `E`
  。开发者必须显式检查结果，从而构建健壮的错误处理流。

```cpp
auto result = motor.enable();
if (result) {
    // 成功
} else {
    // 处理错误：result.error()
}
```

### mp-units

[mp-units](https://mpusz.github.io/mp-units/) 是一个强类型的物理量单位库。

- 解决痛点：彻底根除"这个参数是度还是弧度？是毫秒还是微秒？"这类经典 bug。
- 特性：
    - 所有物理量（角度、速度、扭矩等）都有明确的类型。
    - 编译期自动进行单位换算和量纲检查。
    - 零运行时开销。

示例：

```cpp
using namespace OneMotor::Units;
// 编译器自动处理 deg 到 rad 的转换
motor.setPosRef(180 * deg); 
// 明确的速度单位，防止误传
motor.setAngRef(10 * rad / s);
```

### OnePid

[OnePid](https://gitee.com/dlmu-cone/OnePID) 是专为本项目配套设计的 PID 控制器库。

- 特点：轻量、模板化，支持级联 PID 控制。它与 OneMotor 的架构深度整合，用于在控制策略层实现精确的闭环控制。

## 下一步

- 电机详情：\ref motors "电机文档"
- 集成步骤：\ref integration_guide "集成指南"
- 更多示例：查看 `example/` 目录或 API 参考页。
