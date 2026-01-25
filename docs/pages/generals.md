\page general 跨平台通用封装

为了支持在 Linux 和 Zephyr 等不同平台上高效开发电机驱动，OneMotor 提供了一套统一的硬件抽象层。这些通用封装类屏蔽了底层操作系统的差异，让上层控制逻辑代码可以跨平台复用。

## 目录

- \subpage can_driver : 统一的 CAN 总线驱动接口，支持 Linux (SocketCAN/HyCAN) 和 Zephyr RTOS。
- \subpage othread : 跨平台线程封装，统一了 Linux std::thread 和 Zephyr k_thread 的使用体验。
