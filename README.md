# OneMotor

OneMotor是大连民族大学C·One战队研发的C++电机驱动库，致力于更高效、更便捷、更通用地控制电机。目前该库尚在早期研发阶段。

## 项目亮点

- 比赛常用电机（DJI M3508、DM J4310）开箱即用
- 多平台可用：
    - 原生支持Linux
    - 原生支持基于Zephyr或其它带线程模型的嵌入式RTOS
    - 基于WSL在Windows上可用
- 基于现代C++的高性能实现

## 文档

更多内容请前往 [Github Page](https://robomaster-dlmu-cone.github.io/OneMotor/)

## 支持电机

- [x] DJI M3508/M2006
- [ ] DJI GM6020
- [x] DM-J4310
- [ ] 宇树电机
- [ ] HT
- [ ] ...

## Todo

- [x] 用reinterpret_cast优化CanFrame转换开销
- [x] sleep_for接收std::chrono
- [ ] 各电机对应文档页面

## Credit

- [doxygen-awesome-css](https://github.com/jothepro/doxygen-awesome-css)