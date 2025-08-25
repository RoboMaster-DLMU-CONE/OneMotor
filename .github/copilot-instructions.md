# OneMotor Copilot 使用说明

## 仓库概览

OneMotor 是大连民族大学 C·One 团队为机器人竞赛开发的 C++ 电机驱动库。它提供高效、便捷和通用的电机控制功能。

### 高级信息

- **项目类型**: 用于电机控制的 C++20 库
- **版本**: 0.6.0
- **仓库规模**: 中等（约 78 个源文件）
- **编程语言**: C++（主要）、CMake（构建系统）、Markdown（文档）
- **支持平台**: Linux（原生）、Zephyr RTOS（嵌入式）、Windows（通过 WSL）
- **目标电机**: DJI M3508/M2006、DM J4310，计划支持更多电机

### 主要特性

- 开箱即用支持常见竞赛电机（DJI M3508、DM J4310）
- 多平台兼容性（Linux、Zephyr RTOS、Windows via WSL）
- 现代 C++20 高性能实现
- 模块化架构，包含 CAN 通信、PID 控制、电机驱动、线程和实用工具等独立组件

## 构建说明

### 前置条件

**构建前请确保已安装以下依赖：**

```bash
sudo apt update
sudo apt install -y pkg-config libnl-3-dev libnl-nf-3-dev ninja-build
```

**Linux CAN 支持（电机通信必需）：**
```bash
sudo modprobe vcan
sudo modprobe can
```
注意：CAN 模块在某些环境中可能不可用（例如 Docker 容器）。构建仍会成功，但电机通信将无法工作。

**文档生成（可选）：**
```bash
sudo apt install -y doxygen
```

### 构建命令

**基础库构建（最常用）：**
```bash
cmake -S . -B build
cmake --build build
```

**构建示例和测试：**
```bash
cmake -S . -B build -G Ninja -DBUILD_OM_EXAMPLE=ON -DBUILD_OM_TEST=ON
cmake --build build
```

**清理重构建：**
```bash
rm -rf build
cmake -S . -B build -G Ninja -DBUILD_OM_EXAMPLE=ON -DBUILD_OM_TEST=ON
cmake --build build
```

### 构建要求

- **CMake**: >= 3.20
- **编译器**: GCC >= 13 或 Clang >= 15
- **C++ 标准**: C++20
- **构建工具**: Ninja（推荐）或 Make

### 测试

**运行测试：**
```bash
cd build
ctest --output-on-failure
```

**运行特定测试可执行文件：**
```bash
./build/OneMotorTest_PID
```

### 文档生成

**通过 CMake（推荐）：**
```bash
cmake --build build --target doc
```

**直接使用 Doxygen：**
```bash
cd doc
doxygen Doxyfile
```
输出：`doc/html/index.html`

注意：Doxygen 关于不支持标签的警告是正常的，可以忽略。

## 代码库布局

### 项目架构

代码库采用模块化设计，包含平台特定和跨平台组件：

```
include/OneMotor/          # 公共头文件
├── Can/                   # CAN 总线通信
├── Control/               # PID 控制器和控制算法
├── Motor/                 # 电机驱动器
│   ├── DJI/              # DJI 电机支持（M3508、M2006）
│   └── DM/               # DM 电机支持（J4310）
├── Thread/                # 线程实用工具
└── Util/                  # 实用工具类（DeltaT、SpinLock 等）

src/                       # 实现文件
├── **/L_*.cpp            # Linux 特定实现
├── **/Z_*.cpp            # Zephyr 特定实现
└── **/N_*.cpp            # 普通/跨平台实现
```

### 关键配置文件

- **CMakeLists.txt**: 主构建配置
- **cmake/**: 构建系统模块
  - `OneMotorLinux.cmake`: Linux 构建配置
  - `OneMotorZephyr.cmake`: Zephyr 构建配置
  - `OneMotorExamples.cmake`: 示例目标
  - `OneMotorTests.cmake`: 测试配置
- **west.yml**: Zephyr 工作空间配置
- **zephyr/module.yml**: Zephyr 模块定义
- **doc/Doxyfile**: 文档配置

### 示例和测试

- **example/**: 展示库用法的演示应用程序
  - `basic.cpp`: 基础库用法
  - `DJI/`: DJI 电机示例
  - `DM/`: DM 电机示例
- **tests/**: 测试套件
  - `PID_Latency.cpp`: Linux PID 性能测试
  - `zephyr/`: 带 ZTEST 框架的 Zephyr 特定测试

### CI/CD 流水线

**GitHub Actions 工作流：**
- `.github/workflows/linux.yml`: Linux 构建和测试（Ubuntu 24.04、Ninja、CTest）
- `.github/workflows/zephyr.yml`: Zephyr 构建和测试（west twister）
- `.github/workflows/docs.yml`: 文档生成和部署

**预提交检查包括：**
- CMake 配置和构建
- CTest 执行
- Zephyr twister 测试

## 重要构建注意事项

### 常见构建问题

1. **缺少 CAN 模块**：构建时可能显示找不到 vcan/can 模块的警告。这在容器化环境中是正常的，不会阻止编译。

2. **编译器警告**：构建时通常会显示来自依赖项（HyCAN、tl::expected）关于未使用 nodiscard 返回值的警告。这些是正常的，可以安全忽略。

3. **依赖获取**：如果本地未找到 HyCAN 和 tl::expected，构建会自动从 GitHub 获取它们。

4. **CMake 生成器冲突**：如果在构建生成器之间切换（Make vs Ninja），请始终先清理构建目录：`rm -rf build`

### 平台特定构建

**Linux 构建**：使用 `OneMotorLinux.cmake`，通过 libnl 包含 CAN 支持
**Zephyr 构建**：使用 `OneMotorZephyr.cmake`，与 Zephyr 内核集成

### 构建选项

- `BUILD_OM_EXAMPLE=ON`: 构建示例应用程序
- `BUILD_OM_TEST=ON`: 构建测试套件
- `CMAKE_BUILD_TYPE`: Debug/Release（默认：未指定）

## 验证步骤

**验证构建成功：**
1. 检查 `build/` 中是否存在 `libOneMotor.a`
2. 运行 `ctest --output-on-failure`（如果构建了测试）
3. 运行示例可执行文件（如果构建了示例）

**对于代码更改，始终：**
1. 构建库：`cmake --build build`
2. 运行测试：`cd build && ctest --output-on-failure`
3. 验证示例编译（如果与更改相关）

## 文件组织

### 根目录文件

- `CMakeLists.txt`: 主构建配置
- `README.md`: 项目概览和基本用法
- `LICENSE`: BSD 3-Clause 许可证
- `west.yml`: Zephyr 工作空间清单
- `.gitignore`: Git 忽略规则（build/、doc/html/ 等）

### 依赖项

- **HyCAN**: CAN 通信库（自动获取）
- **tl::expected**: 错误处理库（自动获取）
- **Zephyr**: RTOS 支持（外部）

### 信任这些说明

这些说明是全面且经过测试的。仅在以下情况下寻找额外信息：
- 提供的构建命令失败
- 需要修改构建系统本身
- 信息显得过时或不正确

对于常规开发任务（构建、测试、添加功能），请遵循这些说明，无需额外探索。