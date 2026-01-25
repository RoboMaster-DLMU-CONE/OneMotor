\page integration_guide 集成指南

这里是关于如何在 Linux 、 Windows和 Zephyr 项目中引入 OneMotor 库的说明。

## Linux

### 前置依赖

- 构建工具
    - CMake ( >= 3.20 )
- 编译器
    - GCC ( >= 13 ) 或 Clang ( >= 15 )
- 系统环境
    - Linux can 和 vcan 内核模块（通常通过 linux-modules-extra 包提供）
    - 使用`modprobe`加载内核模块：

    ```shell
    sudo modprobe vcan
    sudo modprobe can
    ```
    - 需要[HyCAN](https://github.com/RoboMaster-DLMU-CONE/HyCAN)
      库。你可以从源代码构建，或者直接在[HyCAN Release](https://github.com/RoboMaster-DLMU-CONE/HyCAN/releases)里下载最新的安装包

### 使用CMake远程获取

可以直接在CMake工程中通过Github仓库远程获取OneMotor仓库。请参考下面的`CMakeLists.txt`示例代码：

```cmake
cmake_minimum_required(VERSION 3.20)
project(MyConsumerApp)

include(FetchContent)

# 尝试查找已安装的 OneMotor
find_package(OneMotor QUIET)

if (NOT OneMotor_FOUND)
    message(STATUS "本地未找到 OneMotor 包，尝试从 GitHub 获取...")
    FetchContent_Declare(
            OneMotor_fetched
            GIT_REPOSITORY "https://github.com/RoboMaster-DLMU-CONE/OneMotor"
            GIT_TAG "main"
    )
    FetchContent_MakeAvailable(OneMotor_fetched)
else ()
    message(STATUS "已找到 OneMotor 版本 ${OneMotor_VERSION}")
endif ()

add_executable(MyConsumerApp main.cpp)

target_link_libraries(MyConsumerApp PRIVATE OneMotor::OneMotor)
```

### 从源代码构建

#### CMake构建

```shell
cmake -S . -B build
cmake --build build
#可选：安装到系统
sudo cmake --install build
```

## Windows (WSL2)

在 Windows 上，推荐使用 WSL2 (Windows Subsystem for Linux) 进行开发。

### 环境配置

1.  安装 WSL2: 在 PowerShell 中运行 `wsl --install`。
2.  安装 Linux 发行版: 推荐使用 Ubuntu 22.04 LTS 或更高版本。
3.  配置 USB/CAN 支持:
    *   WSL2 本身不直接支持硬件直通，需要使用 [usbipd-win](https://github.com/dorssel/usbipd-win) 将 USB CAN 分析仪映射到 WSL 中。
    *   映射后，WSL 中会出现对应的 USB 设备，配置 SocketCAN 的步骤与 Linux 相同。

```powershell
# Windows 端 (PowerShell 管理员)
usbipd wsl list
usbipd wsl attach --busid <BUSID>
```

```bash
# WSL 端
sudo ip link set can0 up type can bitrate 1000000
```

其余编译步骤与 Linux 平台完全一致。

## Zephyr

### 作为 West 模块引入

OneMotor 可以作为标准 Zephyr 模块集成到你的工程中。

1.  编辑项目的 `west.yml`，在 `projects` 列表中添加 OneMotor：

```yaml
manifest:
  remotes:
    - name: dlmu-cone
      url-base: https://github.com/RoboMaster-DLMU-CONE

  projects:
    - name: OneMotor
      remote: dlmu-cone
      revision: main
      path: modules/lib/onemotor
```

2.  更新 west 工作区：

```bash
west update
```

3.  在项目的 `CMakeLists.txt` 中启用模块：

```cmake
# 只要 west update 成功，Zephyr 构建系统会自动发现 OneMotor 模块
# 你只需要链接目标库
target_link_libraries(app PRIVATE OneMotor::OneMotor)
```

