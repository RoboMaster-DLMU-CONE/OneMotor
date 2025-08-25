# OneMotor Copilot Instructions

## Repository Overview

OneMotor is a C++ motor driver library developed by Dalian Nationality University C·One team for robotics competitions. It provides efficient, convenient, and universal motor control capabilities.

### High-Level Information

- **Project Type**: C++20 library for motor control
- **Version**: 0.6.0
- **Repository Size**: Medium (~78 source files)
- **Languages**: C++ (primary), CMake (build system), Markdown (documentation)
- **Platforms**: Linux (native), Zephyr RTOS (embedded), Windows (via WSL)
- **Target Motors**: DJI M3508/M2006, DM J4310, with support for additional motors planned

### Key Features

- Ready-to-use support for common competition motors (DJI M3508, DM J4310)
- Multi-platform compatibility (Linux, Zephyr RTOS, Windows via WSL)
- Modern C++20 high-performance implementation
- Modular architecture with separate components for CAN communication, PID control, motor drivers, threading, and utilities

## Build Instructions

### Prerequisites

**Always ensure these dependencies are installed before building:**

```bash
sudo apt update
sudo apt install -y pkg-config libnl-3-dev libnl-nf-3-dev ninja-build
```

**For Linux CAN support (required for motor communication):**
```bash
sudo modprobe vcan
sudo modprobe can
```
Note: CAN modules may not be available in all environments (e.g., Docker containers). Build will succeed without them, but motor communication will not work.

**For documentation generation (optional):**
```bash
sudo apt install -y doxygen
```

### Build Commands

**Basic library build (most common):**
```bash
cmake -S . -B build
cmake --build build
```

**Build with examples and tests:**
```bash
cmake -S . -B build -G Ninja -DBUILD_OM_EXAMPLE=ON -DBUILD_OM_TEST=ON
cmake --build build
```

**Clean rebuild:**
```bash
rm -rf build
cmake -S . -B build -G Ninja -DBUILD_OM_EXAMPLE=ON -DBUILD_OM_TEST=ON
cmake --build build
```

### Build Requirements

- **CMake**: >= 3.20
- **Compiler**: GCC >= 13 or Clang >= 15
- **C++ Standard**: C++20
- **Build Tool**: Ninja (recommended) or Make

### Testing

**Run tests:**
```bash
cd build
ctest --output-on-failure
```

**Run specific test executable:**
```bash
./build/OneMotorTest_PID
```

### Documentation Generation

**Via CMake (recommended):**
```bash
cmake --build build --target doc
```

**Direct Doxygen:**
```bash
cd doc
doxygen Doxyfile
```
Output: `doc/html/index.html`

Note: Doxygen warnings about unsupported tags are expected and can be ignored.

## Codebase Layout

### Project Architecture

The codebase follows a modular design with platform-specific and cross-platform components:

```
include/OneMotor/          # Public headers
├── Can/                   # CAN bus communication
├── Control/               # PID controllers and control algorithms
├── Motor/                 # Motor drivers
│   ├── DJI/              # DJI motor support (M3508, M2006)
│   └── DM/               # DM motor support (J4310)
├── Thread/                # Threading utilities
└── Util/                  # Utility classes (DeltaT, SpinLock, etc.)

src/                       # Implementation files
├── **/L_*.cpp            # Linux-specific implementations
├── **/Z_*.cpp            # Zephyr-specific implementations
└── **/N_*.cpp            # Normal/cross-platform implementations
```

### Key Configuration Files

- **CMakeLists.txt**: Main build configuration
- **cmake/**: Build system modules
  - `OneMotorLinux.cmake`: Linux build configuration
  - `OneMotorZephyr.cmake`: Zephyr build configuration
  - `OneMotorExamples.cmake`: Example targets
  - `OneMotorTests.cmake`: Test configuration
- **west.yml**: Zephyr workspace configuration
- **zephyr/module.yml**: Zephyr module definition
- **doc/Doxyfile**: Documentation configuration

### Examples and Tests

- **example/**: Demo applications showing library usage
  - `basic.cpp`: Basic library usage
  - `DJI/`: DJI motor examples
  - `DM/`: DM motor examples
- **tests/**: Test suites
  - `PID_Latency.cpp`: Linux PID performance test
  - `zephyr/`: Zephyr-specific tests with ZTEST framework

### CI/CD Pipeline

**GitHub Actions workflows:**
- `.github/workflows/linux.yml`: Linux build and test (Ubuntu 24.04, Ninja, CTest)
- `.github/workflows/zephyr.yml`: Zephyr build and test (west twister)
- `.github/workflows/docs.yml`: Documentation generation and deployment

**Pre-commit checks include:**
- CMake configuration and build
- CTest execution
- Zephyr twister tests

## Important Build Notes

### Common Build Issues

1. **Missing CAN modules**: The build may show warnings about vcan/can modules not found. This is expected in containerized environments and won't prevent compilation.

2. **Compiler warnings**: The build typically shows warnings from dependencies (HyCAN, tl::expected) about unused nodiscard return values. These are expected and safe to ignore.

3. **Dependency fetching**: The build automatically fetches HyCAN and tl::expected from GitHub if not found locally.

4. **CMake generator conflicts**: If switching between build generators (Make vs Ninja), always clean build directory first: `rm -rf build`

### Platform-Specific Builds

**Linux Build**: Uses `OneMotorLinux.cmake`, includes CAN support via libnl
**Zephyr Build**: Uses `OneMotorZephyr.cmake`, integrates with Zephyr kernel

### Build Options

- `BUILD_OM_EXAMPLE=ON`: Build example applications
- `BUILD_OM_TEST=ON`: Build test suites
- `CMAKE_BUILD_TYPE`: Debug/Release (default: unspecified)

## Validation Steps

**To verify a successful build:**
1. Check that `libOneMotor.a` exists in `build/`
2. Run `ctest --output-on-failure` (if tests built)
3. Run example executables (if examples built)

**For code changes, always:**
1. Build the library: `cmake --build build`
2. Run tests: `cd build && ctest --output-on-failure`
3. Verify examples compile (if relevant to changes)

## File Organization

### Root Directory Files

- `CMakeLists.txt`: Main build configuration
- `README.md`: Project overview and basic usage
- `LICENSE`: BSD 3-Clause license
- `west.yml`: Zephyr workspace manifest
- `.gitignore`: Git ignore rules (build/, doc/html/, etc.)

### Dependencies

- **HyCAN**: CAN communication library (auto-fetched)
- **tl::expected**: Error handling library (auto-fetched)
- **Zephyr**: RTOS support (external)

### Trust These Instructions

These instructions are comprehensive and tested. Only search for additional information if:
- The provided build commands fail
- You need to modify the build system itself
- The information appears outdated or incorrect

For routine development tasks (building, testing, adding features), follow these instructions without additional exploration.