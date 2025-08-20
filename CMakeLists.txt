cmake_minimum_required(VERSION 3.20)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 20)

# Check ROS2 environment
find_package(ament_cmake QUIET)

option(BUILD_OM_EXAMPLE "Build OneMotor Examples" OFF)
option(BUILD_OM_TEST "Build OneMotor Tests" OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
include(OneMotorFindTlExpected)

if (ZEPHYR_TOOLCHAIN_VARIANT)
    project(onemotor VERSION 0.6.0)
elseif (ament_cmake_FOUND)
    project(one_motor VERSION 0.6.0)
else ()
    project(OneMotor VERSION 0.6.0)
endif ()

if (NOT ZEPHYR_TOOLCHAIN_VARIANT)
    include(OneMotorFindHyCAN)
endif ()

# Collect source files
file(GLOB_RECURSE LINUX_SOURCES CONFIGURE_DEPENDS "src/L_*.cpp")
file(GLOB_RECURSE ZEPHYR_SOURCES CONFIGURE_DEPENDS "src/Z_*.cpp")
file(GLOB_RECURSE NORMAL_SOURCES CONFIGURE_DEPENDS "src/N_*.cpp")

# Configure for specific environment
if (ZEPHYR_TOOLCHAIN_VARIANT)
    set(BUILD_OM_EXAMPLE OFF)
    set(BUILD_OM_TEST OFF)
    include(OneMotorZephyr)
elseif (ament_cmake_FOUND)
    include(OneMotorRos2)
else ()
    include(OneMotorLinux)
endif ()

include(OneMotorConfiguration)

if (BUILD_OM_EXAMPLE)
    include(OneMotorExamples)
endif ()

if (BUILD_OM_TEST)
    include(OneMotorTests)
endif ()

include(OneMotorGenerateDoc)
