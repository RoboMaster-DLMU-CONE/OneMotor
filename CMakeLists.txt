cmake_minimum_required(VERSION 3.20)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 20)

option(BUILD_OM_EXAMPLE "Build OneMotor Examples" OFF)
option(BUILD_OM_TEST "Build OneMotor Tests" OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


include(OneMotorFindTlExpected)

if (ZEPHYR_TOOLCHAIN_VARIANT)
    project(onemotor VERSION 0.6.0)
else ()
    project(OneMotor VERSION 0.6.0)
endif ()

include(OneMotorConfiguration)

file(GLOB_RECURSE LINUX_SOURCES CONFIGURE_DEPENDS "src/L_*.cpp")
file(GLOB_RECURSE ZEPHYR_SOURCES CONFIGURE_DEPENDS "src/Z_*.cpp")
file(GLOB_RECURSE NORMAL_SOURCES CONFIGURE_DEPENDS "src/N_*.cpp")

if (ZEPHYR_TOOLCHAIN_VARIANT)
    include(OneMotorZephyr)
else ()
    include(OneMotorLinux)
endif ()

if (BUILD_OM_EXAMPLE)
    include(OneMotorExamples)
endif ()

if (BUILD_OM_TEST)
    include(OneMotorTests)
endif ()

include(OneMotorGenerateDoc)
