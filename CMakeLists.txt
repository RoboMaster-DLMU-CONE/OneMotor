cmake_minimum_required(VERSION 3.20)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(BUILD_OM_EXAMPLE "Build OneMotor Examples" OFF)
option(BUILD_OM_INSTALL "Build OneMotor CMake install scripts" OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


include(OneMotorFindTlExpected)
include(OneMotorFindMpUnits)
include(OneMotorFindOnePid)

if (ZEPHYR_TOOLCHAIN_VARIANT)
    project(onemotor VERSION 0.8.0)
else ()
    project(OneMotor VERSION 0.8.0)
endif ()

file(GLOB_RECURSE LINUX_SOURCES CONFIGURE_DEPENDS "src/L_*.cpp")
file(GLOB_RECURSE ZEPHYR_SOURCES CONFIGURE_DEPENDS "src/Z_*.cpp")
file(GLOB_RECURSE NORMAL_SOURCES CONFIGURE_DEPENDS "src/N_*.cpp")

set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

if (ZEPHYR_TOOLCHAIN_VARIANT)
    include(OneMotorZephyr)
else ()
    include(OneMotorLinux)
endif ()

include(OneMotorConfiguration)

if (BUILD_OM_EXAMPLE)
    include(OneMotorExamples)
endif ()

include(OneMotorGenerateDoc)
