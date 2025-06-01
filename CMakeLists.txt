cmake_minimum_required(VERSION 3.27)
project(one-motor VERSION 0.1.0)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 20)

option(BUILD_OM_EXAMPLE "Build OneMotor Examples" ON)

if(ZEPHYR_TOOLCHAIN_VARIANT)
    message(STATUS "Configure OneMotor for Zephyr building system")
else ()
    message(STATUS "Configure OneMotor for Linux building system.")
    find_package(hycan REQUIRED)
    find_package(xtr REQUIRED)
endif()

add_library(${PROJECT_NAME})

file(GLOB_RECURSE LINUX_SOURCES CONFIGURE_DEPENDS "src/L_*.cpp")
file(GLOB_RECURSE ZEPHYR_SOURCES
 CONFIGURE_DEPENDS "src/Z_*.cpp")

if(ZEPHYR_TOOLCHAIN_VARIANT)
target_sources(${PROJECT_NAME} PRIVATE ${ZEPHYR_SOURCES})
else ()
target_sources(${PROJECT_NAME} PRIVATE ${LINUX_SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE hycan::hycan xtr::xtr)
target_compile_definitions(${PROJECT_NAME} PUBLIC ONE_MOTOR_LINUX)
endif ()

target_include_directories(${PROJECT_NAME} PUBLIC include)

if (BUILD_OM_EXAMPLE)
    include(OneMotorExamples)
endif ()