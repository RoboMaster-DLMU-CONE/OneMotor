cmake_minimum_required(VERSION 3.20)
project(OneMotor VERSION 0.2.1)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 23)

option(BUILD_OM_EXAMPLE "Build OneMotor Examples" OFF)
option(BUILD_OM_TEST "Build OneMotor Tests" OFF)

if (ZEPHYR_TOOLCHAIN_VARIANT)
    message(STATUS "Configuring OneMotor for Zephyr building system")
else ()
    message(STATUS "Configuring OneMotor for Linux building system.")
    include(OneMotorFindHyCAN)
endif ()

include(OneMotorConfiguration)

add_library(${PROJECT_NAME})

file(GLOB_RECURSE LINUX_SOURCES CONFIGURE_DEPENDS "src/L_*.cpp")
file(GLOB_RECURSE ZEPHYR_SOURCES CONFIGURE_DEPENDS "src/Z_*.cpp")
file(GLOB_RECURSE NORMAL_SOURCES CONFIGURE_DEPENDS "src/N_*.cpp")

if (ZEPHYR_TOOLCHAIN_VARIANT)
    target_sources(${PROJECT_NAME} PRIVATE ${ZEPHYR_SOURCES} ${NORMAL_SOURCES})
else ()
    target_sources(${PROJECT_NAME} PRIVATE ${LINUX_SOURCES} ${NORMAL_SOURCES})
    target_link_libraries(${PROJECT_NAME} PUBLIC HyCAN::HyCAN)
    target_compile_definitions(${PROJECT_NAME} PUBLIC
            ONE_MOTOR_LINUX
    )
endif ()

target_compile_definitions(${PROJECT_NAME} PUBLIC
        OM_CAN_MAX_DJI_MOTOR=${OM_CAN_MAX_DJI_MOTOR}
)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if (BUILD_OM_EXAMPLE)
    include(OneMotorExamples)
endif ()

if (BUILD_OM_TEST)
    include(OneMotorTests)
endif ()

include(OneMotorGenerateDoc)

if (ZEPHYR_TOOLCHAIN_VARIANT)
    message(STATUS Zephyr RTOS detected, OneMotor won't install itself)
else ()
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)
    install(TARGETS ${PROJECT_NAME}
            EXPORT ${PROJECT_NAME}Targets
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
    install(EXPORT ${PROJECT_NAME}Targets
            FILE ${PROJECT_NAME}Targets.cmake
            NAMESPACE OneMotor::
            DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    )
    write_basic_package_version_file(
            "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
            VERSION ${PROJECT_VERSION}
            COMPATIBILITY AnyNewerVersion
    )
    configure_package_config_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
            "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
            INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    )
    install(
            FILES
            "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
            "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
            DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    )
endif ()