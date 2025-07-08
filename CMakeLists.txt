cmake_minimum_required(VERSION 3.20)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 20)

option(BUILD_OM_EXAMPLE "Build OneMotor Examples" OFF)
option(BUILD_OM_TEST "Build OneMotor Tests" OFF)

include(OneMotorFindTlExpected)

if (ZEPHYR_TOOLCHAIN_VARIANT)
    project(onemotor VERSION 0.6.0)
    message(STATUS "Configuring OneMotor for Zephyr building system")
else ()
    project(OneMotor VERSION 0.6.0)
    message(STATUS "Configuring OneMotor for Linux building system.")
    include(OneMotorFindHyCAN)
endif ()

if (ZEPHYR_TOOLCHAIN_VARIANT)
    zephyr_library_named(${PROJECT_NAME})
else ()
    add_library(${PROJECT_NAME})
endif ()

include(OneMotorConfiguration)

file(GLOB_RECURSE LINUX_SOURCES CONFIGURE_DEPENDS "src/L_*.cpp")
file(GLOB_RECURSE ZEPHYR_SOURCES CONFIGURE_DEPENDS "src/Z_*.cpp")
file(GLOB_RECURSE NORMAL_SOURCES CONFIGURE_DEPENDS "src/N_*.cpp")

if (ZEPHYR_TOOLCHAIN_VARIANT)
    zephyr_library_sources(${ZEPHYR_SOURCES} ${NORMAL_SOURCES})
    zephyr_library_link_libraries(tl::expected)
    zephyr_include_directories(include)
else ()
    target_compile_options(${PROJECT_NAME} INTERFACE
            -ffunction-sections
            -fdata-sections
    )
    target_link_options(${PROJECT_NAME} INTERFACE
            -Wl,--gc-sections
    )
    target_sources(${PROJECT_NAME} PRIVATE ${LINUX_SOURCES} ${NORMAL_SOURCES})
    target_link_libraries(${PROJECT_NAME} PUBLIC HyCAN::HyCAN tl::expected)
    target_compile_definitions(${PROJECT_NAME} PUBLIC
            ONE_MOTOR_LINUX
    )
    target_include_directories(${PROJECT_NAME} PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )
endif ()


if (BUILD_OM_EXAMPLE)
    include(OneMotorExamples)
endif ()

if (BUILD_OM_TEST)
    include(OneMotorTests)
endif ()

include(OneMotorGenerateDoc)

if (ZEPHYR_TOOLCHAIN_VARIANT)
    message(STATUS "Zephyr RTOS detected, OneMotor won't install itself")
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