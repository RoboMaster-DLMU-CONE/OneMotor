cmake_minimum_required(VERSION 3.20)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 23)

option(BUILD_OM_EXAMPLE "Build OneMotor Examples" OFF)
option(BUILD_OM_TEST "Build OneMotor Tests" OFF)

if (ZEPHYR_TOOLCHAIN_VARIANT)
    project(onemotor VERSION 0.6.0)
    message(STATUS "Configuring OneMotor for Zephyr building system")
else ()
    project(OneMotor VERSION 0.6.0)
    message(STATUS "Configuring OneMotor for Linux building system.")
    include(OneMotorFindHyCAN)
endif ()

if (ZEPHYR_TOOLCHAIN_VARIANT)

else ()
    add_library(${PROJECT_NAME})
    set(LIBRARY_TARGET ${PROJECT_NAME})
endif ()

include(OneMotorConfiguration)

target_compile_options(${LIBRARY_TARGET} INTERFACE
        -ffunction-sections
        -fdata-sections
)

target_link_options(${LIBRARY_TARGET} INTERFACE
        -Wl,--gc-sections
)

file(GLOB_RECURSE LINUX_SOURCES CONFIGURE_DEPENDS "src/L_*.cpp")
file(GLOB_RECURSE ZEPHYR_SOURCES CONFIGURE_DEPENDS "src/Z_*.cpp")
file(GLOB_RECURSE NORMAL_SOURCES CONFIGURE_DEPENDS "src/N_*.cpp")

if (ZEPHYR_TOOLCHAIN_VARIANT)
    target_sources(${LIBRARY_TARGET} PRIVATE ${ZEPHYR_SOURCES} ${NORMAL_SOURCES})
else ()
    target_sources(${LIBRARY_TARGET} PRIVATE ${LINUX_SOURCES} ${NORMAL_SOURCES})
    target_link_libraries(${LIBRARY_TARGET} PUBLIC HyCAN::HyCAN)
    target_compile_definitions(${LIBRARY_TARGET} PUBLIC
            ONE_MOTOR_LINUX
    )
endif ()


target_include_directories(${LIBRARY_TARGET} PUBLIC
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
    install(TARGETS ${LIBRARY_TARGET}
            EXPORT ${LIBRARY_TARGET}Targets
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
    install(EXPORT ${LIBRARY_TARGET}Targets
            FILE ${LIBRARY_TARGET}Targets.cmake
            NAMESPACE OneMotor::
            DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${LIBRARY_TARGET}
    )
    write_basic_package_version_file(
            "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_TARGET}ConfigVersion.cmake"
            VERSION ${PROJECT_VERSION}
            COMPATIBILITY AnyNewerVersion
    )
    configure_package_config_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${LIBRARY_TARGET}Config.cmake.in"
            "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_TARGET}Config.cmake"
            INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${LIBRARY_TARGET}
    )
    install(
            FILES
            "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_TARGET}Config.cmake"
            "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_TARGET}ConfigVersion.cmake"
            DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${LIBRARY_TARGET}
    )
endif ()