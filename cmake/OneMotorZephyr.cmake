zephyr_library_named(${PROJECT_NAME})

if (CONFIG_ONE_MOTOR)
    message(STATUS "Configuring OneMotor for Zephyr building system")

    zephyr_library_sources(${ZEPHYR_SOURCES} ${NORMAL_SOURCES})

    zephyr_library_link_libraries(tl::expected)

    zephyr_library_compile_options(
            -ffunction-sections
            -fdata-sections
    )

    zephyr_include_directories(include)

    target_link_libraries(app PRIVATE ${PROJECT_NAME})

endif ()