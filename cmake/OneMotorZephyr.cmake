message(STATUS "Configuring OneMotor for Zephyr building system")

set(ZEPHYR_PROJECT_NAME "onemotor")

zephyr_library_named(${PROJECT_NAME})

zephyr_library_sources(${ZEPHYR_SOURCES} ${NORMAL_SOURCES})

zephyr_library_link_libraries(tl::expected)

zephyr_library_compile_options(
        -ffunction-sections
        -fdata-sections
)

zephyr_include_directories(include)

message(STATUS "Zephyr RTOS detected, OneMotor won't install itself")
