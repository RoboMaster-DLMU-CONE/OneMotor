message(STATUS "Configuring OneMotor for ROS2 ament_cmake building system")

include(OneMotorFindHyCAN)

add_library(${PROJECT_NAME} SHARED)

target_compile_options(${PROJECT_NAME} INTERFACE
        -ffunction-sections
        -fdata-sections
)

target_sources(${PROJECT_NAME} PRIVATE ${LINUX_SOURCES} ${NORMAL_SOURCES})

find_package(tl_expected REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC tl::expected)

target_compile_definitions(${PROJECT_NAME} PUBLIC
        ONE_MOTOR_LINUX
)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_options(${PROJECT_NAME} INTERFACE
        -Wl,--gc-sections
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(tl_expected)
ament_export_include_directories(include)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)

install(EXPORT ${PROJECT_NAME}Targets
        FILE ${PROJECT_NAME}Targets.cmake
        NAMESPACE OneMotor::
        DESTINATION share/${PROJECT_NAME}/cmake
)

ament_package()
