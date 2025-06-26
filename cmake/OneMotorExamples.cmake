add_executable(OneMotorExample_Basic ${PROJECT_SOURCE_DIR}/example/basic.cpp)
target_link_libraries(OneMotorExample_Basic PRIVATE OneMotor)

add_executable(OneMotorExample_M3508 ${PROJECT_SOURCE_DIR}/example/DJI/M3508.cpp)
target_link_libraries(OneMotorExample_M3508 PRIVATE OneMotor)