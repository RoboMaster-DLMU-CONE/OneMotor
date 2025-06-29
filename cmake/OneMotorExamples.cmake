add_executable(OneMotorExample_Basic ${PROJECT_SOURCE_DIR}/example/basic.cpp)
target_link_libraries(OneMotorExample_Basic PRIVATE OneMotor)

add_executable(OneMotorExample_M3508 ${PROJECT_SOURCE_DIR}/example/DJI/M3508.cpp)
target_link_libraries(OneMotorExample_M3508 PRIVATE OneMotor)

add_executable(OneMotorExample_M3508_POS ${PROJECT_SOURCE_DIR}/example/DJI/M3508_Positional.cpp)
target_link_libraries(OneMotorExample_M3508_POS PRIVATE OneMotor)

add_executable(OneMotorExample_J4310 ${PROJECT_SOURCE_DIR}/example/DM/J4310.cpp)
target_link_libraries(OneMotorExample_J4310 PRIVATE OneMotor)