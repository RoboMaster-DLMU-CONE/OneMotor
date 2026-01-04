add_executable(OneMotorExample_M3508_POS ${PROJECT_SOURCE_DIR}/example/DJI/M3508_Positional.cpp)
target_link_libraries(OneMotorExample_M3508_POS PRIVATE OneMotor)

add_executable(OneMotorExample_GM6020 ${PROJECT_SOURCE_DIR}/example/DJI/GM6020.cpp)
target_link_libraries(OneMotorExample_GM6020 PRIVATE OneMotor)

add_executable(OneMotorExample_M3508_Guard ${PROJECT_SOURCE_DIR}/example/DJI/M3508Guard.cpp)
target_link_libraries(OneMotorExample_M3508_Guard PRIVATE OneMotor)

add_executable(OneMotorExample_J4310 ${PROJECT_SOURCE_DIR}/example/DM/J4310.cpp)
target_link_libraries(OneMotorExample_J4310 PRIVATE OneMotor)
