enable_testing()
add_executable(OneMotorTest_PID ${PROJECT_SOURCE_DIR}/tests/PID_Latency.cpp)
target_link_libraries(OneMotorTest_PID PRIVATE OneMotor)

add_test(
        NAME OneMotorPIDTest
        COMMAND OneMotorTest_PID
)