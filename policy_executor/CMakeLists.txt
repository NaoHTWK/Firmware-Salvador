project(policy_executor)

add_library(${PROJECT_NAME}
policy_executor.cpp
policy_executor.h
)
target_link_libraries(${PROJECT_NAME} utils tfliteexecuter sensor_pub_sub time)
target_include_directories(${PROJECT_NAME} PUBLIC .)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)