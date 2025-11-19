cmake_minimum_required(VERSION 3.22)

project(camera_realsense)

add_library(${PROJECT_NAME}
    realsense.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME}
    PUBLIC realsense2 camera_pub_sub utils fluxlog_vision fluxlog utils sensor_pub_sub
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
