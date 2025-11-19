cmake_minimum_required(VERSION 3.22)

project(motion_pub_sub)

add_library(motion_pub_sub
    motion_pub_sub.cpp
    motion_command.cpp
)
target_include_directories(motion_pub_sub
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_SOURCE_DIR}
)
target_link_libraries(motion_pub_sub
    PUBLIC pub_sub utils VisionPrimitives head_control)
target_compile_features(motion_pub_sub INTERFACE cxx_std_17)
set_property(TARGET motion_pub_sub PROPERTY POSITION_INDEPENDENT_CODE ON)
