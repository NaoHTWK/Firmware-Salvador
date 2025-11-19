cmake_minimum_required(VERSION 3.22)

project(motion_connector)

add_library(motion_connector
    motion_connector_t1.h
    motion_connector_t1.cpp
    motion_connector_t1_custom.h
    motion_connector_t1_custom.cpp
    motion_connector_k1.h
    motion_connector_k1.cpp
    walk_custom.h
    walk_custom.cpp
)
target_include_directories(motion_connector
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(motion_connector
    PUBLIC motion_pub_sub utils fluxlog sensor_pub_sub booster_robotics_sdk time policy_executor parameter_tuner)

target_compile_features(motion_connector INTERFACE cxx_std_17)
