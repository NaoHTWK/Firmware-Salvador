cmake_minimum_required(VERSION 3.22)

project(sensor)

add_library(${PROJECT_NAME}
    sensor.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )

target_link_libraries(${PROJECT_NAME}
    PUBLIC  
    sensor_pub_sub 
    utils 
    fluxlog
    booster_robotics_sdk
    time
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
