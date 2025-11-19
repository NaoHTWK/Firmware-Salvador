cmake_minimum_required(VERSION 3.22)

project(odometer)

add_library(${PROJECT_NAME}
    odometer_processor.cpp
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_SOURCE_DIR}
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC utils time motion_pub_sub sensor_pub_sub fluxlog)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
