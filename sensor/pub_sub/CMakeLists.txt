cmake_minimum_required(VERSION 3.22)

project(sensor_pub_sub)

add_library(${PROJECT_NAME}
    sensor_pub_sub.cpp
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC pub_sub utils loguru fluxlog)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
