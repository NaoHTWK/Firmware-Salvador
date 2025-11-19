cmake_minimum_required(VERSION 3.22)

project(fluxlog)

add_library(${PROJECT_NAME} STATIC logging.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC rerun_sdk loguru SoccerField Orders time)

target_include_directories(${PROJECT_NAME} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR} 
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
