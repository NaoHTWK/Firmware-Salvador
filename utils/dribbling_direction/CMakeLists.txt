cmake_minimum_required(VERSION 3.22)

project(dribbling_direction)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} INTERFACE utils SoccerField)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)
