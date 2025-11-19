cmake_minimum_required(VERSION 3.22)

project(utils)

add_library(${PROJECT_NAME} STATIC
    localization_utils.cpp
    quaternion.cpp
    tracked_object.cpp
    point_2d.cpp
    vector_field.cpp
    tts.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC loguru)

target_include_directories(${PROJECT_NAME} 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
