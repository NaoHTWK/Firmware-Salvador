cmake_minimum_required(VERSION 3.22)

project(vision_pub_sub)

add_library(${PROJECT_NAME}
    vision_pub_sub.cpp
    object_hypothesis.h
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_SOURCE_DIR}
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC pub_sub utils VisionPrimitives)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
