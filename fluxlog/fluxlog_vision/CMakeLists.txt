cmake_minimum_required(VERSION 3.22)

project(fluxlog_vision)

find_package(OpenCV REQUIRED)

message(STATUS "OPENCV located at: ${OpenCV_INCLUDE_DIRS}")

add_library(${PROJECT_NAME} STATIC fluxlog_vision.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC fluxlog ${OpenCV_LIBS} )
target_include_directories(${PROJECT_NAME} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR} 
    ${OpenCV_INCLUDE_DIRS} 
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
