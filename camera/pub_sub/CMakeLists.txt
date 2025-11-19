cmake_minimum_required(VERSION 3.22)

project(camera_pub_sub)

find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME}
    image.h
    camera_pub_sub.h
    camera_pub_sub.cpp
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC /l4t/targetfs/usr/include/opencv4
    PUBLIC ${OpenCV_INCLUDE_DIRS} 
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC pub_sub utils sensor_pub_sub fluxlog fluxlog_vision cam_pose)

target_include_directories(${PROJECT_NAME} 
    PUBLIC /l4t/targetfs/usr/include/opencv4
    PUBLIC ${OpenCV_INCLUDE_DIRS} 
)

target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)
