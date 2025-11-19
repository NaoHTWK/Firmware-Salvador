cmake_minimum_required(VERSION 3.16)
project(Vision)

find_package(OpenCV REQUIRED)

option(BUILD_BENCHMARK "Build the benchmark." OFF)
option(VISION_ACTIVATE_VISUALIZATION "Build with visualization." OFF)
option(BUILD_DEMO_APP "Build the demo application" OFF)

set(LIBS
    stdc++
    rt
    m
    pthread
    utils
    VisionPrimitives
)

add_library(${PROJECT_NAME} STATIC
    base_detector.h
    field_border_detector.cpp
    field_border_detector.h
    field_color_detector.cpp
    field_color_detector.h
    image_preprocessor.cpp
    image_preprocessor.h
    htwk_vision.cpp
    htwk_vision.h
    line_detector.cpp
    line_detector.h
    range_check.h
    region_classifier.cpp
    plane_detection.cpp
)

if(VISION_ACTIVATE_VISUALIZATION)
    target_compile_definitions(${PROJECT_NAME} PUBLIC VISION_ACTIVATE_VISUALIZATION)
endif()

add_subdirectory(tfliteexecuter)

target_link_libraries(${PROJECT_NAME} PRIVATE
    utils
    VisionPrimitives
    tfliteexecuter
    vision_booster_vision
    vision_obstacle_detection_legacy
    vision_pub_sub
    camera_pub_sub
    pub_sub
    fluxlog
    time
    cam_pose

    ${LIBS}
    ${CMAKE_DL_LIBS}
    ${OpenCV_LIBS}
)

target_include_directories(${PROJECT_NAME} 
    PUBLIC . 
    ${CMAKE_CURRENT_SOURCE_DIR}/tfliteexecuter
    PUBLIC /l4t/targetfs/usr/local/cuda/include
    PUBLIC /l4t/targetfs/usr/include/opencv4
    PUBLIC ${OpenCV_INCLUDE_DIRS} 
    )
    
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)


