cmake_minimum_required(VERSION 3.5)
project(camera_drobotics)

find_package(OpenCV REQUIRED)

add_library(camera_drobotics 
    drobotics.cpp
    drobotics.h
)

target_include_directories(camera_drobotics
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(camera_drobotics
    PUBLIC
        utils
        camera_pub_sub
        fluxlog_vision
        fluxlog
        ${OpenCV_LIBS}
    PRIVATE
        rt  # for shm_open, sem_open
        pthread  # for semaphores
)
