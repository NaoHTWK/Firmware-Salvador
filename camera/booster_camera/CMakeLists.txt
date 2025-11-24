cmake_minimum_required(VERSION 3.5)
project(camera_booster_camera)

find_package(OpenCV REQUIRED)

add_library(camera_booster_camera
    booster_camera.cpp
    booster_camera.h
)

target_include_directories(camera_booster_camera
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(camera_booster_camera
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
