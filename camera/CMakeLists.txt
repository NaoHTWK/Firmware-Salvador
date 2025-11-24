cmake_minimum_required(VERSION 3.22)

project(camera)

add_library(camera
  camera.h
  camera.cpp
)

target_include_directories(camera INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

if (CAMERA_MODEL STREQUAL "zed2")
  add_subdirectory(zed2)
  target_link_libraries(camera PRIVATE camera_zed2)
elseif (CAMERA_MODEL STREQUAL "realsense")
  add_subdirectory(realsense)
  target_link_libraries(camera PRIVATE camera_realsense)
elseif (CAMERA_MODEL STREQUAL "booster_camera")
    add_subdirectory(booster_camera)
    target_link_libraries(camera PRIVATE camera_booster_camera)
    target_link_libraries(camera_booster_camera PRIVATE rt)
else()
  message(FATAL_ERROR "Unknown camera model: '${CAMERA_MODEL}'. Use 'zed2', 'realsense' or 'booster_camera'.")
endif()

target_link_libraries(camera PUBLIC
  camera_pub_sub
  sensor_pub_sub
  cam_pose
  time
)
