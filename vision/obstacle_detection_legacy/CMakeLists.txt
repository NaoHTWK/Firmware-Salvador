cmake_minimum_required(VERSION 3.22)

project(vision_obstacle_detection_legacy)

find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME}
  obstacle_detector_legacy.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  vision_pub_sub
  camera_pub_sub
  utils
  fluxlog
  cam_pose
  ${OpenCV_LIBS}
)