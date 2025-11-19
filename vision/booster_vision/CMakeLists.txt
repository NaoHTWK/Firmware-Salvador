cmake_minimum_required(VERSION 3.22)

project(vision_booster_vision) # Add CUDA to the project languages

find_package(OpenCV REQUIRED)

if (${ROBOT_MODEL} STREQUAL "k1")
  add_subdirectory(trt_6.2)
elseif(${ROBOT_MODEL} STREQUAL "t1")
  add_subdirectory(trt_6.2)
elseif(${ROBOT_MODEL} STREQUAL "k1d")
  add_subdirectory(trt_6.2)
elseif(${ROBOT_MODEL} STREQUAL "k1z")
  add_subdirectory(trt_6.2)
endif()

add_library(vision_booster_vision
  config.h
  detector.cpp
  detector.h
)

target_include_directories(vision_booster_vision PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(vision_booster_vision PUBLIC
  trt_detect
  ${OpenCV_LIBS}
)

if (${ROBOT_MODEL} STREQUAL "k1")
  target_compile_definitions(vision_booster_vision PUBLIC TRT_6_2)
elseif(${ROBOT_MODEL} STREQUAL "t1")
  target_compile_definitions(vision_booster_vision PUBLIC TRT_6_2)
endif()
