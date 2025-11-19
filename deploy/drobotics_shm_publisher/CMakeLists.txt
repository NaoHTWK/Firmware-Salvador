cmake_minimum_required(VERSION 3.5)
project(drobotics_shm_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(publisher src/publisher.cpp)

target_include_directories(publisher
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(publisher
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS
  publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
