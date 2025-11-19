project(near_obstacle_tracker)

add_library(${PROJECT_NAME}
    near_obstacle_tracker.cpp
)
target_link_libraries(${PROJECT_NAME} utils vision_pub_sub time sensor_pub_sub fluxlog fluxlog_vision near_obstacle_tracker_pub_sub team_strategy_pub_sub)
target_include_directories(${PROJECT_NAME} PUBLIC .)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
