cmake_minimum_required(VERSION 3.22)

project(head_control)

add_library(${PROJECT_NAME}
    head_control.cpp
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC motion_pub_sub game_controller_pub_sub utils localization_pub_sub time
    vision_pub_sub SoccerField sensor_pub_sub team_strategy_pub_sub)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
