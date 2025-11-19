cmake_minimum_required(VERSION 3.22)

project(team_com_pub_sub)

add_library(${PROJECT_NAME}
    tc_pub_sub.cpp
    tc_data.cpp
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC pub_sub game_controller_pub_sub utils multi_target_tracker_pub_sub sensor_pub_sub time localization_pub_sub)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
