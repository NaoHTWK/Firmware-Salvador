cmake_minimum_required(VERSION 3.22)

project(team_com_manager)

add_library(${PROJECT_NAME}
    tc_manager.cpp
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC team_com_pub_sub loguru pub_sub game_controller_pub_sub time game_controller_pub_sub multi_target_tracker_pub_sub fluxlog utils)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
