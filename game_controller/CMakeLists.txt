cmake_minimum_required(VERSION 3.22)

project(game_controller)

add_library(${PROJECT_NAME}
    game_controller.cpp
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC game_controller_pub_sub gc_humanoid Boost::system fluxlog
    PRIVATE utils)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
