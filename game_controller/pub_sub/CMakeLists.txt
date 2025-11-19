cmake_minimum_required(VERSION 3.22)

project(game_controller_pub_sub)

add_library(${PROJECT_NAME}
    gc_pub_sub.cpp
    gc_state.cpp
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC pub_sub)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
