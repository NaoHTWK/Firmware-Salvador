cmake_minimum_required(VERSION 3.22)

project(team_com_network)

add_library(${PROJECT_NAME}
    tc_network.cpp
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC team_com_pub_sub loguru pub_sub Boost::system Boost::serialization Boost::asio)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
