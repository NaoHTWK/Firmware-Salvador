cmake_minimum_required(VERSION 3.22)

project(team_strategy)

add_library(${PROJECT_NAME}
    moveball_strategy.cpp
    robocup_strategy.cpp
    stay_strategy.cpp
    team_strategy.cpp
    team_strategy_factory.cpp
    features/position_util.cpp
    features/ready_positioning.cpp
    standard_situations/penalty_kick.cpp
    standard_situations/line_defender.cpp
    standard_situations/field_defender.cpp
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/features ${CMAKE_CURRENT_SOURCE_DIR}/standard_situations
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC loguru team_com_manager Orders pub_sub time utils
           SoccerField dribbling_direction team_strategy_pub_sub)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
