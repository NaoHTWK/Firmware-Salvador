cmake_minimum_required(VERSION 3.22)
project(TeamStrategyDebugger)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(RERUN_VERSION "0.23.3")
add_compile_definitions(
  RERUN_VERSION="${RERUN_VERSION}"
)

find_package(OpenGL REQUIRED)

include(FetchContent)
FetchContent_Declare(loguru
    GIT_REPOSITORY "https://github.com/emilk/loguru"
    GIT_TAG "4adaa185883e3c04da25913579c451d3c32cfac1"
)
set(LOGURU_WITH_STREAMS TRUE)
FetchContent_MakeAvailable(loguru)

add_subdirectory(../utils/pub_sub pub_sub)
add_subdirectory(../orders orders)
add_subdirectory(../soccerfield soccerfield)
add_subdirectory(../fluxlog fluxlog)
add_subdirectory(../utils/time time)
add_subdirectory(../utils/dribbling_direction dribbling_direction)
add_subdirectory(../localization/pub_sub localization_pub_sub)
add_subdirectory(../team_com/manager team_com_manager)
add_subdirectory(../team_com/pub_sub team_com_pub_sub)
add_subdirectory(../team_com/network team_com_network)
add_subdirectory(../game_controller/pub_sub game_controller_pub_sub)
add_subdirectory(../sensor/pub_sub sensor_pub_sub)
add_subdirectory(../utils utils)
add_subdirectory(../team_strategy team_strategy)
add_subdirectory(../team_strategy/pub_sub team_strategy_pub_sub)
add_subdirectory(../multi_target_tracker/pub_sub multi_target_tracker_pub_sub)

set(Boost_SELECTED_LIBS system thread process interprocess filesystem)

FetchContent_Declare(
    Boost
    URL https://github.com/boostorg/boost/releases/download/boost-1.88.0/boost-1.88.0-cmake.tar.xz
)
FetchContent_MakeAvailable(Boost)

set(IGG_GLFW_TAG "3.4")
set(IGG_IMGUI_TAG "v1.92.0")
set(IGG_GLM_TAG "1.0.1")
FetchContent_Declare(
        imgui-glfw-glad-glm
        GIT_REPOSITORY https://github.com/cmmw/imgui-glfw-glad-glm.git
        GIT_TAG v4.2.0
)

FetchContent_MakeAvailable(imgui-glfw-glad-glm)



# Use system Arrow library instead of building a custom one
# This avoids CMake compatibility issues with Arrow's outdated dependencies
set(RERUN_DOWNLOAD_AND_BUILD_ARROW OFF)

FetchContent_Declare(rerun_sdk
  URL https://github.com/rerun-io/rerun/releases/download/${RERUN_VERSION}/rerun_cpp_sdk.zip
)
FetchContent_MakeAvailable(rerun_sdk)

# Add the main visualizer executable
add_executable(visualizer visualizer.cpp soccer_field_visualizer.cpp)
target_link_libraries(visualizer PRIVATE
    SoccerField
    imgui
    glad
    glfw
    glm
    OpenGL::GL
    Boost::interprocess
    Boost::system
    game_controller_pub_sub
    team_com_network
    fluxlog
)
target_include_directories(visualizer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${glad_SOURCE_DIR}/include
)

# Add the main debugger executable
add_executable(debugger debugger.cpp soccer_field_visualizer.cpp)
target_link_libraries(debugger PRIVATE
    Boost::system
    Boost::thread
    Boost::interprocess
    Boost::process
    Boost::filesystem
    Boost::serialization
    SoccerField
    imgui
    glad
    glfw
    glm
    OpenGL::GL
    game_controller_pub_sub
)
target_include_directories(debugger PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${glad_SOURCE_DIR}/include
)

# Add the robot process executable
add_executable(robot_process robot_process.cpp sim_time.cpp)
target_link_libraries(robot_process PRIVATE team_strategy Boost::interprocess Boost::serialization)
target_include_directories(robot_process PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB TACTIC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../deploy/tactics/*.conf")
add_custom_target(copy_tactics ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_CURRENT_SOURCE_DIR}/../deploy/tactics"
    "${CMAKE_BINARY_DIR}/tactics"
    COMMENT "Copying tactics to build directory"
    DEPENDS ${TACTIC_FILES}
    VERBATIM
)
