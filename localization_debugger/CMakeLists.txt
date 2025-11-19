cmake_minimum_required(VERSION 3.22)
include(FetchContent)
project(LocalizationDebugger)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(RERUN_VERSION "0.23.3")
add_compile_definitions(CAMERA_WIDTH=1280)
add_compile_definitions(CAMERA_HEIGHT=720)
add_compile_definitions(CAMERA_FOV_WIDTH=1.9198621771937625) # 110 degrees
add_compile_definitions(CAMERA_FOV_HEIGHT=1.221730476396) # 70 degrees
add_compile_definitions(
  RERUN_VERSION="${RERUN_VERSION}"
)
add_compile_definitions(ROBOT_MODEL_K1)
add_compile_definitions(ZED2_CAMERA)

set(Boost_SELECTED_LIBS system thread process interprocess filesystem)

FetchContent_Declare(
    Boost
    URL https://github.com/boostorg/boost/releases/download/boost-1.88.0/boost-1.88.0-cmake.tar.xz
)
FetchContent_MakeAvailable(Boost)

find_package(OpenGL REQUIRED)

FetchContent_Declare(loguru
    GIT_REPOSITORY "https://github.com/emilk/loguru"
    GIT_TAG "4adaa185883e3c04da25913579c451d3c32cfac1"
)
set(LOGURU_WITH_STREAMS TRUE)
FetchContent_MakeAvailable(loguru)

add_subdirectory(../utils/pub_sub pub_sub)
add_subdirectory(../soccerfield soccerfield)
add_subdirectory(../fluxlog fluxlog)
add_subdirectory(../utils/time time)
add_subdirectory(../localization/pub_sub localization_pub_sub)
add_subdirectory(../game_controller/pub_sub game_controller_pub_sub)
add_subdirectory(../sensor/pub_sub sensor_pub_sub)
add_subdirectory(../utils utils)
add_subdirectory(../utils/cam_pose utils_cam_pose)
add_subdirectory(../utils/vision_primitives utils_vision_primitives)
add_subdirectory(../vision/pub_sub vision_pub_sub)
add_subdirectory(../parameter_tuner parameter_tuner)
add_subdirectory(../localization localization)
add_subdirectory(../orders orders)
add_subdirectory(../utils/time/real_time real_time)

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

# Add the main debugger executable
add_executable(debugger debugger.cpp soccer_field_visualizer.cpp)
target_link_libraries(debugger PRIVATE
    localization
    SoccerField
    imgui
    glad
    glfw
    glm
    utils
    real_time
    OpenGL::GL
    Boost::system
    Boost::thread
    Boost::interprocess
    Boost::process
    Boost::filesystem
    Boost::serialization
    game_controller_pub_sub
)
target_include_directories(debugger PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${glad_SOURCE_DIR}/include
)
