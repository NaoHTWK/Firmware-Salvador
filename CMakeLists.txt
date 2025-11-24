cmake_minimum_required(VERSION 3.22)

set(ROBOT_MODEL "k1" CACHE STRING "Robot model to use. 'k1' or 't1'")
set(ROBOT_SUBMODEL "" CACHE STRING "Robot submodel variant (optional)")

option(SIMULATION_MODE "enable simulation mode" OFF)

if (${ROBOT_MODEL} STREQUAL "k1")
  set(CAMERA_MODEL "zed2" CACHE STRING "Camera model to use, options: 'zed2' or 'realsense'. Default is 'zed2'.")
#elseif (${ROBOT_MODEL} STREQUAL "k1")
#  set(CAMERA_MODEL "booster_camera" CACHE STRING "Camera model to use, options: 'zed2' or 'booster_camera'. Default is 'zed2'.")
else()
  set(CAMERA_MODEL "realsense" CACHE STRING "Camera model to use, options: 'zed2' or 'realsense'. Default is 'realsense'.")
endif()

if(CAMERA_MODEL STREQUAL "zed2")
  add_compile_definitions(CAMERA_WIDTH=1280)
  add_compile_definitions(CAMERA_HEIGHT=720)
  add_compile_definitions(DEPTH_CAMERA_WIDTH=1280)
  add_compile_definitions(DEPTH_CAMERA_HEIGHT=720)
  add_compile_definitions(CAMERA_FOV_WIDTH=1.9198621771937625) # 110 degrees
  add_compile_definitions(CAMERA_FOV_HEIGHT=1.221730476396) # 70 degrees

elseif(CAMERA_MODEL STREQUAL "realsense")
  add_compile_definitions(CAMERA_HEIGHT=800)
  add_compile_definitions(CAMERA_WIDTH=1280)
  add_compile_definitions(DEPTH_CAMERA_HEIGHT=720)
  add_compile_definitions(DEPTH_CAMERA_WIDTH=1280)
  add_compile_definitions(CAMERA_FOV_WIDTH=1.57079633) # 90 degrees
  add_compile_definitions(CAMERA_FOV_HEIGHT=1.13446401) # 65 degrees
  #this will not change the rs2 config yet.
  message(STATUS "Using Realsense camera")
elseif(CAMERA_MODEL STREQUAL "booster_camera")
    add_compile_definitions(CAMERA_WIDTH=544)
    add_compile_definitions(CAMERA_HEIGHT=448)
    add_compile_definitions(DEPTH_CAMERA_HEIGHT=448)
    add_compile_definitions(DEPTH_CAMERA_WIDTH=544)
    add_compile_definitions(CAMERA_FOV_WIDTH=1.8465583)
    add_compile_definitions(CAMERA_FOV_HEIGHT=1.6559684)
else()
    message(FATAL_ERROR "Unknown camera model: '${CAMERA_MODEL}'. Use 'zed2', 'realsense', or 'booster_camera'.")
endif()

if(SIMULATION_MODE)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/sim_toolchains.cmake CACHE STRING "")
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
else()
  # Set toolchain file before project definition
  # This must be done before the project command
  set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/toolchains.cmake CACHE STRING "")
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

include(FetchContent)
include(ExternalProject)

# Enable colored compiler outputrealsense
add_compile_options(-fdiagnostics-color=always)

project(fw_salvador LANGUAGES C CXX)

if (${ROBOT_MODEL} STREQUAL "k1")
  add_compile_definitions(ROBOT_MODEL_K1)
elseif(${ROBOT_MODEL} STREQUAL "t1")
  add_compile_definitions(ROBOT_MODEL_T1)
endif()

if(${ROBOT_SUBMODEL} STREQUAL "T1_robocup_version")
  add_compile_definitions(ROBOT_SUBMODEL_T1_ROBOCUP_VERSION)
  message(STATUS "Using robot submodel: ${ROBOT_SUBMODEL}")
endif()

if(SIMULATION_MODE)
    message(STATUS "compiling simulation mode")
    add_compile_definitions(SIMULATION_MODE)
else()
    message(STATUS "compiling real robot mode")
    if(CAMERA_MODEL STREQUAL "zed2")
        message(STATUS "Using ZED2 camera")
        add_compile_definitions(ZED2_CAMERA)
    elseif(CAMERA_MODEL STREQUAL "booster_camera")
        message(STATUS "Using Booster camera")
        add_compile_definitions(BOOSTER_CAMERA)
    elseif(CAMERA_MODEL STREQUAL "realsense")
        message(STATUS "Using Realsense camera")
        add_compile_definitions(REALSENSE_CAMERA)
    else()
        message(FATAL_ERROR "Unknown camera model: '${CAMERA_MODEL}'. Use 'zed2', 'booster_camera' or 'realsense'.")
    endif()
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build (Debug, Release, RelWithDebInfo, MinSizeRel)." FORCE)
endif()
message(STATUS "Using build type: ${CMAKE_BUILD_TYPE}")

include(boost_sysroot.cmake)

if(NOT SIMULATION_MODE)
  if (CAMERA_MODEL STREQUAL "realsense")
    include(realsense.cmake)
  endif()
endif()
#find_package(realsense2 REQUIRED)
include(booster_sdk.cmake)

set(HOST_SED_EXECUTABLE "/bin/sed")

include(rerun.cmake)

# loguru is prebuilt in the Docker container
add_library(loguru INTERFACE)
target_include_directories(loguru INTERFACE ${CMAKE_PREFIX_PATH}/include/loguru)
target_link_libraries(loguru INTERFACE ${CMAKE_PREFIX_PATH}/lib/libloguru.a)
target_compile_definitions(loguru INTERFACE LOGURU_WITH_STREAMS=1)

include(tflite_sysroot.cmake)

add_subdirectory(fluxlog fluxlog)
add_subdirectory(fluxlog/fluxlog_vision fluxlog_vision)

add_subdirectory(camera camera)
add_subdirectory(camera/pub_sub camera_pub_sub)

add_subdirectory(sensor/pub_sub sensor_pub_sub)
if(SIMULATION_MODE)
  add_subdirectory(sim_mock)
else()
  add_subdirectory(sensor sensor)
endif()

add_subdirectory(game_controller game_controller)
add_subdirectory(game_controller/humanoid gc_humanoid)
add_subdirectory(game_controller/pub_sub gc_pub_sub)
add_subdirectory(team_com/manager team_com_manager)
add_subdirectory(team_com/network team_com_network)
add_subdirectory(team_com/pub_sub team_com_pub_sub)
add_subdirectory(team_strategy team_strategy)
add_subdirectory(team_strategy/pub_sub team_strategy_pub_sub)
add_subdirectory(utils utils)
add_subdirectory(utils/cam_pose utils_cam_pose)
add_subdirectory(utils/dribbling_direction utils_dribbling_direction)
add_subdirectory(utils/pub_sub utils_pub_sub)
add_subdirectory(utils/time utils_time)
add_subdirectory(utils/time/real_time real_time)
add_subdirectory(utils/vision_primitives utils_vision_primitives)
add_subdirectory(vision vision)
add_subdirectory(vision/booster_vision vision_booster_vision)
add_subdirectory(vision/obstacle_detection_legacy vision_obstacle_detection_legacy)
add_subdirectory(vision/pub_sub vision_pub_sub)
add_subdirectory(near_obstacle_tracker near_obstacle_tracker)
add_subdirectory(near_obstacle_tracker/pub_sub near_obstacle_tracker_pub_sub)
add_subdirectory(soccerfield soccerfield)
add_subdirectory(localization localization)
add_subdirectory(localization/pub_sub localization_pub_sub)
add_subdirectory(multi_target_tracker)
add_subdirectory(multi_target_tracker/pub_sub multi_target_tracker_pub_sub)
add_subdirectory(head_control_nao)
add_subdirectory(agents)
add_subdirectory(policy_executor)
add_subdirectory(orders)
add_subdirectory(odometer)
add_subdirectory(motion_connector)
add_subdirectory(motion_connector/pub_sub)
add_subdirectory(parameter_tuner)


#add_subdirectory(cuda-test)

message(STATUS "System name: ${CMAKE_SYSTEM_NAME}")
message(STATUS "System processor: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "C compiler: ${CMAKE_C_COMPILER}")
message(STATUS "CXX compiler: ${CMAKE_CXX_COMPILER}")

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE
  booster_robotics_sdk
  odometer
  fastcdr
  fastrtps
  foonathan_memory-0.7.3
  fluxlog
  fluxlog_vision
  game_controller
  camera_pub_sub
  camera
  team_com_network
  team_strategy
  Vision
  vision_pub_sub
  vision_booster_vision
  vision_obstacle_detection_legacy
  near_obstacle_tracker
  near_obstacle_tracker_pub_sub
  localization
  sensor_pub_sub
  sensor
  utils
  MultiTargetTracker
  multi_target_tracker_pub_sub
  Orders
  Agents
  head_control
  motion_connector
  real_time
  cam_pose
  Boost::program_options 
  Boost::filesystem
  odometer
)

if (NOT SIMULATION_MODE)
  # Link libraries required only on real robot
  target_link_libraries(${PROJECT_NAME} PUBLIC camera)
else()

  # Link libraries only required in simulation
  target_link_libraries(${PROJECT_NAME} PRIVATE
    sim_mock
  )
  #install(TARGETS ${PROJECT_NAME} flight_log_writer game_controller sim_mock)
  
endif()

target_compile_features(fw_salvador PUBLIC cxx_std_20)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Building for: ${CMAKE_SYSTEM_PROCESSOR} on ${CMAKE_SYSTEM_NAME}")

function(get_all_targets var)
    set(targets)
    get_all_targets_recursive(targets ${CMAKE_CURRENT_SOURCE_DIR})
    set(${var} ${targets} PARENT_SCOPE)
endfunction()

macro(get_all_targets_recursive targets dir)
    get_property(subdirectories DIRECTORY ${dir} PROPERTY SUBDIRECTORIES)
    foreach(subdir ${subdirectories})
        get_all_targets_recursive(${targets} ${subdir})
    endforeach()

    get_property(current_targets DIRECTORY ${dir} PROPERTY BUILDSYSTEM_TARGETS)
    list(APPEND ${targets} ${current_targets})
endmacro()

get_all_targets(all_targets)
#message("All targets: ${all_targets}")

# Exclude pywrap from installation
set(CMAKE_INSTALL_EXCLUDE_PATTERN "_pywrap_tensorflow_interpreter_wrapper.so")

foreach(target ${all_targets})
  get_property(target_type TARGET ${target} PROPERTY TYPE)
  #message(STATUS "Target ${target} is a ${target_type}")
  if (NOT ${target} STREQUAL "flatc" AND NOT ${target} STREQUAL "flathash" AND NOT ${target} STREQUAL "benchmark_model" AND NOT ${target} STREQUAL "label_image" AND ${target_type} STREQUAL EXECUTABLE)
    install(TARGETS ${target})
  elseif(${target_type} STREQUAL SHARED_LIBRARY)
    # Skip the Python wrapper
    get_target_property(target_name ${target} NAME)
    if(NOT target_name STREQUAL "_pywrap_tensorflow_interpreter_wrapper")
      install(TARGETS ${target})
    else()
      message(STATUS "Skipping installation of ${target}")
    endif()
  endif()
endforeach()

# Final override to prevent installing the Python wrapper file
install(CODE "
  # Remove any install rule for _pywrap_tensorflow_interpreter_wrapper.so
  if(\"${CMAKE_INSTALL_MANIFEST_FILES}\" MATCHES \"_pywrap_tensorflow_interpreter_wrapper.so\")
    message(STATUS \"Removing Python wrapper from install manifest\")
    string(REGEX REPLACE \"[^\n]*_pywrap_tensorflow_interpreter_wrapper.so[^\n]*\n?\" \"\" 
           CMAKE_INSTALL_MANIFEST_FILES \"${CMAKE_INSTALL_MANIFEST_FILES}\")
    file(WRITE \"${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt\" \"${CMAKE_INSTALL_MANIFEST_FILES}\")
  endif()
")

