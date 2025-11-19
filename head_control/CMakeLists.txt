project(head_control)

add_library(${PROJECT_NAME}
        head_control.cpp
        head_control.h
)
target_link_libraries(
        ${PROJECT_NAME}
        utils
        sensor_pub_sub
        localization_pub_sub
        booster_robotics_sdk
        camera_pub_sub
        vision_pub_sub
        multi_target_tracker_pub_sub
        cam_pose
)
target_include_directories(${PROJECT_NAME} PUBLIC .)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
