project(localization)

add_library(${PROJECT_NAME}
    localization.cpp
    localization.h
)
target_link_libraries(
    ${PROJECT_NAME}
    utils
    SoccerField
    VisionPrimitives
    vision_pub_sub
    fluxlog 
    sensor_pub_sub
    localization_pub_sub
    parameter_tuner
    cam_pose
    time
    game_controller_pub_sub
    )
target_include_directories(${PROJECT_NAME} PUBLIC . )
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
