project(MultiTargetTracker)

add_library(${PROJECT_NAME}
    multi_target_tracker.cpp
    multi_target_tracker.h
    ball_tracker.cpp
    ball_tracker.h
)

target_link_libraries(
    ${PROJECT_NAME} utils SoccerField fluxlog pub_sub time multi_target_tracker_pub_sub
    cam_pose camera_pub_sub localization_pub_sub vision_pub_sub fluxlog SoccerField)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        .
    )
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
