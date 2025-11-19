project(cam_pose)

add_library(${PROJECT_NAME}
    cam_pose.cpp
)

target_link_libraries(${PROJECT_NAME} utils fluxlog)
target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
