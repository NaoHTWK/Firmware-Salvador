project(VisionPrimitives)

add_library(${PROJECT_NAME}
    lineedge.cpp
)

target_link_libraries(${PROJECT_NAME} utils)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        .
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
