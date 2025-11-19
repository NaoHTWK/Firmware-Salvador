project(real_time)

add_library(${PROJECT_NAME}
    real_time.cpp
)
target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC time)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)