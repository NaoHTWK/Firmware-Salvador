project(parameter_tuner)

add_library(${PROJECT_NAME}
    parameter_tuner.cpp
    parameter_tuner.h
)

target_link_libraries(${PROJECT_NAME} utils fluxlog ) 
target_include_directories(${PROJECT_NAME} PUBLIC .)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
