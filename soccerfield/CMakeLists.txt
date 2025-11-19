project(SoccerField)

add_library(${PROJECT_NAME}
    soccerfield.h
    soccerfield.cpp
    soccerfielddefinitions.h
    soccerfielddefinitions.cpp
    soccerfieldinstance.h
    soccerfieldinstance.cpp
)

target_link_libraries(${PROJECT_NAME} utils)
target_include_directories(${PROJECT_NAME} PUBLIC .)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
