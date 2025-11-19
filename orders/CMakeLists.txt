project(Orders)

add_library(${PROJECT_NAME}
    noorder.h
    order.cpp
    order.h
    walkrelativeorder.h
    walkcustomorder.h
    moveballorder.h
    shootorder.h
)
target_link_libraries(${PROJECT_NAME} utils SoccerField fluxlog)
target_include_directories(${PROJECT_NAME} PUBLIC .)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
