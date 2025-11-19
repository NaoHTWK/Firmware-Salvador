add_library(sim_mock STATIC sim_mock.cpp)

target_include_directories(sim_mock PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

#install(TARGETS sim_mock)

