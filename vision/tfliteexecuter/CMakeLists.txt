project(tfliteexecuter)

set(LIBS
    stdc++
    rt
    m
    pthread
)

add_library(${PROJECT_NAME} SHARED
    tfliteexecuter.cpp
    tfliteexecuter.h
)

target_include_directories(${PROJECT_NAME} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    tensorflow-lite
    XNNPACK
    ruy
    pthreadpool
    cpuinfo
    clog
    farmhash
    flatbuffers
    fft2d_fftsg
    fft2d_fftsg2d
    ${LIBS}
    Boost::filesystem
    ${CMAKE_DL_LIBS}
)

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
