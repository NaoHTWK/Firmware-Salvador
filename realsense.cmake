

message(STATUS "Using prebuilt librealsense2 from sysroot: ${CMAKE_PREFIX_PATH}")

find_library(REALSENSE2_LIBRARY
  NAMES realsense2 librealsense2 librealsense2.so
  PATHS ${CMAKE_PREFIX_PATH}/lib ${CMAKE_PREFIX_PATH}/lib/aarch64-linux-gnu
  NO_DEFAULT_PATH
)

if(NOT REALSENSE2_LIBRARY)
  message(FATAL_ERROR "librealsense2 not found under ${CMAKE_PREFIX_PATH}. Ensure it is installed in the toolchain sysroot.")
endif()

add_library(realsense2 SHARED IMPORTED GLOBAL)
set_target_properties(realsense2 PROPERTIES IMPORTED_LOCATION ${REALSENSE2_LIBRARY})
target_include_directories(realsense2 INTERFACE ${CMAKE_PREFIX_PATH}/include)
