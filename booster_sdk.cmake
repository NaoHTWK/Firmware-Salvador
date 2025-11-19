set(BOOSTER_SDK_DIR ${CMAKE_PREFIX_PATH})

add_library(booster_robotics_sdk STATIC IMPORTED)

target_include_directories(booster_robotics_sdk INTERFACE
  ${BOOSTER_SDK_DIR}/include/
)


add_library(fastcdr SHARED IMPORTED)
add_library(fastrtps SHARED IMPORTED)
add_library(foonathan_memory-0.7.3 STATIC IMPORTED)

# For cross-compiled arm64 libraries
set_target_properties(booster_robotics_sdk PROPERTIES
  IMPORTED_LOCATION ${BOOSTER_SDK_DIR}/lib/libbooster_robotics_sdk.a
)

set_target_properties(fastcdr PROPERTIES
  IMPORTED_LOCATION ${BOOSTER_SDK_DIR}/lib/libfastcdr.so
)

set_target_properties(fastrtps PROPERTIES
  IMPORTED_LOCATION ${BOOSTER_SDK_DIR}/lib/libfastrtps.so
)

set_target_properties(foonathan_memory-0.7.3 PROPERTIES
  IMPORTED_LOCATION ${BOOSTER_SDK_DIR}/lib/libfoonathan_memory-0.7.3.a
)

message(STATUS "Booster-SDK is ${BOOSTER_SDK_DIR}/lib/libbooster_robotics_sdk.a")

install(FILES
  ${BOOSTER_SDK_DIR}/lib/libfastcdr.so
  ${BOOSTER_SDK_DIR}/lib/libfastrtps.so
  ${BOOSTER_SDK_DIR}/lib/libfoonathan_memory-0.7.3.a
  DESTINATION lib/)

