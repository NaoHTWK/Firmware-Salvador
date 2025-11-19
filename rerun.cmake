set(RERUN_VERSION "0.23.3")

message(STATUS "Using prebuilt rerun SDK from Docker image")

# Create imported target for rerun SDK (sysroot-only, deterministic)
add_library(rerun_sdk STATIC IMPORTED)
set_target_properties(rerun_sdk PROPERTIES
  IMPORTED_LOCATION ${CMAKE_PREFIX_PATH}/lib/librerun_sdk.a
)
target_include_directories(rerun_sdk INTERFACE ${CMAKE_PREFIX_PATH}/include)

add_compile_definitions(
  RERUN_VERSION="${RERUN_VERSION}"
)

# Debug: list Arrow/Parquet libs visible in sysroot
file(GLOB _arrow_libs
  "${CMAKE_PREFIX_PATH}/lib/libarrow*"
  "${CMAKE_PREFIX_PATH}/lib/libparquet*"
)
if(_arrow_libs)
  message(STATUS "Arrow/Parquet libs in sysroot: ${_arrow_libs}")
else()
  message(STATUS "No Arrow/Parquet libs found under ${CMAKE_PREFIX_PATH}/lib at configure time")
endif()

# Propagate Arrow static libs when available so linkers resolve rerun's Arrow symbols
set(_arrow_lib_dir "${CMAKE_PREFIX_PATH}/lib")
set(_arrow_static_lib "${_arrow_lib_dir}/libarrow.a")
set(_arrow_bundled_deps_lib "${_arrow_lib_dir}/libarrow_bundled_dependencies.a")
set(_arrow_c_lib "${_arrow_lib_dir}/libarrow_c.a")
set(_parquet_lib "${_arrow_lib_dir}/libparquet.a")
set(_rerun_c_lib "${CMAKE_PREFIX_PATH}/lib/librerun_c.a")

set(_link_libs)
if(EXISTS "${_arrow_static_lib}")
  list(APPEND _link_libs "${_arrow_static_lib}")
endif()
if(EXISTS "${_arrow_bundled_deps_lib}")
  list(APPEND _link_libs "${_arrow_bundled_deps_lib}")
endif()
if(EXISTS "${_arrow_c_lib}")
  list(APPEND _link_libs "${_arrow_c_lib}")
endif()
if(EXISTS "${_parquet_lib}")
  list(APPEND _link_libs "${_parquet_lib}")
endif()
if(EXISTS "${_rerun_c_lib}")
  list(APPEND _link_libs "${_rerun_c_lib}")
endif()

if(_link_libs)
  message(STATUS "Linking rerun SDK with Arrow static libs: ${_link_libs}")
  # Use link-group to help static resolution in case of cyclic deps inside Arrow
  target_link_libraries(rerun_sdk INTERFACE -Wl,--start-group ${_link_libs} -Wl,--end-group)
else()
  message(STATUS "Arrow static libs not found in ${_arrow_lib_dir}; proceeding without explicit Arrow link")
endif()