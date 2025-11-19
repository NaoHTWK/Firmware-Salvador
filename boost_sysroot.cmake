# Boost discovery against sysroot

# Prefer CMake's FindBoost module over Boost's CMake config in the sysroot
set(Boost_NO_BOOST_CMAKE ON)
set(Boost_NO_WARN_NEW_VERSIONS ON)

# Hint FindBoost to the baked-in sysroot directories
set(BOOST_ROOT ${CMAKE_PREFIX_PATH})
set(BOOST_INCLUDEDIR ${CMAKE_PREFIX_PATH}/include)
set(BOOST_LIBRARYDIR ${CMAKE_PREFIX_PATH}/lib)

# Use shared, multithreaded libs
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)

set(Boost_SELECTED_LIBS program_options filesystem serialization)
find_package(Boost 1.89.0 REQUIRED COMPONENTS ${Boost_SELECTED_LIBS})

# Header-only alias targets used by the codebase
if(NOT TARGET Boost::system)
  add_library(Boost::system INTERFACE IMPORTED)
  target_include_directories(Boost::system INTERFACE ${Boost_INCLUDE_DIRS})
endif()
if(NOT TARGET Boost::asio)
  add_library(Boost::asio INTERFACE IMPORTED)
  target_include_directories(Boost::asio INTERFACE ${Boost_INCLUDE_DIRS})
endif()
