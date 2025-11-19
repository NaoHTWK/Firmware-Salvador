# TensorFlow Lite and dependencies from sysroot only

include_directories(
    ${CMAKE_PREFIX_PATH}/include/tensorflow
    ${CMAKE_PREFIX_PATH}/include/tensorflow/lite
)

add_library(XNNPACK STATIC IMPORTED)
set_target_properties(XNNPACK PROPERTIES IMPORTED_LOCATION ${CMAKE_PREFIX_PATH}/lib/libXNNPACK.a)

target_include_directories(XNNPACK INTERFACE ${CMAKE_PREFIX_PATH}/include)

add_library(tensorflow-lite STATIC IMPORTED)
set_target_properties(tensorflow-lite PROPERTIES IMPORTED_LOCATION ${CMAKE_PREFIX_PATH}/lib/libtensorflow-lite.a)

if(NOT TARGET pthreadpool)
  add_library(pthreadpool STATIC IMPORTED)
  set_target_properties(pthreadpool PROPERTIES IMPORTED_LOCATION ${CMAKE_PREFIX_PATH}/lib/libpthreadpool.a)
endif()

if(NOT TARGET cpuinfo)
  add_library(cpuinfo STATIC IMPORTED)
  set_target_properties(cpuinfo PROPERTIES IMPORTED_LOCATION ${CMAKE_PREFIX_PATH}/lib/libcpuinfo.a)
endif()

if(NOT TARGET flatbuffers)
  add_library(flatbuffers STATIC IMPORTED)
  set_target_properties(flatbuffers PROPERTIES IMPORTED_LOCATION ${CMAKE_PREFIX_PATH}/lib/libflatbuffers.a)
endif()

if(NOT TARGET farmhash)
  add_library(farmhash STATIC IMPORTED)
  set_target_properties(farmhash PROPERTIES IMPORTED_LOCATION ${CMAKE_PREFIX_PATH}/lib/libfarmhash.a)
endif()

if(NOT TARGET fft2d_fftsg)
  add_library(fft2d_fftsg STATIC IMPORTED)
  set_target_properties(fft2d_fftsg PROPERTIES IMPORTED_LOCATION ${CMAKE_PREFIX_PATH}/lib/libfft2d_fftsg.a)
endif()
if(NOT TARGET fft2d_fftsg2d)
  add_library(fft2d_fftsg2d STATIC IMPORTED)
  set_target_properties(fft2d_fftsg2d PROPERTIES IMPORTED_LOCATION ${CMAKE_PREFIX_PATH}/lib/libfft2d_fftsg2d.a)
endif()

# CLOG for cpuinfo logging
if(NOT TARGET clog)
  add_library(clog STATIC IMPORTED)
  set_target_properties(clog PROPERTIES IMPORTED_LOCATION ${CMAKE_PREFIX_PATH}/lib/libclog.a)
endif()

if(NOT TARGET ruy)
  add_library(ruy INTERFACE IMPORTED)
  target_link_libraries(ruy INTERFACE
    -Wl,--start-group
    ${CMAKE_PREFIX_PATH}/lib/libruy_allocator.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_apply_multiplier.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_block_map.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_blocking_counter.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_context.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_context_get_ctx.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_cpuinfo.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_ctx.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_denormal.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_frontend.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_have_built_path_for_avx.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_have_built_path_for_avx2_fma.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_have_built_path_for_avx512.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_kernel_arm.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_kernel_avx.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_kernel_avx2_fma.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_kernel_avx512.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_pack_arm.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_pack_avx.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_pack_avx2_fma.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_pack_avx512.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_prepacked_cache.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_prepare_packed_matrices.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_system_aligned_alloc.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_thread_pool.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_trmul.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_tune.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_wait.a
    ${CMAKE_PREFIX_PATH}/lib/libruy_profiler_instrumentation.a
    -Wl,--end-group
  )
endif()
