cmake_minimum_required(VERSION 3.26.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if (MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
endif()

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${ONNXRUNTIME_ROOT}/include)

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${ONNXRUNTIME_ROOT}/lib/onnxruntime_common.lib
                          ${ONNXRUNTIME_ROOT}/lib/onnxruntime_flatbuffers.lib
                          ${ONNXRUNTIME_ROOT}/lib/onnxruntime_framework.lib
                          ${ONNXRUNTIME_ROOT}/lib/onnxruntime_graph.lib
                          ${ONNXRUNTIME_ROOT}/lib/onnxruntime_mlas.lib
                          ${ONNXRUNTIME_ROOT}/lib/onnxruntime_optimizer.lib
                          ${ONNXRUNTIME_ROOT}/lib/onnxruntime_providers_shared.lib
                          ${ONNXRUNTIME_ROOT}/lib/onnxruntime_providers.lib
                          ${ONNXRUNTIME_ROOT}/lib/onnxruntime_session.lib
                          ${ONNXRUNTIME_ROOT}/lib/onnxruntime_util.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_bad_optional_access.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_bad_variant_access.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_base.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_city.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_civil_time.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_cord.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_cord_internal.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_cordz_functions.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_cordz_handle.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_cordz_info.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_debugging_internal.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_demangle_internal.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_exponential_biased.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_graphcycles_internal.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_hash.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_hashtablez_sampler.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_int128.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_log_severity.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_low_level_hash.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_malloc_internal.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_raw_hash_set.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_raw_logging_internal.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_spinlock_wait.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_stacktrace.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_strings.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_strings_internal.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_symbolize.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_synchronization.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_throw_delegate.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_time.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/abseil/absl_time_zone.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/flatbuffers/flatbuffers.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/onnx/onnx.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/onnx/onnx_proto.lib
                          debug ${ONNXRUNTIME_ROOT}/lib/deps/protobuf/libprotobufd.lib
                          optimized ${ONNXRUNTIME_ROOT}/lib/deps/protobuf/libprotobuf.lib
                          debug ${ONNXRUNTIME_ROOT}/lib/deps/protobuf/libprotocd.lib
                          optimized ${ONNXRUNTIME_ROOT}/lib/deps/protobuf/libprotoc.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/pytorch_cpuinfo/cpuinfo.lib
                          ${ONNXRUNTIME_ROOT}/lib/deps/re2/re2.lib)
elseif (APPLE)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${OpenCV_INCLUDE_DIRS})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${OpenCV_LIBRARIES})
else()
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${OpenCV_INCLUDE_DIRS})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${OpenCV_LIBRARIES})
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)

if (MSVC)
    set(BINARY_DATA
        ${ONNXRUNTIME_ROOT}/lib/onnxruntime_providers_shared.dll
        ${CMAKE_CURRENT_SOURCE_DIR}/testdata/mnist.onnx
    )
elseif (APPLE)
else()
endif()

add_custom_command(TARGET ${PROJ_NAME} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different ${BINARY_DATA} ${CMAKE_INSTALL_PREFIX}/bin)