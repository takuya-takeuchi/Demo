cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if (MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
endif()

find_package(Torch REQUIRED)

message("   TORCH_LIBRARIES: ${TORCH_LIBRARIES}")
message("TORCH_INCLUDE_DIRS: ${TORCH_INCLUDE_DIRS}")
message("Protobuf_LIBRARIES: ${Protobuf_LIBRARIES}")

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${TORCH_INCLUDE_DIRS})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${TORCH_LIBRARIES}
                          ${Protobuf_LIBRARIES})
elseif (ANDROID)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${TORCH_INCLUDE_DIRS})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${TORCH_LIBRARIES}
                          ${Protobuf_LIBRARIES}
                          ${sleef_LIBRARIES}
                          ${TORCH_ADDITIONAL_LIBS_DIRS}/libCaffe2_perfkernels_avx.a
                          ${TORCH_ADDITIONAL_LIBS_DIRS}/libCaffe2_perfkernels_avx2.a
                          ${TORCH_ADDITIONAL_LIBS_DIRS}/libCaffe2_perfkernels_avx512.a)
elseif (IOS)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${TORCH_INCLUDE_DIRS})                               

    if ("${TARGET_ARCHITECTURES}" STREQUAL "x86_64")
        target_link_libraries(${PROJ_NAME}
                              PRIVATE
                              ${TORCH_LIBRARIES}
                              ${Protobuf_LIBRARIES}
                              ${sleef_LIBRARIES}
                              ${TORCH_ADDITIONAL_LIBS_DIRS}/libCaffe2_perfkernels_avx.a
                              ${TORCH_ADDITIONAL_LIBS_DIRS}/libCaffe2_perfkernels_avx2.a
                              ${TORCH_ADDITIONAL_LIBS_DIRS}/libCaffe2_perfkernels_avx512.a
                              "-framework Accelerate"
                              "-framework Foundation"
                              "-framework MetalPerformanceShaders"
                              "-framework MetalPerformanceShadersGraph"
                              "-framework Metal")
    elseif ("${TARGET_ARCHITECTURES}" STREQUAL "arm64")
        target_link_libraries(${PROJ_NAME}
                              PRIVATE
                              ${TORCH_LIBRARIES}
                              ${Protobuf_LIBRARIES}
                              ${sleef_LIBRARIES}
                              "-framework Accelerate"
                              "-framework Foundation"
                              "-framework MetalPerformanceShaders"
                              "-framework MetalPerformanceShadersGraph"
                              "-framework Metal")
    endif()
elseif (APPLE)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${TORCH_INCLUDE_DIRS})                               

    if ("${TARGET_ARCHITECTURES}" STREQUAL "x86_64")
        target_link_libraries(${PROJ_NAME}
                              PRIVATE
                              ${TORCH_LIBRARIES}
                              ${Protobuf_LIBRARIES}
                              ${sleef_LIBRARIES}
                              ${TORCH_ADDITIONAL_LIBS_DIRS}/libCaffe2_perfkernels_avx.a
                              ${TORCH_ADDITIONAL_LIBS_DIRS}/libCaffe2_perfkernels_avx2.a
                              ${TORCH_ADDITIONAL_LIBS_DIRS}/libCaffe2_perfkernels_avx512.a
                              "-framework Accelerate"
                              "-framework Foundation"
                              "-framework MetalPerformanceShaders"
                              "-framework MetalPerformanceShadersGraph"
                              "-framework Metal")
    elseif ("${TARGET_ARCHITECTURES}" STREQUAL "arm64")
        target_link_libraries(${PROJ_NAME}
                              PRIVATE
                              ${TORCH_LIBRARIES}
                              ${Protobuf_LIBRARIES}
                              ${sleef_LIBRARIES}
                              "-framework Accelerate"
                              "-framework Foundation"
                              "-framework MetalPerformanceShaders"
                              "-framework MetalPerformanceShadersGraph"
                              "-framework Metal")
    endif()
else()
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${TORCH_INCLUDE_DIRS})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${TORCH_LIBRARIES}
                          ${Protobuf_LIBRARIES}
                          ${sleef_LIBRARIES}
                          ${TORCH_ADDITIONAL_LIBS_DIRS}/libCaffe2_perfkernels_avx.a
                          ${TORCH_ADDITIONAL_LIBS_DIRS}/libCaffe2_perfkernels_avx2.a
                          ${TORCH_ADDITIONAL_LIBS_DIRS}/libCaffe2_perfkernels_avx512.a)
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)