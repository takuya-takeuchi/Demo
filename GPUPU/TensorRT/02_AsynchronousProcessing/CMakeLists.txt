cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(CUDA REQUIRED)

if(CUDA_FOUND)
    message("         CUDA_VERSION: ${CUDA_VERSION}")
    message("    CUDA_INCLUDE_DIRS: ${CUDA_INCLUDE_DIRS}")
    message("       CUDA_LIBRARIES: ${CUDA_LIBRARIES}")

    message("TENSORRT_INCLUDE_DIRS: ${TENSORRT_INCLUDE_DIRS}")
    message("   TENSORRT_LIBRARIES: ${TENSORRT_LIBRARIES}")
endif()

add_executable(${PROJ_NAME} main.cpp)

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           ${CUDA_INCLUDE_DIRS}
                           ${TENSORRT_INCLUDE_DIRS})

target_link_libraries(${PROJ_NAME}
                      PRIVATE
                      ${CUDA_LIBRARIES}
                      ${TENSORRT_LIBRARIES})

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)

set(TEST_DATA
    ${CMAKE_CURRENT_SOURCE_DIR}/ResNet101-DUC-12.trt
)
add_custom_command(TARGET ${PROJ_NAME} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different ${TEST_DATA} ${CMAKE_INSTALL_PREFIX}/bin)