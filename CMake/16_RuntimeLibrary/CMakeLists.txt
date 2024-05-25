cmake_minimum_required(VERSION 3.15.0)

set(PROJ_NAME Demo)

project(${PROJ_NAME} VERSION 1.0.0)

add_subdirectory(add_compile_options)
add_subdirectory(CMAKE_CXX_FLAGS)
add_subdirectory(MSVC_RUNTIME_LIBRARY)