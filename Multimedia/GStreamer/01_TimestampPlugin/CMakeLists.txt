cmake_minimum_required(VERSION 3.26.0)

set(PROJ_NAME plugins)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

add_subdirectory(h264auseifromfile)
add_subdirectory(h264autimestampdump)