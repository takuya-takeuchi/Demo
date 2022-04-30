cmake_minimum_required(VERSION 3.0.0)
project(CppLib VERSION 0.0.0)

include(CTest)
enable_testing()

add_library(CppLib SHARED CppLib.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
