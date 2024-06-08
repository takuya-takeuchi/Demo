cmake_minimum_required(VERSION 3.9.0)
set(PROJ_NAME Demo)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

add_executable(${PROJ_NAME} main.cpp)

# Install sections
install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)