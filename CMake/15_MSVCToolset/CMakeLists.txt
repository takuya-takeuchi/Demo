cmake_minimum_required(VERSION 3.18.0)
cmake_policy(SET CMP0053 NEW)
cmake_policy(SET CMP0054 NEW)

set(PROJ_NAME Demo)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
    message("-- CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
endif()

add_executable(${PROJ_NAME} ${PROJECT_SOURCE_DIR}/main.cpp)

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)