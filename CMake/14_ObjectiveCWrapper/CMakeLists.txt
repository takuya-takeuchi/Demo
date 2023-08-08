cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Luhn)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# for NSCharacterSet, SMutableString...
find_library(FOUNDATION Foundation)

add_library(${PROJ_NAME} SHARED main.cpp)

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           ${FOUNDATION})

target_link_libraries(${PROJ_NAME}
                      PRIVATE
                      ${FOUNDATION})

INSTALL(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)