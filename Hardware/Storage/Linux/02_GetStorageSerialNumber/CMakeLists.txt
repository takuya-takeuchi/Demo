cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(UDEV REQUIRED libudev)

add_executable(${PROJ_NAME} main.cpp)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_include_directories(${PROJ_NAME} PRIVATE
                               ${UDEV_INCLUDE_DIRS})

    target_link_libraries(${PROJ_NAME} PRIVATE
                          ${UDEV_LIBRARIES})
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)