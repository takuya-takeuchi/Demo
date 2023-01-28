cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Test)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if (MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
endif()

find_package(cpprestsdk REQUIRED CONFIG)

if (${cpprestsdk_FOUND})
    message("         cpprestsdk_DIR: ${cpprestsdk_DIR}")
    message("     cpprestsdk_VERSION: ${cpprestsdk_VERSION}")
endif()

add_executable(${PROJ_NAME} main.cpp)

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           cpprestsdk::cpprest
                           ${Boost_INCLUDE_DIRS})

target_link_libraries(${PROJ_NAME}
                      PRIVATE
                      cpprestsdk::cpprest
                      ${Boost_LIBRARIES})