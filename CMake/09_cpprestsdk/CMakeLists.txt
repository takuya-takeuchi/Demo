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
    find_package(Boost REQUIRED COMPONENTS system date_time regex)
else()
    find_package(Boost REQUIRED COMPONENTS system filesystem date_time regex random thread chrono)
endif()

if (NOT ${Boost_FOUND})
    message(FATAL_ERROR "Boost is not found!!")
else()
    message("Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
    message("   Boost_LIBRARIES: ${Boost_LIBRARIES}")
endif()

find_package(cpprestsdk REQUIRED CONFIG)

if (NOT ${cpprestsdk_FOUND})
    message(FATAL_ERROR "cpprestsdk is not found!!")
else()
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