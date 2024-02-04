cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Test)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(Boost REQUIRED COMPONENTS system filesystem program_options)

if (NOT ${Boost_FOUND})
    message(FATAL_ERROR "Boost is not found!!")
else()
    message("Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
    message("   Boost_LIBRARIES: ${Boost_LIBRARIES}")
endif()

add_executable(${PROJ_NAME} main.cpp)

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           ${Boost_INCLUDE_DIRS})

target_link_libraries(${PROJ_NAME}
                      PRIVATE
                      ${Boost_LIBRARIES})