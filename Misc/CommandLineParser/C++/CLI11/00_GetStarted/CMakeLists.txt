cmake_minimum_required(VERSION 3.20.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(CLI11 REQUIRED)

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          CLI11::CLI11)
elseif (APPLE)
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          CLI11::CLI11)
else()
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          CLI11::CLI11)
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)