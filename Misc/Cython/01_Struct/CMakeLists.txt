cmake_minimum_required(VERSION 3.12.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(Python COMPONENTS Interpreter Development)

# add sources
FILE(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/main.cpp)

add_executable(${PROJ_NAME} ${SOURCES})

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           ${Python_INCLUDE_DIRS})

target_link_libraries(${PROJ_NAME}
                      PRIVATE
                      ${Python_LIBRARIES})

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)