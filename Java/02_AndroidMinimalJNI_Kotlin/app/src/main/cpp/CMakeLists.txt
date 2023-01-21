cmake_minimum_required(VERSION 3.18.1)

set(PROJ_NAME hello)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(${PROJ_NAME} VERSION 1.0.0 LANGUAGES CXX)

FILE(GLOB_RECURSE HEADERS ${PROJECT_SOURCE_DIR}/*.hpp)
FILE(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/*.cpp)

add_library(${PROJ_NAME} SHARED ${SOURCES}
                                ${HEADERS})

target_link_libraries(${PROJ_NAME}
                      android
                      log)