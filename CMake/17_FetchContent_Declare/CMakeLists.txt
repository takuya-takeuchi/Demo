cmake_minimum_required(VERSION 3.15.0)

set(PROJ_NAME Demo)

project(${PROJ_NAME} VERSION 1.0.0)

# necessary to use it!!
include(FetchContent)

# External dependencies
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# download and configure zlib
include(zlib)

add_executable(${PROJ_NAME} main.cpp)

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           zlib)

target_link_libraries(${PROJ_NAME}
                      PRIVATE
                      zlib)

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)