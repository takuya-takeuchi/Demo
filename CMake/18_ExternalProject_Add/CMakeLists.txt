cmake_minimum_required(VERSION 3.15.0)

set(PROJ_NAME Demo)

project(${PROJ_NAME} VERSION 1.0.0)

# necessary to use it!!
include(ExternalProject)

# External dependencies
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# download and configure libjpeg-turbo
include(libjpeg-turbo)

add_executable(${PROJ_NAME} main.cpp)

# download and configure libjpeg-turbo
add_dependencies(${PROJ_NAME} libjpeg-turbo)

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           ${LIBJPEG_TURBO_INCLUDE_DIR})

target_link_libraries(${PROJ_NAME}
                      PRIVATE
                      ${LIBJPEG_TURBO_LIBRARIES})

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)