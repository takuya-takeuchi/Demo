cmake_minimum_required(VERSION 3.15.0)

set(PROJ_NAME Demo)

project(${PROJ_NAME} VERSION 1.0.0)

# necessary to use it!!
include(FetchContent)

# External dependencies
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# download and configure json
include(json)

add_executable(${PROJ_NAME} main.cpp)

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           ${JSON_INCLUDE_DIR})

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)
install(FILES example.json DESTINATION bin)