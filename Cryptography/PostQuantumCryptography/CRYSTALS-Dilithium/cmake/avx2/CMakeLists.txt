cmake_minimum_required(VERSION 3.0.0)
cmake_policy(SET CMP0053 NEW)
cmake_policy(SET CMP0054 NEW)

set(PROJ_NAME dilithium)

project(${PROJ_NAME} VERSION 1.0.0)

SET(CMAKE_C_FLAGS  "-Wall -Wextra -Wpedantic -Wmissing-prototypes -Wredundant-decls -Wshadow -Wpointer-arith -mavx2 -mpopcnt -march=native -mtune=native -O3")

# Version info
set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)
set(VERSION_DATE 0)

FILE(GLOB_RECURSE HEADERS ${PROJECT_SOURCE_DIR}/*.h)
FILE(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/*.c)

add_library(${PROJ_NAME} ${HEADERS}
                         ${SOURCES})

INSTALL(TARGETS ${PROJ_NAME})