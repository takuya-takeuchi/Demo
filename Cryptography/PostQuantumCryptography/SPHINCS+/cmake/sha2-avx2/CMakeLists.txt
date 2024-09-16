cmake_minimum_required(VERSION 3.12.0)
cmake_policy(SET CMP0053 NEW)
cmake_policy(SET CMP0054 NEW)

set(PROJ_NAME sphincsplus)

project(${PROJ_NAME} VERSION 1.0.0)

# Version info
set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)
set(VERSION_DATE 0)

SET(CMAKE_C_FLAGS  "-Wall -Wextra -Wpedantic -Wmissing-prototypes -O3 -std=c99 -march=native -flto -fomit-frame-pointer")

find_package(OpenSSL REQUIRED)

add_compile_definitions(PARAMS=sphincs-sha2-128f)
add_compile_definitions(THASH=robust)

FILE(GLOB_RECURSE HEADERS ${PROJECT_SOURCE_DIR}/*.h)
FILE(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/*.c)

add_library(${PROJ_NAME} ${HEADERS}
                         ${SOURCES})

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           ${OPENSSL_INCLUDE_DIR})
target_link_libraries(${PROJ_NAME}
                      PRIVATE
                      ${OPENSSL_LIBRARIES}
                      crypto)

INSTALL(TARGETS ${PROJ_NAME})