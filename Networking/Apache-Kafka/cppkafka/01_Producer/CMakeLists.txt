cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if (MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
endif()

find_package(RdKafka CONFIG REQUIRED)

if (${RdKafka_FOUND})
    message("     RdKafka_VERSION: ${RdKafka_VERSION}")
else()
    message(FATAL_ERROR "librdkafka is missing")
endif()

message("  CPPKAFKA_LIBRARIES: ${CPPKAFKA_LIBRARIES}")
message("CPPKAFKA_INCLUDE_DIR: ${CPPKAFKA_INCLUDE_DIR}")

add_executable(${PROJ_NAME} main.cpp)

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           RdKafka::rdkafka
                           RdKafka::rdkafka++
                           ${CPPKAFKA_INCLUDE_DIR})

target_link_libraries(${PROJ_NAME}
                      PRIVATE
                      RdKafka::rdkafka
                      RdKafka::rdkafka++
                      ${CPPKAFKA_LIBRARIES})

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)