cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Test)
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

find_package(AWSSDK REQUIRED COMPONENTS s3)

if (${AWSSDK_FOUND})
    message(" AWSSDK_PLATFORM_DEPS: ${AWSSDK_PLATFORM_DEPS}")
    message("AWSSDK_LINK_LIBRARIES: ${AWSSDK_LINK_LIBRARIES}")
    message("  AWSSDK_INCLUDE_DIRS: ${AWSSDK_INCLUDE_DIRS}")
endif()

add_executable(${PROJ_NAME} main.cpp)

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           AWSSDK::cpprest)

target_link_libraries(${PROJ_NAME}
                      PRIVATE
                      ${AWSSDK_LINK_LIBRARIES}
                      ${AWSSDK_PLATFORM_DEPS})

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)