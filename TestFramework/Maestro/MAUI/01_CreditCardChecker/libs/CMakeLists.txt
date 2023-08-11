cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Luhn)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_OBJCXX_STANDARD 17)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# add sources
FILE(GLOB_RECURSE HEADERS ${PROJECT_SOURCE_DIR}/include/*.h)
FILE(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_library(${PROJ_NAME} SHARED ${HEADERS}
                                ${SOURCES})

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           ${LUHN_SDK_DIR}/Headers)

set_target_properties(${PROJ_NAME} PROPERTIES PUBLIC_HEADER ${HEADERS})
INSTALL(TARGETS ${PROJ_NAME} LIBRARY DESTINATION lib)
INSTALL(TARGETS ${PROJ_NAME} PUBLIC_HEADER DESTINATION include)