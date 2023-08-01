cmake_minimum_required(VERSION 3.9.0)
set(PROJ_NAME Demo)

set(CMAKE_CXX_STANDARD 17)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

option(NUMCPP_NO_USE_BOOST "Don't use the boost libraries" OFF)

find_package(NumCpp REQUIRED)

if (NumCpp_FOUND)
    message("    NumCpp_VERSION: ${NumCpp_VERSION}")
    message("NumCpp_INCLUDE_DIR: ${NumCpp_INCLUDE_DIR}")
endif()

# target source and header files
FILE(GLOB_RECURSE HEADERS ${PROJECT_SOURCE_DIR}/*.h)
FILE(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/*.cpp)

add_executable(${PROJ_NAME} main.cpp)

if(NUMCPP_NO_USE_BOOST)
    message("NUMCPP_NO_USE_BOOST is ON")
    target_compile_definitions(${PROJ_NAME} PUBLIC -DNUMCPP_NO_USE_BOOST)
endif()

# include eigen headers
target_include_directories(${PROJ_NAME} PUBLIC ${NumCpp_INCLUDE_DIR})

set(CompilerFlags
    CMAKE_CXX_FLAGS
    CMAKE_CXX_FLAGS_DEBUG
    CMAKE_CXX_FLAGS_RELEASE
    CMAKE_C_FLAGS
    CMAKE_C_FLAGS_DEBUG
    CMAKE_C_FLAGS_RELEASE
)
foreach(CompilerFlag ${CompilerFlags})
string(REPLACE "/MDd" "/MD" ${CompilerFlag} "${${CompilerFlag}}")
endforeach()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

# install command
install (TARGETS ${PROJ_NAME} RUNTIME)
#install (FILES conv.dat DESTINATION bin)