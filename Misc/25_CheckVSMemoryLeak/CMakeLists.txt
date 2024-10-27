cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Demo)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# This is not necessary
if(MSVC)
    add_compile_options(
        $<$<CONFIG:>:/MT>
        $<$<CONFIG:Debug>:/MTd>
        $<$<CONFIG:RelWithDebInfo>:/MT>
        $<$<CONFIG:MinSizeRel>:/MT>
        $<$<CONFIG:Release>:/MT>
    )
endif()

add_executable(${PROJ_NAME} main.cpp)

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)