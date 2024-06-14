cmake_minimum_required(VERSION 3.15.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if (MSVC)
    add_compile_options(
        $<$<CONFIG:>:/MT>
        $<$<CONFIG:Debug>:/MTd>
        $<$<CONFIG:RelWithDebInfo>:/MT>
        $<$<CONFIG:MinSizeRel>:/MT>
        $<$<CONFIG:Release>:/MT>
    )
endif()


if (NOT MSVC)
    find_package(Tesseract REQUIRED)
endif()

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${Tesseract_INCLUDE_DIR})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${Tesseract_LIBRARIES})
elseif (APPLE)
else()
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)