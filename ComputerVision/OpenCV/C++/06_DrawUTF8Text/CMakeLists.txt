cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if (MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
endif()

find_package(OpenCV REQUIRED)
find_package(freetype REQUIRED)
find_package(harfbuzz REQUIRED)

if (${OpenCV_FOUND})
    message("         OpenCV_VERSION: ${OpenCV_VERSION}")
    message("    OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
    message("       OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
endif()

if (${FREETYPE_FOUND})
    message("FREETYPE_VERSION_STRING: ${FREETYPE_VERSION_STRING}")
    message("  FREETYPE_INCLUDE_DIRS: ${FREETYPE_INCLUDE_DIRS}")
    message("     FREETYPE_LIBRARIES: ${FREETYPE_LIBRARIES}")
endif()

if (${harfbuzz_FOUND})
    # HarfBuzz cmake configuration does not provide any variables
    message("           harfbuzz_DIR: ${harfbuzz_DIR}")
endif()

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    target_compile_definitions(${PROJ_NAME} PRIVATE _SILENCE_CXX17_CODECVT_HEADER_DEPRECATION_WARNING)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${OpenCV_INCLUDE_DIRS})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${OpenCV_LIBRARIES}
                          Freetype::Freetype
                          harfbuzz::harfbuzz)
elseif (APPLE)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${OpenCV_INCLUDE_DIRS})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${OpenCV_LIBRARIES}
                          Freetype::Freetype)
else()
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${OpenCV_INCLUDE_DIRS})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${OpenCV_LIBRARIES}
                          Freetype::Freetype)
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)

set(TEST_DATA
    ${CMAKE_CURRENT_SOURCE_DIR}/fonts/NotoSansJP-VariableFont_wght.ttf
)
add_custom_command(TARGET ${PROJ_NAME} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_INSTALL_PREFIX}/bin/fonts
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different ${TEST_DATA} ${CMAKE_INSTALL_PREFIX}/bin/fonts)