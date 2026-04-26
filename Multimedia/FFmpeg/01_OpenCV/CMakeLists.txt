cmake_minimum_required(VERSION 3.20.0)

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

find_package(OpenCV)
if (NOT MSVC)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavformat libavutil libswscale)
    pkg_check_modules(OPENH264 REQUIRED IMPORTED_TARGET openh264)
endif()

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    file(GLOB_RECURSE FFMPEG_LIBRARIES "${FFMPEG_LIB_DIRS}/*.lib")
    file(GLOB_RECURSE OPENH264_LIBRARIES "${OPENH264_LIB_DIRS}/*.lib")

    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${OpenCV_INCLUDE_DIRS}
                               ${FFMPEG_INCLUDE_DIRS})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${OpenCV_LIBRARIES}
                          ${FFMPEG_LIBRARIES})
elseif (APPLE)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${OpenCV_INCLUDE_DIRS})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${OpenCV_LIBRARIES}
                          PkgConfig::FFMPEG
                          PkgConfig::OPENH264)
else()
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${OpenCV_INCLUDE_DIRS})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${OpenCV_LIBRARIES}
                          PkgConfig::FFMPEG
                          PkgConfig::OPENH264)
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)

set(TEST_DATA
    ${CMAKE_CURRENT_SOURCE_DIR}/bun33s.mp4
)
add_custom_command(TARGET ${PROJ_NAME} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_INSTALL_PREFIX}/bin
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different ${TEST_DATA} ${CMAKE_INSTALL_PREFIX}/bin)