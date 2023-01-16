cmake_minimum_required(VERSION 3.20.0)
set(PROJ_NAME Test)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if (MSVC OR MSYS OR MINGW)
    set(CMAKE_CXX_FLAGS "/O2")
    set(CMAKE_CXX_FLAGS_DEBUG "/Od /Z7")
elseif(APPLE)
    set(CMAKE_CXX_FLAGS "-O2")
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
elseif(UNIX AND NOT APPLE)
    set(CMAKE_CXX_FLAGS "-Os")
    set(CMAKE_CXX_FLAGS_DEBUG "-O0")
endif()

# Cmake show following error message when building on windows
#
#     CMake Warning at build/win/opencv/win-install/x64/vc17/staticlib/OpenCVConfig.cmake:116 (message):
#       OpenCV: Include directory doesn't exist:
#       'C:/Demo/Misc/15_VSCodeDebugWithCMake/build/win/opencv/win-install/include'.
#       OpenCV installation may be broken.  Skip...
#
# I'm not sure why OpenCVConfig.cmake file can not specify proper directory.
# Therefore, specify OpenCV_INCLUDE_DIRS and OpenCV_LIBS variable directory via OpenCV_DIR variable
if (MSVC OR MSYS OR MINGW)
    set(OpenCV_INCLUDE_DIRS $ENV{OpenCV_DIR}/include)
    FILE(GLOB_RECURSE OpenCV_LIBS $ENV{OpenCV_DIR}/x64/*.lib)
else()
    find_package(OpenCV REQUIRED)
endif()

add_executable(${PROJ_NAME} main.cpp)

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           ${OpenCV_INCLUDE_DIRS})

if (MSVC OR MSYS OR MINGW)
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${OpenCV_LIBS})
elseif(APPLE)
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${OpenCV_LIBS})
elseif(UNIX AND NOT APPLE)
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${OpenCV_LIBS})
else()
    message(FATAL_ERROR "Failed to include directories")
endif()