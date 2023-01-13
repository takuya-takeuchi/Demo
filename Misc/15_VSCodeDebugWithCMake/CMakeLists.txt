cmake_minimum_required(VERSION 3.20.0)
set(PROJ_NAME Test)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(OpenCV REQUIRED)

add_executable(${PROJ_NAME} main.cpp)

target_include_directories(${PROJ_NAME}
                           PUBLIC
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