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

add_executable(${PROJ_NAME} main.cpp)
target_include_directories(${PROJ_NAME}
                           PRIVATE
                           ${OpenCV_INCLUDE_DIRS})
# OpenCV_LIBRARIES is not supported for staticlib so defines each library manually
target_link_libraries(${PROJ_NAME}
                      PRIVATE
                      ade.lib
                      ippicvmt.lib
                      ippiw.lib
                      ittnotify.lib
                      libprotobuf.lib
                      libwebp.lib
                      opencv_world470.lib
                      quirc.lib
                      zlib.lib)

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)