cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(Open3D REQUIRED)

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               Open3D::Open3D)

    target_link_libraries(${PROJ_NAME} PRIVATE Open3D::Open3D)
elseif (APPLE)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               Open3D::Open3D)

    target_link_libraries(${PROJ_NAME} PRIVATE Open3D::Open3D)
else()
    find_package(OpenGL REQUIRED)

    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               OpenGL::GL
                               Open3D::Open3D)

    target_link_libraries(${PROJ_NAME} PRIVATE OpenGL::GL Open3D::Open3D)
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)