cmake_minimum_required(VERSION 3.11.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)
project(${PROJ_NAME} LANGUAGES CXX)

add_definitions(-DWINAPI_FAMILY=WINAPI_FAMILY_APP)
add_definitions(-D_UNICODE)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

add_executable(${PROJ_NAME} main.cpp)

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)
add_custom_command(
    TARGET ${PROJ_NAME} POST_BUILD
    COMMAND PowerShell -Command "if (-Not (Test-Path -Path '${CMAKE_INSTALL_PREFIX}/AppPackages/${PROJ_NAME}_1.0.0.0_x64_Test')) { New-Item -Path '${CMAKE_INSTALL_PREFIX}/AppPackages/${PROJ_NAME}_1.0.0.0_x64_Test' -ItemType Directory }"
    COMMAND PowerShell -Command "Copy-Item -Path '${CMAKE_CURRENT_BINARY_DIR}/AppPackages/${PROJ_NAME}/${PROJ_NAME}_1.0.0.0_x64_Test/*' -Destination '${CMAKE_INSTALL_PREFIX}/AppPackages/${PROJ_NAME}_1.0.0.0_x64_Test' -Recurse -Force"
)