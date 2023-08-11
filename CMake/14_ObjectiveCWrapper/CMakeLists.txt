cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME program)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_OBJCXX_STANDARD 17)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# for NSCharacterSet, SMutableString...
find_library(FOUNDATION Foundation)

# add sources
FILE(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/main.cpp)

add_executable(${PROJ_NAME} ${SOURCES})

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           ${LUHNC_SDK_DIR}/include)

target_link_libraries(${PROJ_NAME}
                      PRIVATE
                      ${FOUNDATION}
                      ${LUHNC_SDK_DIR}/lib/libLuhnc.dylib)

# let loader know where is likned libraries
set_target_properties(${PROJ_NAME} PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "@executable_path;@executable_path/lib"
)

INSTALL(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)
INSTALL(FILES ${LUHNC_SDK_DIR}/lib/libLuhnc.dylib DESTINATION bin)