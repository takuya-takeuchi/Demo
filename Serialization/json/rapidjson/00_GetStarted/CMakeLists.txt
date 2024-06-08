cmake_minimum_required(VERSION 3.26.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Find dependencies
find_package(RapidJSON REQUIRED)
if (RAPIDJSON_FOUND)
    message("Failed to find rapidjson")
else()
    message("RapidJSON_INCLUDE_DIRS: ${RapidJSON_INCLUDE_DIRS}")

    # https://github.com/Tencent/rapidjson/issues/2192
    # https://github.com/Tencent/rapidjson/pull/2193
    # RapidJSON_INCLUDE_DIRS has 'rapidjson'
    set(RapidJSON_INCLUDE_DIRS "${RapidJSON_INCLUDE_DIRS}/..")
endif()

add_executable(${PROJ_NAME} main.cpp)

# Define include headers and libraries
if (MSVC)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${RapidJSON_INCLUDE_DIRS})
elseif (APPLE)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${RapidJSON_INCLUDE_DIRS})
else()
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${RapidJSON_INCLUDE_DIRS})
endif()

# Install sections
install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)

if (MSVC)
    set(BINARY_DATA
        ${CMAKE_CURRENT_SOURCE_DIR}/testdata/test.json
    )
elseif (APPLE)
    set(BINARY_DATA
        ${CMAKE_CURRENT_SOURCE_DIR}/testdata/test.json
    )
else()
    set(BINARY_DATA
        ${CMAKE_CURRENT_SOURCE_DIR}/testdata/test.json
    )
endif()

add_custom_command(TARGET ${PROJ_NAME} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_INSTALL_PREFIX}/bin/testdata
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different ${BINARY_DATA} ${CMAKE_INSTALL_PREFIX}/bin/testdata)