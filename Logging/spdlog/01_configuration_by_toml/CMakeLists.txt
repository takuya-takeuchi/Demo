cmake_minimum_required(VERSION 3.12.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if (MSVC)
elseif (APPLE)
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

find_package(spdlog REQUIRED)
find_package(spdlog_setup REQUIRED)

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    # Do not specify spdlog::spdlog
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          spdlog_setup::spdlog_setup)
elseif (APPLE)
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          spdlog_setup::spdlog_setup)
else()
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          spdlog_setup::spdlog_setup)
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)
add_custom_command(TARGET ${PROJ_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/logging.toml" ${CMAKE_INSTALL_PREFIX}/bin)