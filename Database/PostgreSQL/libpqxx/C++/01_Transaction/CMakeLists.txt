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
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ldl")
endif()

find_package(libpqxx REQUIRED)

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          libpqxx::pqxx)
elseif (APPLE)
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          libpqxx::pqxx)
else()
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          libpqxx::pqxx)
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)

if (MSVC)
    file(TO_CMAKE_PATH "${GSTREAMER_ROOT}" GST_ROOT_CMAKE)
    file(GLOB BINARY_DATA "${GST_ROOT_CMAKE}/bin/*.dll")
    add_custom_command(TARGET ${PROJ_NAME} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${BINARY_DATA} ${CMAKE_INSTALL_PREFIX}/bin)
elseif (APPLE)
    file(TO_CMAKE_PATH "${GSTREAMER_ROOT}" GST_ROOT_CMAKE)
    file(GLOB PostgreSQL_LIBRARIES "${PostgreSQL_ROOT}/lib/*.dylib")
    add_custom_command(TARGET ${PROJ_NAME} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PostgreSQL_LIBRARIES} ${CMAKE_INSTALL_PREFIX}/bin)
else()
endif()