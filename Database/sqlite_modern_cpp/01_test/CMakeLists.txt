cmake_minimum_required(VERSION 3.5.0)

set(PROJ_NAME Test)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

add_executable(${PROJ_NAME} main.cpp
                            ${SQLITE_ROOT_DIR}/sqlite3.c)

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           ${SQLITE_MODERN_CPP_ROOT_DIR}
                           ${SQLITE_ROOT_DIR})

if (MSVC)
    target_link_libraries(${PROJ_NAME})
elseif (APPLE)
    target_link_libraries(${PROJ_NAME})
else()
    target_link_libraries(${PROJ_NAME})
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)