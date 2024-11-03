cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(cpuinfo REQUIRED)

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    if(CMAKE_SYSTEM_NAME STREQUAL "WindowsStore")
        target_compile_options(${PROJ_NAME} PRIVATE /ZW)

        target_include_directories(${PROJ_NAME}
                                   PRIVATE
                                   cpuinfo::cpuinfo)

        target_link_libraries(${PROJ_NAME}
                              PRIVATE
                              windowsapp
                              cpuinfo::cpuinfo)
    else()
        target_include_directories(${PROJ_NAME}
                                   PRIVATE
                                   cpuinfo::cpuinfo)

        target_link_libraries(${PROJ_NAME}
                              PRIVATE
                              cpuinfo::cpuinfo)
    endif()
elseif (ANDROID)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               cpuinfo::cpuinfo)

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          cpuinfo::cpuinfo)
elseif (IOS)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               cpuinfo::cpuinfo)                               

    if ("${TARGET_ARCHITECTURES}" STREQUAL "x86_64")
        target_link_libraries(${PROJ_NAME}
                              PRIVATE
                              cpuinfo::cpuinfo)
    elseif ("${TARGET_ARCHITECTURES}" STREQUAL "arm64")
        target_link_libraries(${PROJ_NAME}
                              PRIVATE
                              cpuinfo::cpuinfo)
    endif()
elseif (APPLE)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               cpuinfo::cpuinfo)                               

    if ("${TARGET_ARCHITECTURES}" STREQUAL "x86_64")
        target_link_libraries(${PROJ_NAME}
                              PRIVATE
                              cpuinfo::cpuinfo)
    elseif ("${TARGET_ARCHITECTURES}" STREQUAL "arm64")
        target_link_libraries(${PROJ_NAME}
                              PRIVATE
                              cpuinfo::cpuinfo)
    endif()
else()
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               cpuinfo::cpuinfo)

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          cpuinfo::cpuinfo)
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)