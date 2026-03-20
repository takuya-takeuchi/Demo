cmake_minimum_required(VERSION 3.26.0)

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

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET
    NO_CMAKE_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    gstreamer-1.0
)

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          PkgConfig::GST)
elseif (APPLE)
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          PkgConfig::GST)
else()
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          PkgConfig::GST)

    set_target_properties(${PROJ_NAME} PROPERTIES
        BUILD_RPATH "${GST_LIBRARY_DIRS}"
        INSTALL_RPATH "${GST_LIBRARY_DIRS}"
    )
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)

if (MSVC)
    set(BINARY_DATA
        ${GSTREAMER_ROOT}/bin/ffi-7.dll
        ${GSTREAMER_ROOT}/bin/glib-2.0-0.dll
        ${GSTREAMER_ROOT}/bin/gmodule-2.0-0.dll
        ${GSTREAMER_ROOT}/bin/gobject-2.0-0.dll
        ${GSTREAMER_ROOT}/bin/gstreamer-1.0-0.dll
        ${GSTREAMER_ROOT}/bin/intl-8.dll
    )
    add_custom_command(TARGET ${PROJ_NAME} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${BINARY_DATA} ${CMAKE_INSTALL_PREFIX}/bin)
elseif (APPLE)
else()
endif()