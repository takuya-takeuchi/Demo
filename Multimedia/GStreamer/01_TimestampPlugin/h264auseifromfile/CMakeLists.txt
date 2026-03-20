cmake_minimum_required(VERSION 3.26.0)

set(PROJ_NAME gsth264auseifromfile)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Version info
set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)
set(VERSION_DATE 20260321)

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
    gstreamer-1.0
    gstreamer-base-1.0
    gstreamer-video-1.0
)

if (MSVC)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/version.rc.in
        ${CMAKE_CURRENT_BINARY_DIR}/version.rc
        @ONLY)

    add_library(${PROJ_NAME} SHARED gsth264auseifromfile.c ${CMAKE_CURRENT_BINARY_DIR}/version.rc)
else()
    add_library(${PROJ_NAME} SHARED gsth264auseifromfile.c)
endif()

if (MSVC)
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          PkgConfig::GST)
elseif (APPLE)
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          PkgConfig::GST)

    set_target_properties(gsth264auseifromfile PROPERTIES
        PREFIX "lib"
        OUTPUT_NAME "gsth264auseifromfile"
    )
else()
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          PkgConfig::GST)

    set_target_properties(gsth264auseifromfile PROPERTIES
        PREFIX "lib"
        OUTPUT_NAME "gsth264auseifromfile"
    )

    set_target_properties(${PROJ_NAME} PROPERTIES
        BUILD_RPATH "${GST_LIBRARY_DIRS}"
        INSTALL_RPATH "${GST_LIBRARY_DIRS}"
    )
endif()

target_compile_definitions(gsth264auseifromfile
    PRIVATE
        PACKAGE_NAME="h264auseifromfile"
        PACKAGE="h264auseifromfile"
        GST_PACKAGE_NAME="h264auseifromfile"
        GST_PACKAGE_ORIGIN="https://taktak.jp"
)

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)