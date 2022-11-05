cmake_minimum_required(VERSION 3.0.0)
cmake_policy(SET CMP0053 NEW)
cmake_policy(SET CMP0054 NEW)

set(PROJ_NAME AsioSample)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# It means that CMakeLists.txt version rather than library version
project(${PROJ_NAME} VERSION 1.0.0)

# OS info
message("-------------------------------------------------------")
message("-- CMAKE_SYSTEM_INFO_FILE: ${CMAKE_SYSTEM_INFO_FILE}")
message("-- CMAKE_SYSTEM_NAME:      ${CMAKE_SYSTEM_NAME}")
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
message("-- CMAKE_SYSTEM:           ${CMAKE_SYSTEM}")
message("-- CMAKE_C_COMPILER:       ${CMAKE_C_COMPILER}")
message("-- CMAKE_CXX_COMPILER:     ${CMAKE_CXX_COMPILER}")
string (REGEX MATCH "\\.el[1-9]" os_version_suffix ${CMAKE_SYSTEM})
message("-- os_version_suffix:      ${os_version_suffix}")
message("-- Project: ${CMAKE_CURRENT_SOURCE_DIR}")
message("-- Binaries: ${CMAKE_CURRENT_BINARY_DIR}")
message("-------------------------------------------------------")

# Version info
set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)
set(VERSION_DATE 20221105)

# Only GCC requires -fPIC
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	set(CMAKE_C_FLAGS "-fPIC")
	set(CMAKE_CXX_FLAGS "-fPIC")
endif()

# set architecture
if (NOT ${CMAKE_GENERATOR} MATCHES "Visual Studio")
    if ("${ARCH_TYPE}" STREQUAL "32")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m32")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")
    elseif ("${ARCH_TYPE}" STREQUAL "64")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m64")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64")
    endif()
endif()

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
    message("-- CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
endif()

# Set /bigobj for MSVC
if (${CMAKE_GENERATOR} MATCHES "Visual Studio")
	# C4819: The file contains a character that cannot be represented in the current code page
	# C4503: identifier' : decorated name length exceeded, name was truncated
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4819 /wd4503 /bigobj")

	# C4146: unary minus operator applied to unsigned type, result still unsigned
	if ("${CMAKE_SYSTEM_NAME}" STREQUAL "WindowsStore")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4146 /wd4996 /ZW /EHsc")
	endif()
endif()

# set parallel build
if (${CMAKE_GENERATOR} MATCHES "Visual Studio")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

# create config file
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/version.rc.in
    ${CMAKE_CURRENT_BINARY_DIR}/version.rc
    @ONLY)

FILE(GLOB_RECURSE HEADERS ${PROJECT_SOURCE_DIR}/src/*.h
                          ${PROJECT_SOURCE_DIR}/asiosdk/common/asio.h
                          ${PROJECT_SOURCE_DIR}/asiosdk/common/asiosys.h
                          ${PROJECT_SOURCE_DIR}/asiosdk/common/combase.h
                          ${PROJECT_SOURCE_DIR}/asiosdk/common/iasiodrv.h
                          ${PROJECT_SOURCE_DIR}/asiosdk/common/wxdebug.h
                          ${PROJECT_SOURCE_DIR}/asiosdk/host/asiodrivers.h
                          ${PROJECT_SOURCE_DIR}/asiosdk/host/ginclude.h
                          ${PROJECT_SOURCE_DIR}/asiosdk/host/pc/asiolist.h)
FILE(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp
                          ${PROJECT_SOURCE_DIR}/asiosdk/common/asio.cpp
                          ${PROJECT_SOURCE_DIR}/asiosdk/common/asiosys.cpp
                          ${PROJECT_SOURCE_DIR}/asiosdk/common/combase.cpp
                          ${PROJECT_SOURCE_DIR}/asiosdk/common/iasiodrv.cpp
                          ${PROJECT_SOURCE_DIR}/asiosdk/common/wxdebug.cpp
                          ${PROJECT_SOURCE_DIR}/asiosdk/host/asiodrivers.cpp
                          ${PROJECT_SOURCE_DIR}/asiosdk/host/pc/asiolist.cpp)

if(APPLE)
    list(APPEND HEADERS ${PROJECT_SOURCE_DIR}/asiosdk/common/asiodrvr.h)
    list(APPEND SOURCES ${PROJECT_SOURCE_DIR}/asiosdk/common/asiodrvr.cpp)
elseif(UNIX AND NOT APPLE)
    list(APPEND HEADERS ${PROJECT_SOURCE_DIR}/asiosdk/common/asiodrvr.h)
    list(APPEND SOURCES ${PROJECT_SOURCE_DIR}/asiosdk/common/asiodrvr.cpp)
endif()

add_executable(${PROJ_NAME} ${HEADERS}
                            ${SOURCES}
                            ${CMAKE_CURRENT_BINARY_DIR}/version.rc)

include_directories(${PROJECT_SOURCE_DIR}/asiosdk/common
                    ${PROJECT_SOURCE_DIR}/asiosdk/host
                    ${PROJECT_SOURCE_DIR}/asiosdk/host/pc)

set(CompilerFlags
    CMAKE_CXX_FLAGS
    CMAKE_CXX_FLAGS_DEBUG
    CMAKE_CXX_FLAGS_RELEASE
    CMAKE_C_FLAGS
    CMAKE_C_FLAGS_DEBUG
    CMAKE_C_FLAGS_RELEASE
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

# install command
install(TARGETS ${PROJ_NAME}
        CONFIGURATIONS Debug
        RUNTIME DESTINATION Debug)
install(TARGETS ${PROJ_NAME}
        CONFIGURATIONS Release
        RUNTIME DESTINATION Release)