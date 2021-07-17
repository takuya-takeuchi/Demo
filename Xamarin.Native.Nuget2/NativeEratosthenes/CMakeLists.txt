cmake_minimum_required(VERSION 3.15.0)

set(PROJ_NAME NativeEratosthenes)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

option(BUILD_FRAMEWORK "Build as framrwork" OFF)

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

FILE(GLOB_RECURSE HEADERS ${PROJECT_SOURCE_DIR}/src/*.h)
FILE(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_library(${PROJ_NAME} ${HEADERS}
                         ${SOURCES})

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

install(TARGETS ${PROJ_NAME} DESTINATION lib)

if (BUILD_FRAMEWORK)
    set_target_properties(${PROJ_NAME} PROPERTIES
        FRAMEWORK TRUE
        FRAMEWORK_VERSION C
        #MACOSX_FRAMEWORK_IDENTIFIER com.cmake.dynamicFramework
        #MACOSX_FRAMEWORK_INFO_PLIST Info.plist
        # "current version" in semantic format in Mach-O binary file
        VERSION 1.0.0
        # "compatibility version" in semantic format in Mach-O binary file
        SOVERSION 1.0.0
        XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "iPhone Developer"
    )
endif()