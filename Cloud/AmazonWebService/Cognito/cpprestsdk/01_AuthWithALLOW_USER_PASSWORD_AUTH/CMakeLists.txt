cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Test)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if (MSVC OR MSYS OR MINGW)
    set(CMAKE_CXX_FLAGS "/O2")
    set(CMAKE_CXX_FLAGS_DEBUG "/Od /Z7")
elseif(APPLE)
    set(CMAKE_CXX_FLAGS "-Os")
    set(CMAKE_CXX_FLAGS_DEBUG "-O0")
elseif(UNIX AND NOT APPLE)
    set(CMAKE_CXX_FLAGS "-O2")
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
endif()

if (MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
endif()

if (NOT MSVC)
    find_package(OpenSSL REQUIRED)

    if (${OPENSSL_FOUND})
        message("    OPENSSL_VERSION: ${OPENSSL_VERSION}")
        message("OPENSSL_INCLUDE_DIR: ${OPENSSL_INCLUDE_DIR}")
        message("  OPENSSL_LIBRARIES: ${OPENSSL_LIBRARIES}")
    endif()
endif()

find_package(cpprestsdk REQUIRED CONFIG)

if (${cpprestsdk_FOUND})
    message("    cpprestsdk_DIR: ${cpprestsdk_DIR}")
    message("cpprestsdk_VERSION: ${cpprestsdk_VERSION}")
endif()

add_executable(${PROJ_NAME} main.cpp)

if (NOT MSVC)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               cpprestsdk::cpprest
                               ${OPENSSL_INCLUDE_DIR})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          cpprestsdk::cpprest
                          ${OPENSSL_LIBRARIES})
else()
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               cpprestsdk::cpprest)

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          cpprestsdk::cpprest)
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)