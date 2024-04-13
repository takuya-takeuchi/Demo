cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Test)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if (MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
endif()

if (APPLE)
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

find_package(OpenAPIClient REQUIRED CONFIG)

if (${OpenAPIClient_FOUND})
    message("        OpenAPIClient_DIR: ${OpenAPIClient_DIR}")
endif()

# print all variables
#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

add_executable(${PROJ_NAME} main.cpp)

if (MSVC OR MSYS OR MINGW)
    # change runtime library to /MD for linked library
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MT")
    
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               cpprestsdk::cpprest
                               ${PACKAGE_PREFIX_DIR}/include)

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          cpprestsdk::cpprest
                          ${PACKAGE_PREFIX_DIR}/lib/OpenAPIClient.lib)
elseif (APPLE)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               cpprestsdk::cpprest
                               ${OPENSSL_INCLUDE_DIR}
                               ${PACKAGE_PREFIX_DIR}/include)

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          cpprestsdk::cpprest
                          ${OPENSSL_LIBRARIES}
                          ${PACKAGE_PREFIX_DIR}/lib/libOpenAPIClient.a)
else()
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               cpprestsdk::cpprest
                               ${PACKAGE_PREFIX_DIR}/include)

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          cpprestsdk::cpprest
                          ${PACKAGE_PREFIX_DIR}/lib/libOpenAPIClient.a)
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)