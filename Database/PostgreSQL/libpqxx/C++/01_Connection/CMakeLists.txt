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

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBPQ REQUIRED libpq)
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
    target_include_directories(${PROJ_NAME}
        PRIVATE
        ${LIBPQ_INCLUDE_DIRS})

    link_directories(${LIBPQ_STATIC_LIBRARY_DIRS})

    file(GLOB POSTGRES_OAUTH_LIBS "${LIBPQ_STATIC_LIBRARY_DIRS}/libpq-oauth.a")
    file(GLOB POSTGRES_SERVER_DEV_STATIC_LIBS "${PostgreSQL_SERVER_DEV_DIR}/*.a")

    if(LIBPQ_LIBRARIES STREQUAL "pq" OR LIBPQ_LIBRARIES STREQUAL "-lpq")
        set(LIBPQ_LIBRARIES "${LIBPQ_LIBRARY_DIRS}/libpq.a")
        message(STATUS "Converted -lpq to full path: ${LIBPQ_LIBRARIES}")
    endif()

    target_link_libraries(${PROJ_NAME}
        PRIVATE
        libpqxx::pqxx

        -Wl,--start-group
        ${POSTGRES_SERVER_DEV_STATIC_LIBS}
        ${POSTGRES_OAUTH_LIBS}
        ${LIBPQ_LIBRARIES}
        -Wl,--end-group

        curl
        ssl
        crypto
        gssapi_krb5
        ldap
    )
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)

if (MSVC)
    file(TO_CMAKE_PATH "${GSTREAMER_ROOT}" GST_ROOT_CMAKE)
    file(GLOB BINARY_DATA "${GST_ROOT_CMAKE}/bin/*.dll")
    add_custom_command(TARGET ${PROJ_NAME} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${BINARY_DATA} ${CMAKE_INSTALL_PREFIX}/bin)
elseif (APPLE)
    file(GLOB PostgreSQL_LIBRARIES "${PostgreSQL_ROOT}/lib/*.dylib")
    add_custom_command(TARGET ${PROJ_NAME} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PostgreSQL_LIBRARIES} ${CMAKE_INSTALL_PREFIX}/bin)
else()
    install(FILES $<TARGET_FILE:libpqxx::pqxx> DESTINATION bin)
endif()