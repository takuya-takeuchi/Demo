cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if (MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
endif()

if (MSVC)
    find_package(iconv REQUIRED)
    
    message("ICONV_INCLUDE_DIR: $ENV{ICONV_INCLUDE_DIR}")
    message("   CHARSET_BINARY: $ENV{CHARSET_BINARY}")
    message("     ICONV_BINARY: $ENV{ICONV_BINARY}")
elseif (APPLE)
    find_package(iconv REQUIRED)
    message("Iconv_INCLUDE_DIRS: ${Iconv_INCLUDE_DIRS}")
    message("   Iconv_LIBRARIES: ${Iconv_LIBRARIES}")
else()
    find_package(Iconv REQUIRED)
    message("Iconv_INCLUDE_DIRS: ${Iconv_INCLUDE_DIRS}")
    message("   Iconv_LIBRARIES: ${Iconv_LIBRARIES}")
endif()

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               Iconv::Iconv)
    
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          Iconv::Iconv)

    install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)
    install(FILES sjis.txt DESTINATION bin)
    install(FILES $ENV{CHARSET_BINARY} DESTINATION bin)
    install(FILES $ENV{ICONV_BINARY} DESTINATION bin)
elseif (APPLE)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${iconv_INCLUDE_DIR})
    
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${Iconv_LIBRARIES})

    install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)
    install(FILES sjis.txt DESTINATION bin)
else()
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${iconv_INCLUDE_DIR})
    
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${Iconv_LIBRARIES})

    install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)
    install(FILES sjis.txt DESTINATION bin)
endif()