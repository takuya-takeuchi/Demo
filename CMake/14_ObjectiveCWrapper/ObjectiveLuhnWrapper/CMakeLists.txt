cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME LuhnC)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_OBJCXX_STANDARD 17)

# make enable ARC (Automatic Reference Counting)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fobjc-arc")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fobjc-arc")
set(CMAKE_OBJC_FLAGS "${CMAKE_OBJC_FLAGS} -fobjc-arc")
set(CMAKE_OBJCXX_FLAGS "${CMAKE_OBJCXX_FLAGS} -fobjc-arc")

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# for NSCharacterSet, SMutableString...
find_library(FOUNDATION Foundation)

# add sources
FILE(GLOB_RECURSE HEADERS ${PROJECT_SOURCE_DIR}/include/*.h)
FILE(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/src/*.mm)

add_library(${PROJ_NAME} SHARED ${HEADERS}
                                ${SOURCES})

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           ${FOUNDATION}
                           ${LUHN_SDK_DIR}/include)

target_link_libraries(${PROJ_NAME}
                      PRIVATE
                      ${FOUNDATION}
                      ${LUHN_SDK_DIR}/lib/libLuhn.a)

INSTALL(TARGETS ${PROJ_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include)