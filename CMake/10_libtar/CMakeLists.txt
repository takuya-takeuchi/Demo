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
    message(FATAL_ERROR "Windows is not supported!!")
else()
    find_library(libtar_LIBS
                 NAMES libtar.a tar libtar
                 PATHS ${LIBTAR_ROOT_DIR} "/usr/local" ${CMAKE_SYSTEM_PREFIX_PATH}
                 PATH_SUFFIXES lib lib64)
    if(${libtar_LIBS} STREQUAL libtar_LIBS-NOTFOUND)
        message(FATAL_ERROR "libtar could not be found. Try to specify it's location with `-DLIBTAR_ROOT_DIR`.")
    endif()

    message("libtar_LIBS: ${libtar_LIBS}")
endif()

add_executable(${PROJ_NAME} main.cpp)

#target_include_directories(${PROJ_NAME}
#                           PRIVATE
#                           ${Boost_INCLUDE_DIRS})

target_link_libraries(${PROJ_NAME}
                      PRIVATE
                      ${libtar_LIBS})