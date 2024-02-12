cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Test)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

#find_package(Botan REQUIRED)
#find_package(PkgConfig)
#pkg_check_modules(BOTAN2 botan-2)
find_library(BOTAN2 botan2 PATHS $CMAKE_PREFIX_PATH)

if (NOT BOTAN2_FOUND)
    message(FATAL_ERROR "Botan is not found!!")
else()
    message("BOTAN2_INCLUDE_DIRS: ${Botan_INCLUDE_DIRS}")
    message("   Botan_LIBRARIES: ${Botan_LIBRARIES}")
endif()

add_executable(${PROJ_NAME} main.cpp)

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           ${TINY_AES_C_INSTALL_DIR})

if (MSVC)
    target_link_libraries(${PROJ_NAME}
                        PRIVATE
                        ${TINY_AES_C_INSTALL_DIR}/tiny-aes.lib)
elseif (APPLE)
    target_link_libraries(${PROJ_NAME}
                        PRIVATE
                        ${TINY_AES_C_INSTALL_DIR}/libtiny-aes.a)
else()
    target_link_libraries(${PROJ_NAME}
                        PRIVATE
                        ${TINY_AES_C_INSTALL_DIR}/libtiny-aes.a)
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)