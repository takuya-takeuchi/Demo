cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Test)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
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