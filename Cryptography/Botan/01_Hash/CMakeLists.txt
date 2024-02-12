cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Test)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

add_executable(${PROJ_NAME} main.cpp)

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           ${BOTAN_ROOT_DIR}/include/botan-3)

if (MSVC)
    target_link_libraries(${PROJ_NAME}
                        PRIVATE
                        ${BOTAN_ROOT_DIR}/lib/botan-3.lib)
elseif (APPLE)
    target_link_libraries(${PROJ_NAME}
                        PRIVATE
                        ${BOTAN_ROOT_DIR}/lib/libbotan-3.a)
else()
    target_link_libraries(${PROJ_NAME}
                        PRIVATE
                        ${BOTAN_ROOT_DIR}/lib/libbotan-3.a)
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)