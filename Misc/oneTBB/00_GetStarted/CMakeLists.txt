cmake_minimum_required(VERSION 3.5.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(TBB REQUIRED)

message("    TBB_VERSION: ${TBB_VERSION}")
message("        TBB_DIR: ${TBB_DIR}")

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               TBB::tbb)

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          TBB::tbb)
elseif (APPLE)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               TBB::tbb)

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          TBB::tbb)
else()
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               TBB::tbb)

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          TBB::tbb)
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)

if (MSVC)
    install(FILES "${TBB_DIR}/../../../bin/tbb12.dll" DESTINATION bin)
    install(FILES "${TBB_DIR}/../../../bin/tbbmalloc_proxy.dll" DESTINATION bin)
    install(FILES "${TBB_DIR}/../../../bin/tbbmalloc.dll" DESTINATION bin)
elseif (APPLE)
else()
    install(FILES "${TBB_DIR}/../../../lib/libtbb.so" DESTINATION bin)
    install(FILES "${TBB_DIR}/../../../lib/libtbb.so.12" DESTINATION bin)
    install(FILES "${TBB_DIR}/../../../lib/libtbb.so.12.16" DESTINATION bin)
    install(FILES "${TBB_DIR}/../../../lib/libtbbmalloc.so" DESTINATION bin)
    install(FILES "${TBB_DIR}/../../../lib/libtbbmalloc.so.2" DESTINATION bin)
    install(FILES "${TBB_DIR}/../../../lib/libtbbmalloc.so.2.16" DESTINATION bin)
    install(FILES "${TBB_DIR}/../../../lib/libtbbmalloc_proxy.so" DESTINATION bin)
    install(FILES "${TBB_DIR}/../../../lib/libtbbmalloc_proxy.so.2" DESTINATION bin)
    install(FILES "${TBB_DIR}/../../../lib/libtbbmalloc_proxy.so.2.16" DESTINATION bin)
endif()