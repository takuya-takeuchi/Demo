cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# if (MSVC)
#     set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
# endif()

find_package(benchmark REQUIRED)
if (${benchmark_FOUND})
    message("    benchmark_VERSION: ${benchmark_VERSION}")
endif()

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               benchmark::benchmark)

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          benchmark::benchmark)
elseif (APPLE)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               benchmark::benchmark)

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          benchmark::benchmark)
else()
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               benchmark::benchmark)

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          benchmark::benchmark)
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)