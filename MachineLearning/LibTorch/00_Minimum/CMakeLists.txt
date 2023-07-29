cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if (MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
endif()

find_package(Torch REQUIRED)

message("   TORCH_LIBRARIES: ${TORCH_LIBRARIES}")
message("TORCH_INCLUDE_DIRS: ${TORCH_INCLUDE_DIRS}")
message("Protobuf_LIBRARIES: $ENV{Protobuf_LIBRARIES}")

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${TORCH_INCLUDE_DIRS})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${TORCH_LIBRARIES}
                          $ENV{Protobuf_LIBRARIES})
elseif (APPLE)
    message("sleef_LIBRARIES: $ENV{sleef_LIBRARIES}")
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${TORCH_INCLUDE_DIRS})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${TORCH_LIBRARIES}
                          $ENV{Protobuf_LIBRARIES}
                          $ENV{Protobuf_LIBRARIES})
else()
    message("sleef_LIBRARIES: $ENV{sleef_LIBRARIES}")
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${TORCH_INCLUDE_DIRS})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${TORCH_LIBRARIES}
                          $ENV{Protobuf_LIBRARIES}
                          $ENV{sleef_LIBRARIES})
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)