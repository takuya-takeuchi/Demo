cmake_minimum_required(VERSION 3.26.0)

set(PROJ_NAME client)
project(${PROJ_NAME} VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if (MSVC)
elseif (APPLE)
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ldl")
endif()

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    set(LIBRARIES
        greeter
        protobuf::libprotobuf
        gRPC::grpc++
    )
elseif (APPLE)
    set(LIBRARIES
        greeter
        protobuf::libprotobuf
        gRPC::grpc++
    )
else()
    set(LIBRARIES
        greeter
        protobuf::libprotobuf
        gRPC::grpc++
    )
endif()

target_link_libraries(${PROJ_NAME} PRIVATE ${LIBRARIES})

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)