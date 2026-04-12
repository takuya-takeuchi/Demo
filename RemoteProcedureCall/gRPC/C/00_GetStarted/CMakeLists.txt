cmake_minimum_required(VERSION 3.26.0)

set(PROJ_NAME greeter)
project(${PROJ_NAME} VERSION 1.0.0)

find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)

set(PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/proto/greeter.proto)
get_filename_component(PROTO_DIR ${PROTO_FILE} PATH)

set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${GENERATED_DIR})

set(PROTO_SRCS ${GENERATED_DIR}/greeter.pb.cc)
set(PROTO_HDRS ${GENERATED_DIR}/greeter.pb.h)
set(GRPC_SRCS  ${GENERATED_DIR}/greeter.grpc.pb.cc)
set(GRPC_HDRS  ${GENERATED_DIR}/greeter.grpc.pb.h)

add_custom_command(
    OUTPUT ${PROTO_SRCS} ${PROTO_HDRS} ${GRPC_SRCS} ${GRPC_HDRS}
    COMMAND protobuf::protoc
    --proto_path=${PROTO_DIR}
    --cpp_out=${GENERATED_DIR}
    --grpc_out=${GENERATED_DIR}
    --plugin=protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>
    ${PROTO_FILE}
    DEPENDS ${PROTO_FILE}
    COMMENT "Generating protobuf and gRPC sources from ${PROTO_FILE}"
    VERBATIM
)

add_library(${PROJ_NAME}
            ${PROTO_SRCS}
            ${GRPC_SRCS}
)

target_include_directories(${PROJ_NAME} PUBLIC
                           ${GENERATED_DIR}
)

# This definition will be propagated to the client and service targets that link against this library
target_link_libraries(${PROJ_NAME} PUBLIC
    protobuf::libprotobuf
    gRPC::grpc++
)

add_subdirectory(client)
add_subdirectory(service)