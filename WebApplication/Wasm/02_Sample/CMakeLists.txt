cmake_minimum_required(VERSION 3.10)

set(PROJ_NAME Demo)
project(${PROJ_NAME} VERSION 1.0.0)

set(CMAKE_CXX_COMPILER em++)
set(CMAKE_C_COMPILER emcc)

add_executable(${PROJ_NAME} library.cpp)

set_target_properties(${PROJ_NAME} PROPERTIES SUFFIX ".wasm")
set_target_properties(${PROJ_NAME} PROPERTIES
    LINK_FLAGS "-Os -s WASM=1 -s SIDE_MODULE=1 -s EXPORTED_FUNCTIONS='[_add]'"
)

install(FILES
        "${CMAKE_BINARY_DIR}/${PROJ_NAME}.wasm"
        DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)