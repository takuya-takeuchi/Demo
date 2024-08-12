cmake_minimum_required(VERSION 3.10)

set(PROJ_NAME Demo)
project(${PROJ_NAME} VERSION 1.0.0)

set(CMAKE_CXX_COMPILER em++)
set(CMAKE_C_COMPILER emcc)

set(CMAKE_C_FLAGS "-fPIC")
set(CMAKE_CXX_FLAGS "-fPIC")

FILE(GLOB_RECURSE OpenCV_LIBRARIES $ENV{OpenCV_DIR}/*.a)

add_executable(${PROJ_NAME} library.cpp)

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           $ENV{OpenCV_DIR}/include/opencv4)

target_link_libraries(${PROJ_NAME}
                      PRIVATE
                      ${OpenCV_LIBRARIES})

set_target_properties(${PROJ_NAME} PROPERTIES SUFFIX ".js")
set_target_properties(${PROJ_NAME} PROPERTIES
    LINK_FLAGS "-Os -s WASM=1 -s MODULARIZE=1 -s EXPORTED_FUNCTIONS='[_showBuildInformation]' -s EXPORT_NAME=Demo"
)

install(FILES
        "${CMAKE_BINARY_DIR}/${PROJ_NAME}.wasm"
        "${CMAKE_BINARY_DIR}/${PROJ_NAME}.js"
        DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)