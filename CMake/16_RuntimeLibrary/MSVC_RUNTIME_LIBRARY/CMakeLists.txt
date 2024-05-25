cmake_minimum_required(VERSION 3.15.0)

set(PROJ_NAME Demo3)

project(${PROJ_NAME} VERSION 1.0.0)

add_executable(${PROJ_NAME} ${PROJECT_SOURCE_DIR}/../main.cpp)

set_property(TARGET ${PROJ_NAME} PROPERTY
    MSVC_RUNTIME_LIBRARY MultiThreaded$<$<CONFIG:Debug>:Debug>
)

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)