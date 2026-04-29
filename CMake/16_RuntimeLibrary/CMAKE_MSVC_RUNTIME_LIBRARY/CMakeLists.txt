cmake_minimum_required(VERSION 3.15.0)

set(PROJ_NAME Demo4)

project(${PROJ_NAME} VERSION 1.0.0)

add_executable(${PROJ_NAME} ${PROJECT_SOURCE_DIR}/../main.cpp)

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)