cmake_minimum_required(VERSION 3.0.0)
cmake_policy(SET CMP0053 NEW)
cmake_policy(SET CMP0054 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PROJ_NAME Demo)

project(${PROJ_NAME} VERSION 1.0.0)

add_executable(${PROJ_NAME} ${PROJECT_SOURCE_DIR}/main.cpp)

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)
