cmake_minimum_required(VERSION 3.9.0)
set(PROJ_NAME Test)

project(${PROJ_NAME} VERSION 1.0.0)

add_executable(${PROJ_NAME} main.cpp)

target_include_directories(${PROJ_NAME}
                           PRIVATE ${PROJECT_SOURCE_DIR}/libsodium-1.0.18/include)

if(CMAKE_GENERATOR MATCHES "Visual Studio")
set(STATIC_LIBRARIES
    optimized ${PROJECT_SOURCE_DIR}/libsodium-1.0.18/x64/Release/v141/static/libsodium.lib
    debug     ${PROJECT_SOURCE_DIR}/libsodium-1.0.18/x64/Debug/v141/static/libsodium.lib
)
target_link_libraries(${PROJ_NAME} PUBLIC ${STATIC_LIBRARIES})
endif()

# copy data to run program
set(DATA
    ${CMAKE_CURRENT_SOURCE_DIR}/Lenna.png
    ${CMAKE_CURRENT_SOURCE_DIR}/run.ps1
)
add_custom_command(TARGET ${PROJ_NAME} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different
                           ${DATA}
                           $<TARGET_FILE_DIR:${PROJ_NAME}>
                   )

set(CompilerFlags
CMAKE_CXX_FLAGS
CMAKE_CXX_FLAGS_DEBUG
CMAKE_CXX_FLAGS_RELEASE
CMAKE_C_FLAGS
CMAKE_C_FLAGS_DEBUG
CMAKE_C_FLAGS_RELEASE
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
