cmake_minimum_required(VERSION 3.15.0)

set(PROJ_NAME Demo1)

project(${PROJ_NAME} VERSION 1.0.0)

if(MSVC)
    add_compile_options(
        $<$<CONFIG:>:/MT>
        $<$<CONFIG:Debug>:/MTd>
        $<$<CONFIG:RelWithDebInfo>:/MT>
        $<$<CONFIG:MinSizeRel>:/MT>
        $<$<CONFIG:Release>:/MT>
    )
endif()

add_executable(${PROJ_NAME} ${PROJECT_SOURCE_DIR}/../main.cpp)

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)