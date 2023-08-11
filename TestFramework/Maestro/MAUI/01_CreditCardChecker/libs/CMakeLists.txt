cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Luhn)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_OBJCXX_STANDARD 17)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# add sources
FILE(GLOB_RECURSE HEADERS ${PROJECT_SOURCE_DIR}/include/*.h)
FILE(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_library(${PROJ_NAME} SHARED ${HEADERS}
                                ${SOURCES})

# create cmake.config
install(TARGETS ${PROJ_NAME} 
        EXPORT ${PROJ_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include/
)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT ${PROJ_NAME}Targets
        FILE ${PROJ_NAME}Config.cmake
        DESTINATION share/cmake/${PROJ_NAME}
)
export(EXPORT ${PROJ_NAME}Targets
       FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJ_NAME}Config.cmake"
)
export(PACKAGE ${PROJ_NAME})