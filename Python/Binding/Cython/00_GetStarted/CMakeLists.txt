cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if( ${CMAKE_VERSION} LESS 3.12 )
    set(PYTHON_EXECUTABLE ${Python_EXECUTABLE})

    find_package(PythonInterp ${Python_FIND_VERSION} REQUIRED)
    find_package(PythonLibs ${Python_FIND_VERSION} REQUIRED)
    set(Python_INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS})
    set(Python_LIBRARIES ${PYTHON_LIBRARIES})
else()
    find_package(Python ${Python_FIND_VERSION} REQUIRED COMPONENTS Interpreter Development)
endif()

message("  Python_EXECUTABLE: ${Python_EXECUTABLE}")
message("Python_INCLUDE_DIRS: ${Python_INCLUDE_DIRS}")
message("   Python_LIBRARIES: ${Python_LIBRARIES}")

add_executable(${PROJ_NAME} ${PROJECT_SOURCE_DIR}/main.cpp)

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           ${Python_INCLUDE_DIRS})

target_link_libraries(${PROJ_NAME}
                      PRIVATE
                      ${Python_LIBRARIES})

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)
if (MSVC)
    file(GLOB_RECURSE PYD_FILES ${CMAKE_SOURCE_DIR}/*.pyd)
else()
    file(GLOB_RECURSE PYD_FILES ${CMAKE_SOURCE_DIR}/*.so)
endif()
foreach(pydFile ${PYD_FILES})
    get_filename_component(file_name ${pydFile} NAME)
    install(FILES ${pydFile} DESTINATION bin RENAME ${file_name})
endforeach()

if (MSVC)
    install(FILES ${Python_RUNTIME_LIBRARY} DESTINATION bin)
endif()