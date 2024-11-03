cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(Boost REQUIRED COMPONENTS system filesystem python)

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
message(" Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message("Python_INCLUDE_DIRS: ${Python_INCLUDE_DIRS}")
message("    Boost_LIBRARIES: ${Boost_LIBRARIES}")
message("   Python_LIBRARIES: ${Python_LIBRARIES}")

add_executable(${PROJ_NAME} ${PROJECT_SOURCE_DIR}/main.cpp)

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           ${Python_INCLUDE_DIRS}
                           ${Boost_INCLUDE_DIRS})

target_link_libraries(${PROJ_NAME}
                      PRIVATE
                      ${Python_LIBRARIES}
                      ${Boost_LIBRARIES})

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)
install(FILES ${PROJECT_SOURCE_DIR}/PythonMod.py DESTINATION bin)