cmake_minimum_required(VERSION 3.9.0)
set(PROJ_NAME Test)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# target source and header files
FILE(GLOB_RECURSE HEADERS ${PROJECT_SOURCE_DIR}/*.h)
FILE(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/*.cpp)

include_directories(${BOOST_ROOT})

#add_executable(${PROJ_NAME} ${HEADERS} ${SOURCES})
add_executable(${PROJ_NAME} main.cpp)

if(CMAKE_GENERATOR MATCHES "Visual Studio")
set(STATIC_LIBRARIES
    ${BOOST_LIBRARYDIR}/libboost_system-vc140-mt-1_65_1.lib
    ${BOOST_LIBRARYDIR}/libboost_filesystem-vc140-mt-1_65_1.lib
)
target_link_libraries(${PROJ_NAME} ${STATIC_LIBRARIES})
else()
set(STATIC_LIBRARIES
    ${BOOST_LIBRARYDIR}/libboost_system.a
    ${BOOST_LIBRARYDIR}/libboost_filesystem.a
)
target_link_libraries(${PROJ_NAME} ${STATIC_LIBRARIES})
endif()

set(CompilerFlags
CMAKE_CXX_FLAGS
CMAKE_CXX_FLAGS_DEBUG
CMAKE_CXX_FLAGS_RELEASE
CMAKE_C_FLAGS
CMAKE_C_FLAGS_DEBUG
CMAKE_C_FLAGS_RELEASE
)
foreach(CompilerFlag ${CompilerFlags})
string(REPLACE "/MDd" "/MD" ${CompilerFlag} "${${CompilerFlag}}")
endforeach()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
