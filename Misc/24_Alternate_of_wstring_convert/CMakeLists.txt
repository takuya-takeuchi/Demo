cmake_minimum_required(VERSION 3.0.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if (MSVC)
elseif (APPLE)
    find_package(iconv REQUIRED)
    message("Iconv_INCLUDE_DIRS: ${Iconv_INCLUDE_DIRS}")
    message("   Iconv_LIBRARIES: ${Iconv_LIBRARIES}")
else()
    find_package(Iconv REQUIRED)
    message("Iconv_INCLUDE_DIRS: ${Iconv_INCLUDE_DIRS}")
    message("   Iconv_LIBRARIES: ${Iconv_LIBRARIES}")
endif()

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
elseif (APPLE)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${iconv_INCLUDE_DIR})
    
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${Iconv_LIBRARIES})
else()
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               "/usr/local/include")
    
    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          iconv)
endif()

# define install taget
install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)