cmake_minimum_required(VERSION 3.5.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(websocketpp REQUIRED)
# https://www.boost.org/doc/libs/develop/doc/html/boost_asio/using.html
# build option to use asio is `--with-system --with-thread --with-date_time --with-regex --with-serialization`
find_package(Boost REQUIRED COMPONENTS system thread date_time regex serialization)

if (WEBSOCKETPP_FOUND)
    message("WEBSOCKETPP_INCLUDE_DIR: {WEBSOCKETPP_INCLUDE_DIR}")
endif()

if (Boost_FOUND)
    message("     Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
    message("        Boost_LIBRARIES: ${Boost_LIBRARIES}")
endif()


# Use c++ standard library instead of boost
add_definitions (-D_WEBSOCKETPP_CPP11_FUNCTIONAL_)
add_definitions (-D_WEBSOCKETPP_CPP11_SYSTEM_ERROR_)
add_definitions (-D_WEBSOCKETPP_CPP11_RANDOM_DEVICE_)
add_definitions (-D_WEBSOCKETPP_CPP11_MEMORY_)

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${WEBSOCKETPP_INCLUDE_DIR}
                               ${Boost_INCLUDE_DIRS})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${Boost_LIBRARIES})
elseif (APPLE)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${WEBSOCKETPP_INCLUDE_DIR}
                               ${Boost_INCLUDE_DIRS})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${Boost_LIBRARIES})
else()
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${WEBSOCKETPP_INCLUDE_DIR}
                               ${Boost_INCLUDE_DIRS})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${Boost_LIBRARIES})
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)