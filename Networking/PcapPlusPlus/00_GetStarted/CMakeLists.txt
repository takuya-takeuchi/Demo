cmake_minimum_required(VERSION 3.5.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(PcapPlusPlus CONFIG)
message("    PcapPlusPlus_VERSION: ${PcapPlusPlus_VERSION}")
message("PcapPlusPlus_INCLUDE_DIR: ${PcapPlusPlus_INCLUDE_DIR}")
message("  PcapPlusPlus_LIBRARIES: ${PcapPlusPlus_LIBRARIES}")

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${PcapPlusPlus_INCLUDE_DIR})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          "${PcapPlusPlus_INCLUDE_DIR}/../../lib/Common++.lib"
                          "${PcapPlusPlus_INCLUDE_DIR}/../../lib/Packet++.lib"
                          "${PcapPlusPlus_INCLUDE_DIR}/../../lib/Pcap++.lib"
                          Ws2_32.lib)
elseif (APPLE)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${PcapPlusPlus_INCLUDE_DIR})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${PCAP_LIBRARIES})
else()
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${PcapPlusPlus_INCLUDE_DIR})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${PCAP_LIBRARIES})
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)