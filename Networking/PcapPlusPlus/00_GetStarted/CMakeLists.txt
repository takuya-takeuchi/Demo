cmake_minimum_required(VERSION 3.5.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    find_package(PcapPlusPlus CONFIG)
    message("    PcapPlusPlus_VERSION: ${PcapPlusPlus_VERSION}")
    message("PcapPlusPlus_INCLUDE_DIR: ${PcapPlusPlus_INCLUDE_DIR}")

    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${PcapPlusPlus_INCLUDE_DIR})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          "${PcapPlusPlus_DIR}/../../Common++.lib"
                          "${PcapPlusPlus_DIR}/../../Packet++.lib"
                          "${PcapPlusPlus_DIR}/../../Pcap++.lib"
                          "${PCAP_ROOT}/Lib/x64/Packet.lib"
                          "${PCAP_ROOT}/Lib/x64/wpcap.lib"
                          Iphlpapi.lib
                          Ws2_32.lib)
elseif (APPLE)
    find_package(PcapPlusPlus REQUIRED)
    message("    PcapPlusPlus_VERSION: ${PcapPlusPlus_VERSION}")
    message("PcapPlusPlus_INCLUDE_DIR: ${PcapPlusPlus_INCLUDE_DIR}")

    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${PcapPlusPlus_INCLUDE_DIR})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          PcapPlusPlus::Pcap++)
else()
    find_package(PcapPlusPlus REQUIRED)
    message("    PcapPlusPlus_VERSION: ${PcapPlusPlus_VERSION}")
    message("PcapPlusPlus_INCLUDE_DIR: ${PcapPlusPlus_INCLUDE_DIR}")
    
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${PcapPlusPlus_INCLUDE_DIR})

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          PcapPlusPlus::Pcap++)
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)

if (MSVC)
    install(FILES "$ENV{windir}/System32/Npcap/Packet.dll" DESTINATION bin)
    install(FILES "$ENV{windir}/System32/Npcap/wpcap.dll" DESTINATION bin)
endif()