cmake_minimum_required(VERSION 3.9.0)
set(PROJ_NAME Minimum)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(OpenCV 4.5.5 EXACT REQUIRED)
find_package(Eigen3 REQUIRED CONFIG)

add_executable(${PROJ_NAME} ../main.cpp)

if(OpenCV_FOUND)
    if (MSVC OR MSYS OR MINGW)
        message("# Build for Windows")
        message("${OpenCV_LIBRARIES}")
        target_include_directories(${PROJ_NAME}
            PUBLIC ${OpenCV_INCLUDE_DIRS}
        )
        set(STATIC_LIBRARIES
            "opencv_world"
        )
        target_link_libraries(${PROJ_NAME}
            ${STATIC_LIBRARIES}
        )

	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /Gy")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Gy")

        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /OPT:REF")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /OPT:REF")
        set(CMAKE_STATIC_LINKER_FLAGS "${CMAKE_STATIC_LINKER_FLAGS} /OPT:REF")
    elseif(APPLE)
    elseif(UNIX AND NOT APPLE)
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--as-needed -Wl,--gc-sections -Wl,--strip-all")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--as-needed -Wl,--gc-sections -Wl,--strip-all")
        set(CMAKE_STATIC_LINKER_FLAGS "${CMAKE_STATIC_LINKER_FLAGS} -Wl,--as-needed -Wl,--gc-sections -Wl,--strip-all")
    else()
        message(FATAL_ERROR "Failed to link library")
    endif()
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
