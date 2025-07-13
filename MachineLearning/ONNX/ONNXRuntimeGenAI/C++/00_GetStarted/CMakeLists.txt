cmake_minimum_required(VERSION 3.26.0)

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

# Select the release build type by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# if (MSVC)
# elseif (APPLE)
# else ()
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ldl")
# endif()

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${ONNXRUNTIME_GENAI_ROOT}/include
    )

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${ONNXRUNTIME_GENAI_ROOT}/lib/onnxruntime_providers_shared.lib
                          ${ONNXRUNTIME_GENAI_ROOT}/lib/onnxruntime-genai.lib
                          ${ONNXRUNTIME_GENAI_ROOT}/lib/onnxruntime.lib
    )
elseif (APPLE)
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${ONNXRUNTIME_GENAI_ROOT}/include
    )

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${ONNXRUNTIME_GENAI_ROOT}/lib/libonnxruntime-genai.dylib
                          ${ONNXRUNTIME_GENAI_ROOT}/lib/libonnxruntime.dylib
    )
else()
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${ONNXRUNTIME_GENAI_ROOT}/include
    )

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${ONNXRUNTIME_GENAI_ROOT}/lib/libonnxruntime_providers_shared.so
                          ${ONNXRUNTIME_GENAI_ROOT}/lib/libonnxruntime-genai.so
                          ${ONNXRUNTIME_GENAI_ROOT}/lib/libonnxruntime.so
    )
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)

if (MSVC)
    set(BINARY_DATA
        ${ONNXRUNTIME_GENAI_ROOT}/lib/onnxruntime_providers_shared.dll
        ${ONNXRUNTIME_GENAI_ROOT}/lib/onnxruntime-genai.dll
        ${ONNXRUNTIME_GENAI_ROOT}/lib/onnxruntime.dll
    )
elseif (APPLE)
    set(BINARY_DATA
        ${ONNXRUNTIME_GENAI_ROOT}/lib/libonnxruntime-genai.dylib
        ${ONNXRUNTIME_GENAI_ROOT}/lib/libonnxruntime.dylib
    )
else()
    set(BINARY_DATA
        ${ONNXRUNTIME_GENAI_ROOT}/lib/libonnxruntime_providers_shared.so
        ${ONNXRUNTIME_GENAI_ROOT}/lib/libonnxruntime-genai.so
        ${ONNXRUNTIME_GENAI_ROOT}/lib/libonnxruntime.so
    )
endif()

add_custom_command(TARGET ${PROJ_NAME} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different ${BINARY_DATA} ${CMAKE_INSTALL_PREFIX}/bin)
install(FILES australia.jpg DESTINATION bin)