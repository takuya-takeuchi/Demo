cmake_minimum_required(VERSION 3.26.0)

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
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ldl")
endif()

find_package(OpenCV)

if (${OpenCV_FOUND})
    message("     OpenCV_VERSION: ${OpenCV_VERSION}")
    message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
    message("   OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
endif()

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    set_property(TARGET ${PROJ_NAME} PROPERTY
        MSVC_RUNTIME_LIBRARY MultiThreaded$<$<CONFIG:Debug>:Debug>
    )
    
    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${ONNXRUNTIME_ROOT}/include
                               ${OpenCV_INCLUDE_DIRS})

    file(GLOB ORT_LIBS      "${ONNXRUNTIME_ROOT}/lib/*.lib")
    file(GLOB ORT_LIBS_DEPS "${ONNXRUNTIME_ROOT}/lib/deps/*.lib")

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${OpenCV_LIBRARIES}
                          ${ORT_LIBS}
                          ${ORT_LIBS_DEPS})
elseif (APPLE)
    find_package(Iconv REQUIRED)

    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${ONNXRUNTIME_ROOT}/include
                               Iconv::Iconv
                               ${OpenCV_INCLUDE_DIRS})

    file(GLOB ORT_LIBS      "${ONNXRUNTIME_ROOT}/lib/*.a")
    file(GLOB ORT_LIBS_DEPS "${ONNXRUNTIME_ROOT}/lib/deps/*.a")

    target_link_libraries(${PROJ_NAME}
                          PRIVATE
                          ${OpenCV_LIBRARIES}
                          Iconv::Iconv
                          ${ORT_LIBS}
                          ${ORT_LIBS_DEPS})
else()
    find_package(Iconv REQUIRED)

    target_include_directories(${PROJ_NAME}
                               PRIVATE
                               ${ONNXRUNTIME_ROOT}/include
                               Iconv::Iconv
                               ${OpenCV_INCLUDE_DIRS})

    set(LINKER_FLAGS "${LINKER_FLAGS} -Wl,--as-needed -Wl,--gc-sections -Wl,--no-undefined -Wl,--strip-all")

    file(GLOB ORT_LIBS      "${ONNXRUNTIME_ROOT}/lib/*.a")
    file(GLOB ORT_LIBS_DEPS "${ONNXRUNTIME_ROOT}/lib/deps/*.a")

    target_link_libraries(${PROJ_NAME} PRIVATE
                          Iconv::Iconv
                          ${OpenCV_LIBRARIES}
                          -Wl,--whole-archive
                          ${ORT_LIBS}
                          ${ORT_LIBS_DEPS}
                          -Wl,--no-whole-archive
                          dl
                          z
                          pthread)
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)

if (MSVC)
    set(BINARY_DATA
        ${ONNXRUNTIME_ROOT}/lib/onnxruntime_providers_shared.dll
        ${ONNXRUNTIME_ROOT}/lib/onnxruntime_providers_cuda.dll
        ${CMAKE_CURRENT_SOURCE_DIR}/testdata/mnist-7.bmp
        ${CMAKE_CURRENT_SOURCE_DIR}/testdata/mnist.onnx
    )
elseif (APPLE)
    set(BINARY_DATA
        ${ONNXRUNTIME_ROOT}/lib/libonnxruntime_providers_shared.so
        ${CMAKE_CURRENT_SOURCE_DIR}/testdata/mnist-7.bmp
        ${CMAKE_CURRENT_SOURCE_DIR}/testdata/mnist.onnx
    )
else()
    set(BINARY_DATA
        ${ONNXRUNTIME_ROOT}/lib/libonnxruntime_providers_shared.so
        ${ONNXRUNTIME_ROOT}/lib/libonnxruntime_providers_cuda.so
        ${CMAKE_CURRENT_SOURCE_DIR}/testdata/mnist-7.bmp
        ${CMAKE_CURRENT_SOURCE_DIR}/testdata/mnist.onnx
    )
endif()

add_custom_command(TARGET ${PROJ_NAME} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different ${BINARY_DATA} ${CMAKE_INSTALL_PREFIX}/bin)