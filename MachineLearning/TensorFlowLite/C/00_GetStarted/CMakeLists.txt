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
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
elseif (APPLE)
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ldl")
endif()

add_executable(${PROJ_NAME} main.cpp)

if (MSVC)
    target_include_directories(${PROJ_NAME}
                               PRIVATE ${TFLITE_INCLUDE_DIR}
    )

    target_link_libraries(${PROJ_NAME}
                          PRIVATE ${TFLITE_LIBS}
    )
elseif (APPLE)
else()
endif()

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)

if (MSVC)
   set(BINARY_DATA
       ${TFLITE_BINS}
       ${CMAKE_CURRENT_SOURCE_DIR}/saved_model/mobilenetv2-12_dynamic_range_quant.tflite
       ${CMAKE_CURRENT_SOURCE_DIR}/saved_model/mobilenetv2-12_float16.tflite
       ${CMAKE_CURRENT_SOURCE_DIR}/saved_model/mobilenetv2-12_float32.tflite
       ${CMAKE_CURRENT_SOURCE_DIR}/saved_model/mobilenetv2-12_full_integer_quant_with_int16_act.tflite
       ${CMAKE_CURRENT_SOURCE_DIR}/saved_model/mobilenetv2-12_full_integer_quant.tflite
       ${CMAKE_CURRENT_SOURCE_DIR}/saved_model/mobilenetv2-12_integer_quant_with_int16_act.tflite
       ${CMAKE_CURRENT_SOURCE_DIR}/saved_model/mobilenetv2-12_integer_quant.tflite
   )
elseif (APPLE)
else()
endif()

add_custom_command(TARGET ${PROJ_NAME} POST_BUILD
                  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${BINARY_DATA} ${CMAKE_INSTALL_PREFIX}/bin)