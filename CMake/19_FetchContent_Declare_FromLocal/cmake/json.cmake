# 1. build and source will be output into _deps/json-build (json_BINARY_DIR) and _deps/json-src (json_SOURCE_DIR)
FetchContent_Declare(
  json
  URL ${CMAKE_SOURCE_DIR}/patch/v3.12.0.zip
)
FetchContent_MakeAvailable(json)

if (MSVC)
  set(JSON_INCLUDE_DIR ${json_SOURCE_DIR}/include)
elseif (APPLE)
  set(JSON_INCLUDE_DIR ${json_SOURCE_DIR}/include)
else()
  set(JSON_INCLUDE_DIR ${json_SOURCE_DIR}/include)
endif()