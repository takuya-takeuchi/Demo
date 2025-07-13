# 1. build and source will be output into _deps/zlib-build (zlib_BINARY_DIR) and _deps/zlib-src (zlib_SOURCE_DIR)
# 2. CMAKE_ARGS is ignored. Refer: https://discourse.cmake.org/t/fetchcontent-declare-with-cmake-args-does-not-work/11227/7
FetchContent_Declare(
  zlib
  GIT_REPOSITORY https://github.com/madler/zlib
  GIT_TAG v1.3.1
)
set(ZLIB_INSTALL       OFF)
set(SKIP_INSTALL_ALL   OFF)
set(ZLIB_BUILD_STATIC  ON)
set(ZLIB_BUILD_SHARED  OFF)
set(BUILD_TESTING      OFF)
set(ZLIB_BUILD_MINIZIP OFF)
FetchContent_MakeAvailable(zlib)

if (MSVC)
  set(ZLIB_INCLUDE_DIR ${zlib_SOURCE_DIR})
  set(ZLIB_LIBRARIES ${zlib_BINARY_DIR}/z.lib)
elseif (APPLE)
  set(ZLIB_INCLUDE_DIR ${zlib_SOURCE_DIR})
  set(ZLIB_LIBRARIES ${zlib_BINARY_DIR}/libz.a)
else()
  set(ZLIB_INCLUDE_DIR ${zlib_SOURCE_DIR})
  set(ZLIB_LIBRARIES ${zlib_BINARY_DIR}/libz.a)
endif()