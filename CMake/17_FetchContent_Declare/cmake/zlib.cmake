# 1. build and source will be output into _deps/zlib-build (zlib_BINARY_DIR) and _deps/zlib-src (zlib_SOURCE_DIR)
# 2. CMAKE_ARGS is ignored. Refer: https://discourse.cmake.org/t/fetchcontent-declare-with-cmake-args-does-not-work/11227/7
FetchContent_Declare(
  zlib
  GIT_REPOSITORY https://github.com/madler/zlib
  GIT_TAG v1.3.1
  FIND_PACKAGE_ARGS NAMES ZLIB
)
FetchContent_MakeAvailable(zlib)

# zlib has some problem about FetchContent_Declare
# https://github.com/madler/zlib/issues/759
if(NOT TARGET ZLIB::ZLIB)
    add_library(ZLIB::ZLIB STATIC IMPORTED GLOBAL)
    if (MSVC)
        set_property(TARGET ZLIB::ZLIB PROPERTY IMPORTED_LOCATION ${zlib_BINARY_DIR}/z.lib)
    elseif (APPLE)
        set_property(TARGET ZLIB::ZLIB PROPERTY IMPORTED_LOCATION ${zlib_BINARY_DIR}/libz.a)
    else()
        set_property(TARGET ZLIB::ZLIB PROPERTY IMPORTED_LOCATION ${zlib_BINARY_DIR}/libz.a)
    endif()
    set_property(TARGET ZLIB::ZLIB PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${zlib_SOURCE_DIR} ${zlib_BINARY_DIR})
    add_dependencies(ZLIB::ZLIB zlibstatic)
    if (MSVC)
        set(ZLIB_LIBRARY ${zlib_BINARY_DIR}/z.lib CACHE INTERNAL "")
    elseif (APPLE)
        set(ZLIB_LIBRARY ${zlib_BINARY_DIR}/libz.a CACHE INTERNAL "")
    else()
        set(ZLIB_LIBRARY ${zlib_BINARY_DIR}/libz.a CACHE INTERNAL "")
    endif()
endif()
