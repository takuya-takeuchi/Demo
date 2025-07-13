# 1. build and source will be output into _deps/zlib-build (zlib_BINARY_DIR) and _deps/zlib-src (zlib_SOURCE_DIR)
# 2. CMAKE_ARGS is ignored. Refer: https://discourse.cmake.org/t/fetchcontent-declare-with-cmake-args-does-not-work/11227/7
FetchContent_Declare(
  zlib
  GIT_REPOSITORY https://github.com/madler/zlib
  GIT_TAG v1.3.1
)
FetchContent_MakeAvailable(zlib)

# zlib has some problem about FetchContent_Declare
# https://github.com/madler/zlib/issues/759
if(NOT TARGET ZLIB::ZLIB)
    if (MSVC)
        add_library(ZLIB::ZLIB STATIC IMPORTED GLOBAL)
        set_property(TARGET ZLIB::ZLIB PROPERTY IMPORTED_LOCATION ${zlib_BINARY_DIR}/z.lib)
        set_property(TARGET ZLIB::ZLIB PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${zlib_SOURCE_DIR} ${zlib_BINARY_DIR})
        add_dependencies(ZLIB::ZLIB zlibstatic)
        set(ZLIB_LIBRARY ${zlib_BINARY_DIR}/z.lib CACHE INTERNAL "")
    elseif (APPLE)
        add_library(ZLIB::ZLIB STATIC IMPORTED GLOBAL)
        set_property(TARGET ZLIB::ZLIB PROPERTY IMPORTED_LOCATION ${zlib_BINARY_DIR}/libz.a)
        set_property(TARGET ZLIB::ZLIB PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${zlib_SOURCE_DIR} ${zlib_BINARY_DIR})
        add_dependencies(ZLIB::ZLIB zlibstatic)
        set(ZLIB_LIBRARY ${zlib_BINARY_DIR}/libz.a CACHE INTERNAL "")
    else()
        add_library(ZLIB::ZLIB STATIC IMPORTED GLOBAL)
        set_property(TARGET ZLIB::ZLIB PROPERTY IMPORTED_LOCATION ${zlib_BINARY_DIR}/libz.a)
        set_property(TARGET ZLIB::ZLIB PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${zlib_SOURCE_DIR} ${zlib_BINARY_DIR})
        add_dependencies(ZLIB::ZLIB zlibstatic)
        set(ZLIB_LIBRARY ${zlib_BINARY_DIR}/libz.a CACHE INTERNAL "")
    endif()
endif()
