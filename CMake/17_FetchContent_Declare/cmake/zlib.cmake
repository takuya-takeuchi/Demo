# build and source will be output into _deps/zlib-build and _deps/zlib-src
FetchContent_Declare(
  zlib
  GIT_REPOSITORY https://github.com/madler/zlib
  GIT_TAG v1.3.1
  CMAKE_ARGS -D ZLIB_INSTALL=OFF
             -D SKIP_INSTALL_ALL=OFF
             -D ZLIB_BUILD_STATIC=ON
             -D ZLIB_BUILD_SHARED=OFF
             -D BUILD_TESTING=OFF
             -D ZLIB_BUILD_MINIZIP=OFF
)
FetchContent_MakeAvailable(zlib)