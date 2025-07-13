ExternalProject_Add(
  libjpeg-turbo
  GIT_REPOSITORY https://github.com/libjpeg-turbo/libjpeg-turbo
  GIT_TAG 3.1.1
  CMAKE_ARGS
      -D CMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/external/libjpeg-turbo
      -D BUILD_SHARED_LIBS=OFF
      -D WITH_JPEG8=ON
)

if (MSVC)
  set(LIBJPEG_TURBO_INCLUDE_DIR ${CMAKE_BINARY_DIR}/external/libjpeg-turbo/include)
  set(LIBJPEG_TURBO_LIB_DIR ${CMAKE_BINARY_DIR}/external/libjpeg-turbo/lib)
  set(LIBJPEG_TURBO_LIBRARIES ${LIBJPEG_TURBO_LIB_DIR}/jpeg-static.lib
                              ${LIBJPEG_TURBO_LIB_DIR}/turbojpeg-static.lib)
elseif (APPLE)
  set(LIBJPEG_TURBO_INCLUDE_DIR ${CMAKE_BINARY_DIR}/external/libjpeg-turbo/include)
  set(LIBJPEG_TURBO_LIB_DIR ${CMAKE_BINARY_DIR}/external/libjpeg-turbo/lib)
  set(LIBJPEG_TURBO_LIBRARIES ${LIBJPEG_TURBO_LIB_DIR}/libjpeg.a
                              ${LIBJPEG_TURBO_LIB_DIR}/libturbojpeg.a)
else()
  set(LIBJPEG_TURBO_INCLUDE_DIR ${CMAKE_BINARY_DIR}/external/libjpeg-turbo/include)
  set(LIBJPEG_TURBO_LIB_DIR ${CMAKE_BINARY_DIR}/external/libjpeg-turbo/lib)
  set(LIBJPEG_TURBO_LIBRARIES ${LIBJPEG_TURBO_LIB_DIR}/libjpeg.a
                              ${LIBJPEG_TURBO_LIB_DIR}/libturbojpeg.a)
endif()