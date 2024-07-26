cmake_minimum_required(VERSION 3.22.1)

project(Hello LANGUAGES C CXX)

add_executable(hello main.cpp)

target_include_directories(hello PRIVATE ${Native_DIR}/include)
target_link_libraries(hello PRIVATE ${Native_DIR}/lib/libwrapper.so)

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)