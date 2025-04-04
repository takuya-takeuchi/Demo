cmake_minimum_required(VERSION 3.18) 

set(PROJ_NAME Demo)
set(CMAKE_CXX_STANDARD 17)

project(${PROJ_NAME} LANGUAGES CXX CUDA)

find_package(CUDAToolkit REQUIRED)

add_executable(${PROJ_NAME} main.cu)

# it is equivalent to -gencode=arch=compute_75,code=sm_75 if specify 75
set_target_properties(${PROJ_NAME} PROPERTIES CUDA_ARCHITECTURES "75")

install(TARGETS ${PROJ_NAME} RUNTIME DESTINATION bin)