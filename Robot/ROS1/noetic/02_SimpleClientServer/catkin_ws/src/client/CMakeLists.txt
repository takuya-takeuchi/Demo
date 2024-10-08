cmake_minimum_required(VERSION 3.0.2)

set(PROJ_NAME client)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJ_NAME} VERSION 1.0.0)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sample_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sample_msgs
)

add_executable(${PROJ_NAME} src/main.cpp)

target_include_directories(${PROJ_NAME}
                           PRIVATE
                           ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJ_NAME}
                      PRIVATE
                      ${catkin_LIBRARIES}
)