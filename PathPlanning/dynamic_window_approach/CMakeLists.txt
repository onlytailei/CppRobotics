cmake_minimum_required(VERSION 3.1)
project(dynamic_window_approach)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(dwa src/dynamic_window_approach.cpp)
target_link_libraries(dwa ${OpenCV_LIBS} )
