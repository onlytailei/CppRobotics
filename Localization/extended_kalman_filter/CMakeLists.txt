cmake_minimum_required(VERSION 3.1)
project(extended_kalman_filter)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(ekf src/extended_kalman_filter.cpp)
target_link_libraries(ekf ${OpenCV_LIBS} )
