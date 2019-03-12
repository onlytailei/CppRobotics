cmake_minimum_required(VERSION 3.1)
project(particle_filter)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(pf src/particle_filter.cpp)
target_link_libraries(pf ${OpenCV_LIBS} )
