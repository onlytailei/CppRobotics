cmake_minimum_required(VERSION 3.1)
project(cubic_spline_planner)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
#include_directories($ENV{CPP_ROBOTICS}/PathPlanning/model_predictive_trajectory_generator/include)
include_directories(./include)

add_executable(csp src/cubic_spline_planner.cpp)
target_link_libraries(csp ${OpenCV_LIBS})
