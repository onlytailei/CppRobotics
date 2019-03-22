cmake_minimum_required(VERSION 3.1)
project(model_predictive_trajectory_generator)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
set( BOOST_ROOT "/home/tai/Software/boost_1_65_0/stage/lib" CACHE PATH "Boost library path" )
set( Boost_NO_SYSTEM_PATHS on CACHE BOOL "Do not search system for Boost" )
include_directories(${EIGEN3_INCLUDE_DIR})
# find_package(Boost 1.65 REQUIRED math)
# include_directories(${Boost_INCLUDE_DIR})
include_directories(./include)
include_directories(/home/tai/Software/boost_1_65_0)

add_executable(mptg src/model_predictive_trajectory_generator.cpp)
target_link_libraries(mptg ${OpenCV_LIBS} boost_math_tr1f)
