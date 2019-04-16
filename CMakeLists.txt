cmake_minimum_required(VERSION 3.1)
project(extended_kalman_filter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(./include)

##############################
#Localiation
##############################
add_executable(ekf src/extended_kalman_filter.cpp)
target_link_libraries(ekf ${OpenCV_LIBS} )

add_executable(pf src/particle_filter.cpp)
target_link_libraries(pf ${OpenCV_LIBS} )

##############################
#PathPlanning
##############################

add_executable(dwa src/dynamic_window_approach.cpp)
target_link_libraries(dwa ${OpenCV_LIBS} )

add_executable(mptg src/model_predictive_trajectory_generator.cpp)
target_link_libraries(mptg ${OpenCV_LIBS})

add_executable(slp src/state_lattice_planner.cpp)
target_link_libraries(slp ${OpenCV_LIBS})

add_executable(csp src/cubic_spline_planner.cpp)
target_link_libraries(csp ${OpenCV_LIBS})

add_executable(frenet src/frenet_optimal_trajectory.cpp)
target_link_libraries(frenet ${OpenCV_LIBS})


##############################
#PathTracking
##############################
