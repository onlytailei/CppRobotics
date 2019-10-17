cmake_minimum_required(VERSION 3.1)
project(CPP_ROBOTICS)

set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_BUILD_TYPE Debug)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#############################
#Dependencies
#############################

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(./include)

#include_directories(/home/tai/Software/Ipopt-3.12.13/build/include)
#link_directories(/home/tai/Software/Ipopt-3.12.13/build/lib)



##############################
#Localization
##############################

add_executable(ekf src/extended_kalman_filter.cpp)
target_link_libraries(ekf ${OpenCV_LIBS} )

add_executable(pf src/particle_filter.cpp)
target_link_libraries(pf ${OpenCV_LIBS} )

##############################
#PathPlanning
##############################

add_executable(dijkstra src/dijkstra.cpp)
target_link_libraries(dijkstra ${OpenCV_LIBS} )

add_executable(astar src/a_star.cpp)
target_link_libraries(astar ${OpenCV_LIBS} )

add_executable(rrt src/rrt.cpp)
target_link_libraries(rrt ${OpenCV_LIBS} )

add_executable(rrtstar src/rrt_star.cpp)
target_link_libraries(rrtstar ${OpenCV_LIBS} )

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

add_executable(lqr src/lqr_steer_control.cpp)
target_link_libraries(lqr ${OpenCV_LIBS})

add_executable(lqr_full src/lqr_speed_steer_control.cpp)
target_link_libraries(lqr_full ${OpenCV_LIBS})

#add_executable(mpc src/model_predictive_control.cpp)
#target_link_libraries(mpc ${OpenCV_LIBS} ipopt)
