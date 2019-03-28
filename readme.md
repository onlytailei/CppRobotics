# CppRobotics

This is the cpp implementation of the [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)

## Requirment
- cmake
- opencv 3.3
- Eigen 3

## Build
     $ export CPP_ROBOTICS=/path/to/root_of_this_repo
     $ cd /path/to/specific/method
     $ mkdir build
     $ cd build
     $ cmake ../
     $ make -j 8


# Table of Contents
* [Localization](#localization)
    * [Extended kalmam filter](#extended-kalman-filter-localization)
    * [Particle filter](#particle-filter-localization)
    * Histogram filter
* [Mapping](#mapping)
    * Gaussian grid map
* [SLAM](#SLAM)
    * FastSLAM 1.0
* [Path Planning](#path-planning)
    * [Dynamic Window Approach](#dynamic-window-approach)
    * [Model Predictive Trajectory Generator](#model-predictive-trajectory-generator)
    * [State Lattice Planner](#state-lattice-planner)

# Localization
## Extended Kalman Filter Localization
* green line: the groundtruth trajectory
* black line: dead reckoning
* red points: observations (e.g. GPS)
* blue line: estimated positions

![ekf_gif](./Localization/extended_kalman_filter/ekf.gif)

## Particle Filter Localization
* green line: the groundtruth trajectory
* black line: dead reckoning
* red points: landmarks
* blue line: estimated positions

![pf_gif](./Localization/particle_filter/pf.gif)

# Path Planning
## Dynamic Window Approach
* blue circle: the target point
* red circle: the robot

![dwa_gif](./PathPlanning/dynamic_window_approach/dwa.gif)

## Model Predictive Trajectory Generator
This part is based on the bicycle motion model.
* blue circle: the target point
* red circle: the initial point

![mptg_gif](./PathPlanning/model_predictive_trajectory_generator/mptg.gif)

## State Lattice Planner
* blue circle: the target point
* red circle: the initial point

![mptg_gif](./PathPlanning/state_lattice_planner/slp.gif)
