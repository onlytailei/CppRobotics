# CppRobotics

This is the cpp implementation of the [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)

## Requirment
The target is to minimize the requirements in this project
- cmake
- opencv 3.3
- Eigen 3


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
    * [State Lattice Planner](#state-lattice-lanner)

# Localization
## Extended Kalman Filter Localization
![ekf_gif](./Localization/extended_kalman_filter/ekf.gif)

## Particle Filter Localization
![pf_gif](./Localization/particle_filter/pf.gif)

# Path Planning
## Dynamic Window Approach
![dwa_gif](./PathPlanning/dynamic_window_approach/dwa.gif)

## Model Predictive Trajectory Generator
![mptg_gif](./PathPlanning/model_predictive_trajectory_generator/mptg.gif)

## State Lattice Planner
![mptg_gif](./PathPlanning/state_lattice_planner/slp.gif)
