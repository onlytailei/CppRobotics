# CppRobotics

This is the cpp implementation of the [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)

## Requirment
- cmake
- opencv 3.3
- Eigen 3
- ~~ROS~~ (I don't want to make the project tooooooo huge :))

## Build
     $ mkdir build
     $ cd build
     $ cmake ../
     $ make -j 8

Find all the executable files in ***build/bin***.

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
    * [Cubic Spline Planner](#cubic-spline-planner)
    * [State Lattice Planner](#state-lattice-planner)

# Localization
## Extended Kalman Filter Localization
* green line: the groundtruth trajectory
* black line: dead reckoning
* red points: observations (e.g. GPS)
* blue line: estimated positions

<!-- ![ekf_gif](./gif/ekf.gif) -->
<img src="./gif/ekf.gif" alt="ekf" width="400"/>

## Particle Filter Localization
* green line: the groundtruth trajectory
* black line: dead reckoning
* red points: landmarks
* blue line: estimated positions

<!-- ![pf_gif](./gif/pf.gif) -->
<img src="./gif/pf.gif" alt="pf" width="400"/>

# Path Planning
## Dynamic Window Approach
* blue circle: the target point
* red circle: the robot

<!-- ![dwa_gif](./gif/dwa.gif) -->
<img src="./gif/dwa.gif" alt="dwa" width="400"/>

## Model Predictive Trajectory Generator
This part is based on the bicycle motion model.
* blue circle: the target point
* red circle: the initial point

<!-- ![mptg_gif](./gif/mptg.gif) -->
<img src="./gif/mptg.gif" alt="mptg" width="400"/>

## Cubic Spline Planner

<!-- ![mptg_gif](./gif/csp.png =500x) -->
<img src="./gif/csp.png" alt="csp" width="400"/>

## State Lattice Planner
* blue circle: the target point
* red circle: the initial point

<!-- ![mptg_gif](./gif/slp.gif) -->
<img src="./gif/slp.gif" alt="slp" width="400"/>

[State Space Sampling of Feasible Motions for High-Performance Mobile Robot Navigation in Complex Environments](https://www.ri.cmu.edu/pub_files/pub4/howard_thomas_2008_1/howard_thomas_2008_1.pdf)
