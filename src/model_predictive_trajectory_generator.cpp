/*************************************************************************
	> File Name: main.cpp
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Thu Mar  7 19:39:14 2019
 ************************************************************************/

#include<iostream>
#include<vector>
#include<array>
#include<cmath>
#include<Eigen/Eigen>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include"motion_model.h"
#include"trajectory_optimizer.h"

#define L 1.0
#define DS 0.1
#define CONST_V 3.0  // use a const linear velocity here

using namespace cpprobotics;

int main(){
  State init_state(0, 0, 0, CONST_V);
  TrajState target_(5, 2.0, 0);
  MotionModel m_model(L, DS, init_state);

  Parameter p_(6, {{0,0,0}});
  float cost_th_ = 0.1;
  std::vector<float> h_step_{0.2, 0.005, 0.005};
  int max_iter = 100;

  TrajectoryOptimizer traj_opti_obj(m_model, p_, target_);
  Traj traj = traj_opti_obj.optimizer_traj(max_iter, cost_th_, h_step_, true, true);

};
