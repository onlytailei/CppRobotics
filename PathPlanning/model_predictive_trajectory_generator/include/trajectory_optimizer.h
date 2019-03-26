/*************************************************************************
	> File Name: motion_model.h
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Thu Mar 21 16:06:07 2019
 ************************************************************************/

#ifndef _TRAJECTORY_OPTIMIZER_H
#define _TRAJECTORY_OPTIMIZER_H

#include <iostream>
#include <cmath>
#include <cfenv>
#include <Eigen/Eigen>
#include "motion_model.h"

class TrajectoryOptimizer{
  public:
    MotionModel m_model;
    Parameter p;
    TrajState target;

    Traj optimizer_traj(int, float, std::vector<float>);
    TrajectoryOptimizer(MotionModel m_model_,
                        Parameter init_p_,
                        TrajState target_):
                        m_model(m_model_),
                        p(init_p_),
                        target(target_){};

  private:
    float selection_learning_param(Eigen::Vector3f dp);
    TrajState calc_diff(TrajState );
    Eigen::Matrix3f calc_J(std::vector<float>);
    Eigen::Vector3f error_vector(Parameter test_p);
};

Traj TrajectoryOptimizer::optimizer_traj(
      int max_iter, float cost_th, std::vector<float> h_step){

  Traj sample_traj;
  for(int i=0; i<max_iter; i++){
    sample_traj = m_model.generate_trajectory(p);
    TrajState dc = calc_diff(sample_traj.back());
    Eigen::Vector3f dc_vct;
    dc_vct<< dc.x , dc.y, dc.yaw;
    float cost = std::sqrt(std::pow(dc.x, 2) \
        +std::pow(dc.y, 2)+std::pow(dc.yaw, 2));
    if (cost < cost_th){
      std::cout << "find the traj under the cost threshold" << std::endl;
      return sample_traj;
    }

    Eigen::Matrix3f J = calc_J(h_step);
    Eigen::Vector3f dp = - J.inverse() * dc_vct;


    float alpha = selection_learning_param(dp);

    p.distance += alpha * dp(0);
    p.steering_sequence[1] += alpha * dp(1);
    p.steering_sequence[2] += alpha * dp(2);

  }
  return sample_traj;
};

TrajState TrajectoryOptimizer::calc_diff(TrajState traj_state_){
    float yaw_ = target.yaw - traj_state_.yaw;
    return {target.x - traj_state_.x,
            target.y - traj_state_.y,
            YAW_P2P(yaw_)};
};

Eigen::Vector3f TrajectoryOptimizer::error_vector(Parameter test_p){
    TrajState last_state = m_model.generate_last_state(test_p);
    TrajState d_error = calc_diff(last_state);
    Eigen::Vector3f d_vct;
    d_vct<< d_error.x , d_error.y, d_error.yaw;
    return d_vct;

};

Eigen::Matrix3f TrajectoryOptimizer::calc_J(std::vector<float> h){

    Parameter p00 = p;
    p00.distance = p.distance + h[0];
    Eigen::Vector3f d00_vct = error_vector(p00);
    Parameter p01 = p;
    p01.distance = p.distance - h[0];
    Eigen::Vector3f d01_vct = error_vector(p01);
    Eigen::Vector3f d0 = (d00_vct - d01_vct) / (2.0 * h[0]);

    Parameter p10 = p;
    p10.steering_sequence[1] = p.steering_sequence[1] + h[1];
    Eigen::Vector3f d10_vct = error_vector(p10);
    Parameter p11 = p;
    p11.steering_sequence[1] = p.steering_sequence[1] - h[1];
    Eigen::Vector3f d11_vct = error_vector(p11);
    Eigen::Vector3f d1 = (d10_vct - d11_vct) / (2.0 * h[1]);

    Parameter p20 = p;
    p20.steering_sequence[2] = p.steering_sequence[2] + h[2];
    Eigen::Vector3f d20_vct = error_vector(p20);
    Parameter p21 = p;
    p21.steering_sequence[2] = p.steering_sequence[2] - h[2];
    Eigen::Vector3f d21_vct = error_vector(p21);
    Eigen::Vector3f d2 = (d20_vct - d21_vct) / (2.0 * h[2]);

    Eigen::Matrix3f J;
    J<<d0, d1, d2;
    return J;
};


float TrajectoryOptimizer::selection_learning_param(Eigen::Vector3f dp){
    float mincost = std::numeric_limits<float>::max();
    float mina = 1.0;
    float maxa = 2.0;
    float da = 0.5;

    for(float a=mina; a<maxa; a+=da){
        Parameter new_p = p;
        new_p.distance += a * dp[0];
        new_p.steering_sequence[1] += a * dp[1];
        new_p.steering_sequence[2] += a * dp[2];
        TrajState laststate = m_model.generate_last_state(new_p);
        TrajState dc = calc_diff(laststate);
        float cost = std::sqrt(std::pow(dc.x, 2)+std::pow(dc.y, 2)+std::pow(dc.yaw, 2));

        if ((cost <= mincost) && (a != 0.0)){
            mina = a;
            mincost = cost;
        }
    }

    return mina;
};


#endif
