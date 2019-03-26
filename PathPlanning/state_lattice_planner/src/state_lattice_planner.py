/*************************************************************************
	> File Name: main.cpp
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Thu Mar  7 19:39:14 2019
 ************************************************************************/

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Eigen>
#include "motion_model.h"

#define L 1.0
#define DS 0.1
#define CONST_V 3.0  // use a const linear velocity here


TrajState calc_diff(TrajState target_, TrajState traj_state_){
    float yaw_ = target_.yaw - traj_state_.yaw;
    return {target_.x - traj_state_.x,
            target_.y - traj_state_.y,
            YAW_P2P(yaw_)};
};

Eigen::Vector3f error_vector(MotionModel m_model_, Parameter p, TrajState target_){
    TrajState last_state = m_model_.generate_last_state(p);
    TrajState d_error = calc_diff(target_, last_state);
    Eigen::Vector3f d_vct;
    d_vct<< d_error.x , d_error.y, d_error.yaw;
    return d_vct;

};

Eigen::Matrix3f calc_J(MotionModel m_model_,
                       TrajState target_,
                       Parameter p,
                       std::vector<float> h){

    Parameter p00 = p;
    p00.distance = p.distance + h[0];
    Eigen::Vector3f d00_vct = error_vector(m_model_, p00, target_);
    Parameter p01 = p;
    p01.distance = p.distance - h[0];
    Eigen::Vector3f d01_vct = error_vector(m_model_, p01, target_);
    Eigen::Vector3f d0 = (d00_vct - d01_vct) / (2.0 * h[0]);

    Parameter p10 = p;
    p10.steering_sequence[1] = p.steering_sequence[1] + h[1];
    Eigen::Vector3f d10_vct = error_vector(m_model_, p10, target_);
    Parameter p11 = p;
    p11.steering_sequence[1] = p.steering_sequence[1] - h[1];
    Eigen::Vector3f d11_vct = error_vector(m_model_, p11, target_);
    Eigen::Vector3f d1 = (d10_vct - d11_vct) / (2.0 * h[1]);

    Parameter p20 = p;
    p20.steering_sequence[2] = p.steering_sequence[2] + h[2];
    Eigen::Vector3f d20_vct = error_vector(m_model_, p20, target_);
    Parameter p21 = p;
    p21.steering_sequence[2] = p.steering_sequence[2] - h[2];
    Eigen::Vector3f d21_vct = error_vector(m_model_, p21, target_);
    Eigen::Vector3f d2 = (d20_vct - d21_vct) / (2.0 * h[2]);

    Eigen::Matrix3f J;
    J<<d0, d1, d2;
    return J;

};


float selection_learning_param(
    MotionModel motion_model, Eigen::Vector3f dp, Parameter p, TrajState target){
    float mincost = std::numeric_limits<float>::max();
    float mina = 1.0;
    float maxa = 2.0;
    float da = 0.5;

    for(float a=mina; a<maxa; a+=da){
        Parameter new_p = p;
        new_p.distance += a * dp[0];
        new_p.steering_sequence[1] += a * dp[1];
        new_p.steering_sequence[2] += a * dp[2];
        TrajState laststate = motion_model.generate_last_state(new_p);
        TrajState dc = calc_diff(target, laststate);
        float cost = std::sqrt(std::pow(dc.x, 2)+std::pow(dc.y, 2)+std::pow(dc.yaw, 2));

        if ((cost <= mincost) && (a != 0.0)){
            mina = a;
            mincost = cost;
        }
    }

    return mina;
};


cv::Point2i cv_offset(
    float x, float y, int image_width=2000, int image_height=2000){
  cv::Point2i output;
  output.x = int(x * 100) + image_width/4;
  output.y = image_height - int(y * 100) - image_height/3;
  return output;
};


int main(){
  State init_state(0, 0, 0, CONST_V);
  TrajState target_state(5, 2.0, 0);
  MotionModel m_model(L, DS, init_state);

  Parameter p_(6, {{0,0,0}});
  float cost_th_ = 0.05;
  std::vector<float> h_step_{0.2, 0.005, 0.005};
  int max_iter = 100;

  cv::namedWindow("mptg", cv::WINDOW_NORMAL);
  int count = 0;

  for(int i=0; i<max_iter; i++){
    Traj sample_traj = m_model.generate_trajectory(p_);
    TrajState dc = calc_diff(target_state, sample_traj.back());
    Eigen::Vector3f dc_vct;
    dc_vct<< dc.x , dc.y, dc.yaw;
    float cost = std::sqrt(std::pow(dc.x, 2)+std::pow(dc.y, 2)+std::pow(dc.yaw, 2));
    if (cost < cost_th_){
      std::cout << "find the traj under the cost threshold" << std::endl;
      break;
    }

    Eigen::Matrix3f J = calc_J(m_model, target_state, p_, h_step_);
    Eigen::Vector3f dp = - J.inverse() * dc_vct;


    float alpha = selection_learning_param(m_model, dp, p_, target_state);

    p_.distance += alpha * dp(0);
    p_.steering_sequence[1] += alpha * dp(1);
    p_.steering_sequence[2] += alpha * dp(2);

    // visualization
    cv::Mat bg(500, 1000, CV_8UC3, cv::Scalar(255, 255, 255));
    for(int i=1; i<sample_traj.size(); i++){
      cv::line(
        bg,
        cv_offset(sample_traj[i-1].x, sample_traj[i-1].y, bg.cols, bg.rows),
        cv_offset(sample_traj[i].x, sample_traj[i].y, bg.cols, bg.rows),
        cv::Scalar(0, 0, 0),
        5);
    }

    cv::circle(bg, cv_offset(target_state.x, target_state.y, bg.cols, bg.rows),
               20, cv::Scalar(255,0,0), 5);
    cv::circle(bg, cv_offset(init_state.x, init_state.y, bg.cols, bg.rows),
               20, cv::Scalar(0,0,255), 5);

    cv::imshow("mptg", bg);
    cv::waitKey(5);

    // std::string int_count = std::to_string(count);
    // cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);

    count++;

  }
};
