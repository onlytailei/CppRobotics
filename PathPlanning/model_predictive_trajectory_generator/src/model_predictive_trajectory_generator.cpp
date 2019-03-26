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
#include "trajectory_optimizer.h"

#define L 1.0
#define DS 0.1
#define CONST_V 3.0  // use a const linear velocity here


cv::Point2i cv_offset(
    float x, float y, int image_width=2000, int image_height=2000){
  cv::Point2i output;
  output.x = int(x * 100) + image_width/4;
  output.y = image_height - int(y * 100) - image_height/3;
  return output;
};


int main(){
  State init_state(0, 0, 0, CONST_V);
  TrajState target_(5, 2.0, 0);
  MotionModel m_model(L, DS, init_state);

  Parameter p_(6, {{0,0,0}});
  float cost_th_ = 0.05;
  std::vector<float> h_step_{0.2, 0.005, 0.005};
  int max_iter = 100;

  TrajectoryOptimizer traj_opti_obj(m_model, p_, target_);
  Traj traj = traj_opti_obj.optimizer_traj(max_iter, cost_th_, h_step_);


  cv::namedWindow("mptg", cv::WINDOW_NORMAL);
  int count = 0;
  cv::Mat bg(500, 1000, CV_8UC3, cv::Scalar(255, 255, 255));
  for(unsigned int i=1; i<traj.size(); i++){
    cv::line(
      bg,
      cv_offset(traj[i-1].x, traj[i-1].y, bg.cols, bg.rows),
      cv_offset(traj[i].x, traj[i].y, bg.cols, bg.rows),
      cv::Scalar(0, 0, 0),
      5);
  }

  cv::circle(bg, cv_offset(target_.x, target_.y, bg.cols, bg.rows),
             20, cv::Scalar(255,0,0), 5);
  cv::circle(bg, cv_offset(init_state.x, init_state.y, bg.cols, bg.rows),
             20, cv::Scalar(0,0,255), 5);

  cv::imshow("mptg", bg);
  cv::waitKey(5);

  std::string int_count = std::to_string(count);
  cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);
};
