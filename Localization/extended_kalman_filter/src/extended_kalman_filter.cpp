/*************************************************************************
	> File Name: main.cpp
	> Author: TAI Lei
	> Mail: lei.tai@my.cityu.edu.hk
	> Created Time: Thu Mar  7 19:39:14 2019
 ************************************************************************/

#include<iostream>
#include<random>
#include<math.h>
#include<Eigen/Eigen>
#include<opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define SIM_TIME 50.0
#define DT 0.1
#define PI 3.141592653

using namespace std;

// x_{t+1} = F@x_{t}+B@u_t
Eigen::Vector4f motion_model(Eigen::Vector4f x, Eigen::Vector2f u){
  Eigen::Matrix4f F_;
  F_<<1.0,   0,   0,   0,
        0, 1.0,   0,   0,
        0,   0, 1.0,   0,
        0,   0,   0, 1.0;

  Eigen::Matrix<float, 4, 2> B_;
  B_<< DT * std::cos(x(2,0)),  0,
       DT * std::sin(x(2,0)),  0,
                        0.0,  DT,
                        1.0,  0.0;

  return F_ * x + B_ * u;
};

Eigen::Matrix4f jacobF(Eigen::Vector4f x, Eigen::Vector2f u){
  Eigen::Matrix4f jF_ = Eigen::Matrix4f::Identity();
  float yaw = x(2);
  float v = u(0);
  jF_(0,2) = -DT * v * std::sin(yaw);
  jF_(0,3) = DT * std::cos(yaw);
  jF_(1,2) = DT * v * std::cos(yaw);
  jF_(1,3) = DT * std::sin(yaw);
  return jF_;
};

//observation mode H
Eigen::Vector2f observation_model(Eigen::Vector4f x){
  Eigen::Matrix<float, 2, 4> H_;
  H_<< 1, 0, 0, 0,
       0, 1, 0, 0;
  return H_ * x;
};

Eigen::Matrix<float, 2, 4> jacobH(Eigen::Vector4f x){
  Eigen::Matrix<float, 2, 4> jH_;
  jH_<< 1, 0, 0, 0,
        0, 1, 0, 0;
  return jH_;
};

void ekf_estimation(Eigen::Vector4f& xEst, Eigen::Matrix4f& PEst,
  Eigen::Vector2f z, Eigen::Vector2f u,
  Eigen::Matrix4f Q, Eigen::Matrix2f R){
    Eigen::Vector4f xPred = motion_model(xEst, u);
    Eigen::Matrix4f jF = jacobF(xPred, u);
    Eigen::Matrix4f PPred = jF * PEst * jF.transpose() + Q;

    Eigen::Matrix<float, 2, 4> jH = jacobH(xPred);
    Eigen::Vector2f zPred = observation_model(xPred);
    Eigen::Vector2f y = z - zPred;
    Eigen::Matrix2f S = jH * PPred * jH.transpose() + R;
    Eigen::Matrix<float, 4, 2> K = PPred * jH.transpose() * S.inverse();
    xEst = xPred + K * y;
    PEst = (Eigen::Matrix4f::Identity() - K * jH) * PPred;
};

cv::Point2i cv_offset(Eigen::Vector2f e_p, int image_width=2000, int image_height=2000){
  cv::Point2i output;
  output.x = int(e_p(0) * 100) + image_width/2;
  output.y = image_height - int(e_p(1) * 100) - image_height/3;
  return output;
};

int main(){
  float time=0.0;

  // control input
  Eigen::Vector2f u;
  u<<1.0, 0.1;

  // nosie control input
  Eigen::Vector2f ud;

  // observation z
  Eigen::Vector2f z;

  // dead reckoning
  Eigen::Vector4f xDR;
  xDR<<0.0,0.0,0.0,0.0;

  // ground truth reading
  Eigen::Vector4f xTrue;
  xTrue<<0.0,0.0,0.0,0.0;

  // Estimation
  Eigen::Vector4f xEst;
  xEst<<0.0,0.0,0.0,0.0;

  Eigen::Matrix4f PEst = Eigen::Matrix4f::Identity();

  // Motional model covariance
  Eigen::Matrix4f Q = Eigen::Matrix4f::Identity();
  Q(0,0)=0.1 * 0.1;
  Q(1,1)=0.1 * 0.1;
  Q(2,2)=(1.0/180 * M_PI) * (1.0/180 * M_PI);
  Q(3,3)=0.1 * 0.1;

  // Observation model covariance
  Eigen::Matrix2f  R = Eigen::Matrix2f::Identity();
  R(0,0)=1.0;
  R(1,1)=1.0;

  // Motion model simulation error
  Eigen::Matrix2f Qsim = Eigen::Matrix2f::Identity();
  Qsim(0,0)=1.0;
  Qsim(1,1)=(30.0/180 * M_PI) * (30.0/180 * M_PI);

  // Observation model simulation error
  Eigen::Matrix2f Rsim = Eigen::Matrix2f::Identity();
  Rsim(0,0)=0.5 * 0.5;
  Rsim(1,1)=0.5 * 0.5;

  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> gaussian_d{0,1};

  //for visualization
  cv::namedWindow("ekf", cv::WINDOW_NORMAL);
  cv::Mat bg(3500,3500, CV_8UC3, cv::Scalar(255,255,255));
  int count = 0;

  while(time <= SIM_TIME){
    time += DT;
    ud(0) = u(0) + gaussian_d(gen) * Qsim(0,0);
    ud(1) = u(1) + gaussian_d(gen) * Qsim(1,1);

    xTrue = motion_model(xTrue, u);
    xDR = motion_model(xDR, ud);

    z(0) = xTrue(0) + gaussian_d(gen) * Rsim(0,0);
    z(1) = xTrue(1) + gaussian_d(gen) * Rsim(1,1);

    ekf_estimation(xEst, PEst, z, ud, Q, R);

    // blue estimation
    cv::circle(bg, cv_offset(xEst.head(2), bg.cols, bg.rows), 10, cv::Scalar(255,0,0), -1);
    // green groundtruth
    cv::circle(bg, cv_offset(xTrue.head(2), bg.cols, bg.rows), 10, cv::Scalar(0,255,0), -1);
    // black dead reckoning
    cv::circle(bg, cv_offset(xDR.head(2), bg.cols, bg.rows), 10, cv::Scalar(0, 0, 0), -1);
    // red observation
    cv::circle(bg, cv_offset(z, bg.cols, bg.rows), 10, cv::Scalar(0, 0, 255), -1);
    cv::imshow("ekf", bg);
    cv::waitKey(5);

    cv::imwrite("./pngs/"+std::to_string(count)+".png", bg);
    count++;
  }
}
