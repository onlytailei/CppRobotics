/*************************************************************************
	> File Name: main.cpp
	> Author: TAI Lei
	> Mail: lei.tai@my.cityu.edu.hk
	> Created Time: Thu Mar  7 19:39:14 2019
 ************************************************************************/

#include <iostream>
#include <vector>
#include <random>
#include <math.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define SIM_TIME 50.0
#define DT 0.1
#define PI 3.141592653
#define MAX_RANGE 20.0
#define NP 100
#define NTh NP/2

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

// TODO gaussian likelihood
// float gauss_likelihood(x, sigma):
//     p = 1.0 / math.sqrt(2.0 * math.pi * sigma ** 2) * \
//         math.exp(-x ** 2 / (2 * sigma ** 2))
//
//     return p

void pf_localization(
  Eigen::Matrix<float, 4, NP>& px, Eigen::Vector<float, NP>& pw,
  Eigen::Vector4f& xEst, Eigen::Matrix4f& PEst,
  std::vector<Eigen::RowVector3f> z, Eigen::Vector2f u){

    for(int ip=0; ip<NP; i++){
      Eigen::Vector4f x = px.col(ip);
      w = pw(ip);

      Eigen::Vector2f ud;

      ud(0) = u(0) + gaussian_d(gen) * Qsim(0,0);
      ud(1) = u(1) + gaussian_d(gen) * Qsim(1,1);

      x = motion_model(x, ud);

      for(int i=0; i<z.size(); i++){
          float dx = x(0) - z[i](1);
          float dy = x(1) - z[i](2);
          float prez = std::sqrt(dx*dx + dy*dy);
          float dz = prez - z[i](0);
          // w = w * gauss_likelihood(dz, math.sqrt(Q[0, 0]))
      }
      // px[:, ip] = x[:, 0]
      // pw[0, ip] = w
    }

    // Eigen::Vector4f xPred = motion_model(xEst, u);
    // Eigen::Matrix4f jF = jacobF(xPred, u);
    // Eigen::Matrix4f PPred = jF * PEst * jF.transpose() + Q;
    //
    // Eigen::Matrix<float, 2, 4> jH = jacobH(xPred);
    // Eigen::Vector2f zPred = observation_model(xPred);
    // Eigen::Vector2f y = z - zPred;
    // Eigen::Matrix2f S = jH * PPred * jH.transpose() + R;
    // Eigen::Matrix<float, 4, 2> K = PPred * jH.transpose() * S.inverse();
    // xEst = xPred + K * y;
    // PEst = (Eigen::Matrix4f::Identity() - K * jH) * PPred;
};

cv::Point2i cv_offset(
    Eigen::Vector2f e_p, int image_width=2000, int image_height=2000){
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
  std::vector<Eigen::RowVector3f> z;

  // RFID remarks
  Eigen::Matrix<float, 4, 2> RFID;
  RFID<<10.0, 0.0,
        10.0, 10.0,
        0.0,  15.0,
        -5.0, 20.0;

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
  Eigen::Matrix1f Q = Eigen::Matrix1f::Identity();
  Q(0,0)=0.1 * 0.1;

  // Observation model covariance
  Eigen::Matrix2f  R = Eigen::Matrix2f::Identity();
  R(0,0)=1.0;
  R(1,1)=40.0/180.0 * PI * 40.0/180.0 * PI;

  // Motion model simulation error
  Eigen::Matrix1f Qsim = Eigen::Matrix1f::Identity();
  Qsim(0,0)=0.2*0.2;

  // Observation model simulation error
  Eigen::Matrix2f Rsim = Eigen::Matrix2f::Identity();
  Rsim(0,0)=1.0 * 1.0;
  Rsim(1,1)=30.0/180.0 * PI * 30.0/180.0 * PI;

  // particle stor
  Eigen::Matrix<float, 4, NP> px = Eigen::Matrix<float, 4, NP>::Zero();

  Eigen::Vector<float, NP> pw = Eigen::Vector<float, NP>::Zero();

  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> gaussian_d{0,1};

  //for visualization
  cv::namedWindow("ekf", cv::WINDOW_NORMAL);
  cv::Mat bg(3500,3500, CV_8UC3, cv::Scalar(255,255,255));
  int count = 0;

  while(time <= SIM_TIME){
    time += DT;

    ud(0) = u(0) + gaussian_d(gen) * Rsim(0,0);
    ud(1) = u(1) + gaussian_d(gen) * Rsim(1,1);

    xTrue = motion_model(xTrue, u);
    xDR = motion_model(xDR, ud);

    z.clear();
    for(int i=0; i<NP; i++){
      float dx = xTrue(0) - RFID(i, 0);
      float dy = xTrue(1) - RFID(i, 1);
      float d = std::sqrt(dx*dx + dy*dy);
      if (d <= MAX_RANGE){
        float dn = d + gaussian_d(gen) * Qsim(0);
        Eigen::RowVector3f zi;
        zi<<dn, RFID(i, 0), RFID(i, 1);
        z.push_back(zi);
      }
    }

    // TODO pf localization

    // blue estimation
    cv::circle(bg, cv_offset(xEst.head(2), bg.cols, bg.rows),
               10, cv::Scalar(255,0,0), -1);

    // green groundtruth
    cv::circle(bg, cv_offset(xTrue.head(2), bg.cols, bg.rows),
               10, cv::Scalar(0,255,0), -1);

    // black dead reckoning
    cv::circle(bg, cv_offset(xDR.head(2), bg.cols, bg.rows),
               10, cv::Scalar(0, 0, 0), -1);

    // red observation
    cv::circle(bg, cv_offset(z, bg.cols, bg.rows),
               10, cv::Scalar(0, 0, 255), -1);

    cv::imshow("ekf", bg);
    cv::waitKey(5);

    //std::string int_count = std::to_string(count);
    //cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);
    count++;
  }
}
