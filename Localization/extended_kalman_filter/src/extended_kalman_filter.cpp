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

#define SIM_TIME 50.0
#define DT 0.1
#define PI 3.141592653

using namespace std;

class ExtendedKalmanFilter{
  public:
    ExtendedKalmanFilter(Eigen::Matrix4f F, Eigen::Matrix<float, 4,2> B): F_(F), B_(B){};

    // x_{t+1} = F@x_{t}+B@u_t
    Eigen::Vector4f motion_model(Eigen::Vector4f, Eigen::Vector2f);
    Eigen::Vector2f observation(Eigen::Vector4f&, Eigen::Vector2f&, Eigen::Vector2f&);

  private:
    // x_{t+1} = F@x_{t}+B@u_t
    Eigen::Matrix4f F_;
    Eigen::Matrix<float, 4, 2> B_;
    Eigen::Matrix4f Q_;
    Eigen::Matrix4f R_;


    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> gaussian_d{0,1};
};


// x_{t+1} = F@x_{t}+B@u_t
Eigen::Vector4f ExtendedKalmanFilter::motion_model(Eigen::Vector4f x, Eigen::Vector2f u){
  return F_ * x + B_ * u;
};

// add noise to ud
Eigen::Vector2f ExtendedKalmanFilter::observation(Eigen::Vector4f &xTrue, Eigen::Vector2f &xd, Eigen::Vector2f &u){
  xTrue = motion_model(xTrue, xd);

  Eigen::Vector2f z;
  z(0) = xTrue(0) + gaussian_d(gen);
  z(1) = xTrue(1) + gaussian_d(gen);
  return z;
}


int main(){
  int time=0.0;

  // control input
  Eigen::Vector2f u;
  u<<1.0, 0.1;

  // dead reckoning
  Eigen::Vector4f xDR;
  xDR<<0.0,0.0,0.0,0.0;

  // Q, R
  Eigen::Matrix4f Q = Eigen::Matrix4f::Identity();
  Q(0,0)=0.1 * 0.1;
  Q(1,1)=0.1 * 0.1;
  Q(2,2)=(1.0/180 * M_PI) * (1.0/180 * M_PI);
  Q(3,3)=0.1 * 0.1;

  Eigen::Matrix2f R = Eigen::Matrix2f::Identity();
  R(0,0)=1.0;
  R(1,1)=1.0;

  // Qsim Rsim

  while(time <= SIM_TIME){
    time += DT;
  }
}
