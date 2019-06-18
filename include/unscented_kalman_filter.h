
#ifndef UNSCENTED_KALMAN_FILTER_H
#define UNSCENTED_KALMAN_FILTER_H

#include <iostream>
#include <random>
#include <cmath>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace cpprobotics {

class MerweScaledSigmaPoints {

public:
  MerweScaledSigmaPoints(uint const n, float const alpha, float const beta, float const kappa);

  uint n;
  float alpha;
  float beta;
  float kappa;

  Eigen::ArrayXf Wc; // Weight for each sigma point for the mean
  Eigen::ArrayXf Wm; // Weight for each sigma point for the covariance

  uint numSigmas();
  Eigen::MatrixXf sigmaPoints(Eigen::VectorXf const x, Eigen::MatrixXf const P);

};

class UnscentedKalmanFilter {

public:
  UnscentedKalmanFilter(uint const dim_x, uint const dim_z, float const dt);


  Eigen::VectorXf x;        // state estimate vector// = Eigen::VectorXf::Zero(dim_x);
  Eigen::MatrixXf P;        // covariance estimate matrix// = Eigen::Matrix4f::Identity();

  Eigen::VectorXf x_prior;  //
  Eigen::MatrixXf P_prior;

  Eigen::VectorXf x_post;
  Eigen::MatrixXf P_post;

  Eigen::MatrixXf Q;
  Eigen::MatrixXf R;

  Eigen::MatrixXf K;
  Eigen::VectorXf y;
  Eigen::VectorXf z;
  Eigen::MatrixXf S;  // system uncertainty
  Eigen::MatrixXf SI; // inverse system uncertainty


  void predict();

private:
  uint dim_x;
  uint dim_z;
  float dt;

  //unscentedTransform();
  //computeProcessSigmas(dt);
};
}

#endif // UNSCENTED_KALMAN_FILTER_H
