#include <string>
#include "unscented_kalman_filter.h"


namespace cpprobotics {

MerweScaledSigmaPoints::MerweScaledSigmaPoints(
    uint const n,
    float const alpha,
    float const beta,
    float const kappa)
: n(n),
  alpha(alpha),
  beta(beta),
  kappa(kappa) {
    float lambda = (alpha*alpha) + (n*kappa) - n;
    float c = 0.5 / (n+lambda);
    Wc = Eigen::ArrayXf::Zero(2*n+1) + c;
    Wm = Eigen::ArrayXf::Zero(2*n+1) + c;
    Wc(0) = lambda / (n+lambda) + (1-alpha*alpha+beta);
    Wm(0) = lambda / (n+lambda);
  }

uint MerweScaledSigmaPoints::numSigmas()
{
  return 2*n + 1;
}

Eigen::MatrixXf MerweScaledSigmaPoints::sigmaPoints(
  Eigen::VectorXf const x,
  Eigen::MatrixXf const P)
{
  if (x.size()!=n) {
    throw std::string("Expected size(x): "+std::to_string(n));
  }

  auto lambda = (alpha*alpha) * (n+kappa) - n;

  // Use cholesky decomposition.
  Eigen::LLT<Eigen::MatrixXf>lltofP((lambda+n) * P);

  // Need the upper triangular matrix.
  Eigen::MatrixXf U = lltofP.matrixU();

  Eigen::MatrixXf sigmas = Eigen::MatrixXf::Zero(2*n+1, n);

  sigmas.row(0) = x;
  for (auto i=0; i<n; i++) {
    sigmas.row(i+1) =  x.transpose() + U.row(i);
    sigmas.row(n+i+1) =  x.transpose() - U.row(i);
  }

  return sigmas;
}

UnscentedKalmanFilter::UnscentedKalmanFilter(
    uint const dim_x,
    uint const dim_z,
    float const dt)
: dim_x(dim_x),
  dim_z(dim_z),
  dt(dt)
{

  x = Eigen::VectorXf::Zero(dim_x);
  P = Eigen::MatrixXf::Identity(dim_x,dim_x);

  x_prior = x;
  P_prior = P;

  x_post = x;
  P_post = P;

  Q = Eigen::MatrixXf::Identity(dim_x,dim_x);
  R = Eigen::MatrixXf::Identity(dim_z,dim_z);

  K = Eigen::MatrixXf::Zero(dim_x,dim_z); // Kalman Gain
  y = Eigen::VectorXf::Zero(dim_z);
  z = Eigen::VectorXf::Zero(dim_z);
  S = Eigen::MatrixXf::Zero(dim_z,dim_z);
  SI = Eigen::MatrixXf::Zero(dim_z,dim_z);
}

void UnscentedKalmanFilter::predict()
{
  //computeProcessSigmas(dt);

  //std::tuple<Eigen::VectorXf, Eigen::MatrixXf> prior = unscentedTransform();
  //x_prior = std::get<0>(prior);
  //P_prior = std::get<1>(prior);
}
}
