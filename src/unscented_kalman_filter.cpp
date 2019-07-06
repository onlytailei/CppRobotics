#include <string>
#include <iomanip>
#include <functional>
#include "unscented_kalman_filter.h"


namespace ukf
{

MerweScaledSigmaPoints::MerweScaledSigmaPoints(
    uint const n,
    double const alpha,
    double const beta,
    double const kappa)
: n(n),
  alpha(alpha),
  beta(beta),
  kappa(kappa)
{

  auto lambda = (alpha*alpha) * (n+kappa) - n;

  auto c = 0.5 / (n+lambda);

  Wc = Eigen::VectorXd::Zero(2*n+1).array() + c;
  Wm = Eigen::VectorXd::Zero(2*n+1).array() + c;
  Wc(0) = lambda / (n+lambda) + (1-(alpha*alpha)+beta);
  Wm(0) = lambda / (n+lambda);
}

uint MerweScaledSigmaPoints::num_sigmas() const
{
  return 2*n + 1;
}

Eigen::MatrixXd MerweScaledSigmaPoints::sigma_points(
  Eigen::VectorXd const& x,
  Eigen::MatrixXd const& P)
{
  if (x.size()!=n) {
    throw std::string("Expected size(x): "+std::to_string(n));
  }

  auto lambda = (alpha*alpha) * (n+kappa) - n;

  // Use cholesky decomposition.
  Eigen::LLT<Eigen::MatrixXd>lltofP((lambda+n) * P);

  // Need the upper triangular matrix.
  Eigen::MatrixXd U = lltofP.matrixU();
  Eigen::MatrixXd sigmas = Eigen::MatrixXd::Zero(2*n+1, n);

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
    double const dt,
    MerweScaledSigmaPoints& points,
    std::function<Eigen::VectorXd(Eigen::VectorXd, Eigen::VectorXd, double)> motion_model,
    std::function<Eigen::VectorXd(Eigen::VectorXd)> observation_model,
    std::function<Eigen::VectorXd(Eigen::VectorXd const&, Eigen::VectorXd const&)> residuals_x,
    std::function<Eigen::VectorXd(Eigen::VectorXd const&, Eigen::VectorXd const&)> residuals_z)
: dim_x_(dim_x),
  dim_z_(dim_z),
  dt_(dt),
  points_(points),
  motion_model_(motion_model),
  observation_model_(observation_model),
  residuals_x_(residuals_x),
  residuals_z_(residuals_z)
{

  Wm = points.Wm;
  Wc = points.Wc;

  x = Eigen::VectorXd::Zero(dim_x_);
  P = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
  x_prior = x;
  P_prior = P;
  x_post = x;
  P_post = P;
  Q = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
  R = Eigen::MatrixXd::Identity(dim_z_, dim_z_);
  K = Eigen::MatrixXd::Zero(dim_x_ , dim_z_);
  y = Eigen::VectorXd::Zero(dim_z_);
  S = Eigen::MatrixXd::Zero(dim_z_, dim_z_);
  SI = Eigen::MatrixXd::Zero(dim_z_, dim_z_);

  predict_called_ = false;
  num_sigmas_ = points.num_sigmas();
}

void UnscentedKalmanFilter::predict(Eigen::VectorXd u)
{
  // The function predict() needs to be called at least once before calling update().
  if (!predict_called_) { predict_called_ = true; };

  sigmas_ = points_.sigma_points(x, P);
  //std::cout << "sigmas" << sigmas_ << std::endl;
  sigmas_f_ = Eigen::MatrixXd::Zero(num_sigmas_, dim_x_);

  // Pass the sigmas through the process model.
  for (auto i=0; i<num_sigmas_; i++) {
    sigmas_f_.row(i) = motion_model_(sigmas_.row(i), u, dt_);
  }
  //std::cout << "sigmas passed" << sigmas_f_ << std::endl;
  std::tuple<Eigen::VectorXd, Eigen::MatrixXd> prior = unscented_transform(sigmas_f_, Wm, Wc, Q, residuals_x_);
  //std::cout << "prior" << std::get<0>(prior) << std::endl;
  x_prior = std::get<0>(prior);
  P_prior = std::get<1>(prior);
}

void UnscentedKalmanFilter::update(Eigen::VectorXd z)
{
  // Make sure that predict called once before update
  if (!predict_called_) {
    throw std::string("The function predict() needs to be called at least once.");
  }

  Eigen::MatrixXd sigmas_h = Eigen::MatrixXd::Zero(num_sigmas_, dim_z_);

  // Pass the sigmas processed by the process(motion) model through the observation model.
  for (auto i=0; i<num_sigmas_; i++) {
    sigmas_h.row(i) = observation_model_(sigmas_f_.row(i));
  }

  std::tuple<Eigen::VectorXd, Eigen::MatrixXd> pair = unscented_transform(sigmas_h, Wm, Wc, R, residuals_z_);
  auto zp = std::get<0>(pair);
  S = std::get<1>(pair);
  SI = S.inverse();

  // Compute the crosse variance of the state and measurements.
  Eigen::MatrixXd Pxz = cross_variance(x, z, sigmas_f_, sigmas_h);

  K = Pxz * SI; // Kalman gain
  y = residuals_z_(z,zp);

  // Update the gaussian state estimate.
  x += K*y;
  P -= K*(S*K.transpose());

  x_post = x;
  P_post = P;
}

Eigen::MatrixXd UnscentedKalmanFilter::cross_variance(
  Eigen::VectorXd x,
  Eigen::VectorXd z,
  Eigen::MatrixXd sigmas_f,
  Eigen::MatrixXd sigmas_h)
{
  Eigen::MatrixXd CV = Eigen::MatrixXd::Zero(sigmas_f.cols(),sigmas_h.cols());

  for (auto i=0; i<num_sigmas_; i++) {
    // TODO: consider different residual function calls for process and observation.
    auto dx = residuals_x_(sigmas_f.row(i), x);
    auto dz = residuals_z_(sigmas_h.row(i), z);
    auto outer = dx*dz.transpose();
    CV += outer * Wc(i);
  }
  return CV;
}

std::tuple<Eigen::VectorXd, Eigen::MatrixXd>
unscented_transform(Eigen::MatrixXd sigmas, Eigen::VectorXd Wm, Eigen::VectorXd Wc, Eigen::MatrixXd noise_cov, std::function<Eigen::VectorXd(Eigen::VectorXd const&, Eigen::VectorXd const&)> residuals)
{
  // TODO: Need to handle case where simple dot product will not suffice
  // Construct the new state estimation.
  auto x = sigmas.transpose() * Wm;

  // Construct the covariance matrix.
  auto k_max = sigmas.rows();
  auto n = sigmas.cols();
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n,n);

  for (auto k=0; k < k_max; k++) {
    auto y = residuals(sigmas.row(k), x);
    auto outer = y*y.transpose();
    P += outer * Wc(k);
  }

  P += noise_cov;

  auto tranformed = std::make_pair(x,P);
  return tranformed;
}
} // namespace ukf
