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
    double const kappa,
    std::function<Eigen::VectorXd(Eigen::VectorXd const&, Eigen::VectorXd const&)> subtract_fn)
: n(n),
  alpha(alpha),
  beta(beta),
  kappa(kappa),
  subtract_fn_(subtract_fn)
{
  double lambda = (alpha*alpha) * (n+kappa) - n;
  double c = 0.5 / (n+lambda);

  // The sum of Wm should be 1.
  // The sum of Wc should ~4.
  // http://users.isy.liu.se/rt/fredrik/edu/sensorfusion/lecture5.pdf
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
  Eigen::VectorXd const& t_x,
  Eigen::MatrixXd const& t_P)
{
  if (t_x.size()!=n) {
    throw std::string("Expected size(x): "+std::to_string(n));
  }

  double lambda = (alpha*alpha) * (n+kappa) - n;

  // Use cholesky decomposition.
  Eigen::MatrixXd L = ((lambda+n) * t_P).llt().matrixL();
  Eigen::MatrixXd sigmas = Eigen::MatrixXd::Zero(2*n+1, n);

  sigmas.row(0) = t_x;
  for (auto i=0; i<n; ++i) {
    // Need to address subtraction when dealing with angles.
    sigmas.row(i+1) =  subtract_fn_(t_x, -1.0*L.col(i));
    sigmas.row(n+i+1) = subtract_fn_(t_x, L.col(i));
  }
  return sigmas;
}


UnscentedKalmanFilter::UnscentedKalmanFilter(
    uint const dim_x,
    uint const dim_z,
    double const dt,
    MerweScaledSigmaPoints& points,
    std::function<Eigen::VectorXd(Eigen::VectorXd)> wrap_angles,
    std::function<Eigen::VectorXd(Eigen::VectorXd, Eigen::VectorXd, double)> motion_model,
    std::function<Eigen::VectorXd(Eigen::VectorXd)> observation_model,
    std::function<Eigen::VectorXd(Eigen::VectorXd const&, Eigen::VectorXd const&)> residuals_fn_x,
    std::function<Eigen::VectorXd(Eigen::VectorXd const&, Eigen::VectorXd const&)> residuals_fn_z,
    std::function<Eigen::VectorXd(Eigen::MatrixXd const&, Eigen::VectorXd const&)> mean_fn_x,
    std::function<Eigen::VectorXd(Eigen::MatrixXd const&, Eigen::VectorXd const&)> mean_fn_z)
: dim_x_(dim_x),
  dim_z_(dim_z),
  dt_(dt),
  points_(points),
  wrap_angles_(wrap_angles),
  motion_model_(motion_model),
  observation_model_(observation_model),
  residuals_fn_x_(residuals_fn_x),
  residuals_fn_z_(residuals_fn_z),
  mean_fn_x_(mean_fn_x),
  mean_fn_z_(mean_fn_z)
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


void UnscentedKalmanFilter::predict(Eigen::VectorXd const& u)
{
  // The function predict() needs to be called at least once before calling update().
  if (!predict_called_) { predict_called_ = true; };

  sigmas_ = points_.sigma_points(x, P);

  sigmas_f_ = Eigen::MatrixXd::Zero(num_sigmas_, dim_x_);

  // Pass the sigmas through the process model.
  for (auto i=0; i<num_sigmas_; ++i) {
    sigmas_f_.row(i) = motion_model_(sigmas_.row(i), u, dt_);
  }

  std::tuple<Eigen::VectorXd, Eigen::MatrixXd> prior = unscented_transform(sigmas_f_, Wm, Wc, Q, residuals_fn_x_, mean_fn_x_);
  x = std::get<0>(prior);
  P = std::get<1>(prior);

  x = wrap_angles_(x);

  x_prior = x;
  P_prior = P;
}


void UnscentedKalmanFilter::update(Eigen::VectorXd const& z)
{
  // Make sure that predict called once before update
  if (!predict_called_) {
    throw std::string("The function predict() needs to be called at least once.");
  }

  // No measurement can really be error free, address case where 0.
  // TODO: CLEAN UP
  for (auto i=0; i<R.rows(); ++i) {
    if (R(i,i) < 1e-9) {
      // TODO: Output a debug warning that you are adding some noise.
      R(i,i) = 1e-9;
    }
  }

  Eigen::MatrixXd sigmas_h = Eigen::MatrixXd::Zero(num_sigmas_, dim_z_);

  // Pass the sigmas processed by the process(motion) model through the observation model.
  for (auto i=0; i<num_sigmas_; ++i) {
    sigmas_h.row(i) = observation_model_(sigmas_f_.row(i));
  }

  std::tuple<Eigen::VectorXd, Eigen::MatrixXd> pair = unscented_transform(sigmas_h, Wm, Wc, R, residuals_fn_z_, mean_fn_z_);
  auto zp = std::get<0>(pair);
  S = std::get<1>(pair);
  SI = S.inverse();

  // Compute the crosse variance of the state and measurements.
  Eigen::MatrixXd Pxz = cross_variance(x, z, sigmas_f_, sigmas_h);

  // Compute the Kalman gain.
  K = Pxz * SI;

  // Compute the innovation.
  y = residuals_fn_z_(z,zp);

  // Update the gaussian state estimate.
  x += K*y;

  x = wrap_angles_(x);

  // Compute the new estimate error covariance P = P - (K * S * K').
  P -= K*(S*K.transpose());

  //TODO: Handle negative (read: bad) covariances in the measurement.
  x_post = x;
  P_post = P;
}


Eigen::MatrixXd UnscentedKalmanFilter::cross_variance(
  Eigen::VectorXd const& x,
  Eigen::VectorXd const& z,
  Eigen::MatrixXd const& sigmas_f,
  Eigen::MatrixXd const& sigmas_h)
{
  Eigen::MatrixXd CV = Eigen::MatrixXd::Zero(sigmas_f.cols(),sigmas_h.cols());

  for (auto i=0; i<num_sigmas_; ++i) {
    // TODO: consider different residual function calls for process and observation.
    // Results in a mx1 vector.
    auto dx = residuals_fn_x_(sigmas_f.row(i), x);
    // Results in a nx1 vector.
    auto dz = residuals_fn_z_(sigmas_h.row(i), z);

    // Results in a mxn matrix as a result mx1 dot 1xn.
    auto outer = dx*dz.transpose();

    CV += outer * Wc(i);
  }
  return CV;
}


std::tuple<Eigen::VectorXd, Eigen::MatrixXd>
unscented_transform(
  Eigen::MatrixXd const& sigmas,
  Eigen::VectorXd const& Wm,
  Eigen::VectorXd const& Wc,
  Eigen::MatrixXd const& noise_cov,
  std::function<Eigen::VectorXd(Eigen::VectorXd const&, Eigen::VectorXd const&)> residuals_fn,
  std::function<Eigen::VectorXd(Eigen::MatrixXd const&, Eigen::VectorXd const&)> mean_fn)
{
  // Simple dot product will not suffice when representation has angles.
  // Construct the new state estimation.
  auto x = mean_fn(sigmas, Wm);

  // Construct the covariance matrix.
  auto k_max = sigmas.rows();
  auto n = sigmas.cols();
  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(n,n);

  for (auto k=0; k < k_max; ++k) {
    auto y = residuals_fn(sigmas.row(k), x);
    //TODO: can we just take the dot when dealing with angles
    auto outer = y*y.transpose();
    cov += Wc(k) * outer;
  }

  //TODO: Handle negative (read: bad) covariances in the measurement.
  // https://github.com/cra-ros-pkg/robot_localization/blob/melodic-devel/src/ukf.cpp
  for (auto r=0; r< cov.rows(); ++r) {
    for (auto c=0; c< cov.cols(); ++c) {
      if (cov(r,c) < 0.0) {
        cov(r,c) = std::fabs(cov(r,c));
      }
    }
  }
  cov += noise_cov;
  auto tranformed = std::make_pair(x,cov);
  return tranformed;
}
} // namespace ukf
