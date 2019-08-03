
#ifndef UNSCENTED_KALMAN_FILTER_H
#define UNSCENTED_KALMAN_FILTER_H

#include <iostream>
#include <random>
#include <cmath>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace ukf {

// Uses R. Van der Merwe method to generate sigma points and weights.
class MerweScaledSigmaPoints {

public:
  MerweScaledSigmaPoints(
    uint const n,
    double const alpha,
    double const beta,
    double const kappa,
    std::function<Eigen::VectorXd(Eigen::VectorXd const&, Eigen::VectorXd const&)> subtract_fn
  );

  uint n;
  double alpha;
  double beta;
  double kappa;
  Eigen::VectorXd Wm;  // Weight for each sigma point for the mean
  Eigen::VectorXd Wc;  // Weight for each sigma point for the covariance
  std::function<Eigen::VectorXd(Eigen::VectorXd const&, Eigen::VectorXd const&)> subtract_fn_;

  uint num_sigmas() const;

  // Computes the sigma points for an Unscented Kalman Filter given the mean and covariance.
  // Returns a (2*n+1, n) dimension matrix containing the sigma points.
  Eigen::MatrixXd sigma_points(Eigen::VectorXd const& t_x, Eigen::MatrixXd const& t_P);
};

class UnscentedKalmanFilter {

public:
  UnscentedKalmanFilter(
    uint const dim_x,
    uint const dim_z,
    double const dt,
    MerweScaledSigmaPoints& points,
    std::function<Eigen::VectorXd(Eigen::VectorXd)> wrap_angles,
    std::function<Eigen::VectorXd(Eigen::VectorXd, Eigen::VectorXd, double)> motion_model,
    std::function<Eigen::VectorXd(Eigen::VectorXd)> observation_model,
    std::function<Eigen::VectorXd(Eigen::VectorXd const&, Eigen::VectorXd const&)> residuals_x,
    std::function<Eigen::VectorXd(Eigen::VectorXd const&, Eigen::VectorXd const&)> residuals_z,
    std::function<Eigen::VectorXd(Eigen::MatrixXd const&, Eigen::VectorXd const&)> mean_fn_x,
    std::function<Eigen::VectorXd(Eigen::MatrixXd const&, Eigen::VectorXd const&)> mean_fn_z
  );

  Eigen::VectorXd Wm;
  Eigen::VectorXd Wc;
  Eigen::VectorXd x;        // State estimate vector// = Eigen::VectorXd::Zero(dim_x);
  Eigen::MatrixXd P;        // Covariance estimate matrix// = Eigen::Matrix4f::Identity();
  Eigen::VectorXd x_prior;
  Eigen::MatrixXd P_prior;
  Eigen::VectorXd x_post;
  Eigen::MatrixXd P_post;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
  Eigen::MatrixXd K;        // Kalman Gain
  Eigen::VectorXd y;
  Eigen::MatrixXd S;        // System uncertainty
  Eigen::MatrixXd SI;       // Inverse system uncertainty

  std::function<Eigen::VectorXd(Eigen::VectorXd)> wrap_angles_;
  std::function<Eigen::VectorXd(Eigen::VectorXd, Eigen::VectorXd, double)> motion_model_;
  std::function<Eigen::VectorXd(Eigen::VectorXd)> observation_model_;
  std::function<Eigen::VectorXd(Eigen::VectorXd const&, Eigen::VectorXd const&)> residuals_fn_x_;
  std::function<Eigen::VectorXd(Eigen::VectorXd const&, Eigen::VectorXd const&)> residuals_fn_z_;
  std::function<Eigen::VectorXd(Eigen::MatrixXd const&, Eigen::VectorXd const&)> mean_fn_x_;
  std::function<Eigen::VectorXd(Eigen::MatrixXd const&, Eigen::VectorXd const&)> mean_fn_z_;

  // Performs the predict step of the UKF. On return, member variables x and P contain
  // the predicted state and covariance.
  void predict(Eigen::VectorXd const& u);

  void update(Eigen::VectorXd const& z);

  Eigen::MatrixXd cross_variance(Eigen::VectorXd const& x, Eigen::VectorXd const& z, Eigen::MatrixXd const& sigmas_f, Eigen::MatrixXd const& sigmas_h);

private:
  uint dim_x_;
  uint dim_z_;
  double dt_;
  MerweScaledSigmaPoints points_;

  bool predict_called_;     // Predict needs to be called once before update can be called.
  uint num_sigmas_;         // Number of sigma points.
  Eigen::MatrixXd sigmas_;
  Eigen::MatrixXd sigmas_f_;

};

std::tuple<Eigen::VectorXd, Eigen::MatrixXd> unscented_transform(Eigen::MatrixXd const& sigmas, Eigen::VectorXd const& Wm, Eigen::VectorXd const& Wc, Eigen::MatrixXd const& noise_cov, std::function<Eigen::VectorXd(Eigen::VectorXd const&, Eigen::VectorXd const&)> residuals, std::function<Eigen::VectorXd(Eigen::MatrixXd const&, Eigen::VectorXd const&)> mean_fn);
}

#endif // UNSCENTED_KALMAN_FILTER_H
