#include <iomanip>
#include <cstdlib>
#include <math.h>
#include <cmath>
#include <random>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "gtest/gtest.h"
#include "../include/unscented_kalman_filter.h"


namespace rviz {
  cv::Point2i cv_offset(
      Eigen::Vector2d e_p, int image_width=2000, int image_height=2000){
    cv::Point2i output;
    output.x = int(e_p(0) * 100) + image_width/2;
    output.y = image_height - int(e_p(1) * 100) - image_height/3;
    return output;
  };

  void ellipse_drawing(
    cv::Mat bg_img, Eigen::Matrix2d pest, Eigen::Vector2d center,
    cv::Scalar ellipse_color=cv::Scalar(0, 0, 255)
  ){
    Eigen::EigenSolver<Eigen::Matrix2d> ces(pest);
    Eigen::Matrix2d e_value = ces.pseudoEigenvalueMatrix();
    Eigen::Matrix2d e_vector = ces.pseudoEigenvectors();

    double angle = std::atan2(e_vector(0, 1), e_vector(0, 0));
    cv::ellipse(
      bg_img,
      cv_offset(center, bg_img.cols, bg_img.rows),
      cv::Size(e_value(0,0)*100, e_value(1,1)*100),
      angle / M_PI * 180,
      0,
      360,
      ellipse_color,
      2,
      4);
  };
}

namespace ukf {
// Constan velocity model
auto motion_model_cv = [](Eigen::Vector4d x, Eigen::Vector2d u, double dt)-> Eigen::Vector4d {
  Eigen::Matrix4d F;
  F <<1.0,  dt,   0.,   0.,
          0., 1.0,   0.,   0.,
          0.,   0., 1.0,   dt,
          0.,   0.,   0.,  1.0;

  return F * x;
};

auto observation_model_cv = [](Eigen::Vector4d x) -> Eigen::Vector2d {
  Eigen::Matrix<double, 2, 4> H;
  H << 1., 0., 0., 0.,
       0., 0., 1., 0.;
  return H * x;
};

auto wrap_angles_default = [](Eigen::VectorXd x) -> Eigen::VectorXd {
  return x;
};


auto linear_motion_model = [](Eigen::Vector2d x, Eigen::Vector2d u, double dt)-> Eigen::Vector2d {
  Eigen::Matrix2d F;
  F <<  1.0, dt,
        0, 1.0;

  return F * x;
};

auto observation_model = [](Eigen::Vector2d x) -> Eigen::Vector2d {
  Eigen::Matrix<double, 2, 2> H;
  H << 1., 0.,
       0., 1.;
  return H * x;
};

auto observation_model_2x4 = [](Eigen::Vector4d x) -> Eigen::Vector2d {
  // State to measurement function H, is with dimensions dim_z x dim_x with ones  in the locations to be updated.
  Eigen::Matrix<double, 2, 4> H;
  H << 1., 0., 0., 0.,
       0., 1., 0., 0.;
  return H * x;
};

auto motion_model_4x4 = [](Eigen::Vector4d x, Eigen::Vector2d u, double dt) -> Eigen::Vector4d {
  Eigen::Matrix4d F;
  F <<1.0,   0.,   0.,   0.,
        0., 1.0,   0.,   0.,
        0.,   0., 1.0,   0.,
        0.,   0.,   0.,  0.; // TODO: Should this be a 1.0? PythonRobotics has 0.0

  // control input v (m/s), yaw_rate (rad/s)
  Eigen::Matrix<double, 4, 2> B;
  B << dt * std::cos(x(2)),  0.,
       dt * std::sin(x(2)),  0.,
                        0.0,  dt,
                        1.0,  0.0;
  return F * x + B * u;
};

auto wrap_angles = [](Eigen::Vector4d x) -> Eigen::Vector4d{
  // When state is represented by [x y yaw v].
  Eigen::Vector4d angles_wrapped = x;

  while (angles_wrapped(2) < -M_PI) {
    angles_wrapped(2) += (2*M_PI);
  }

  while (angles_wrapped(2) > M_PI) {
    angles_wrapped(2) -= (2*M_PI);
  }

  return angles_wrapped;
};

Eigen::VectorXd residuals(Eigen::VectorXd const& sigma, Eigen::VectorXd const& x)
{
  return sigma - x;
}

Eigen::VectorXd residuals_x(Eigen::VectorXd const& sigma, Eigen::VectorXd const& x)
{
  // Residuals when state is represented by [x y yaw v].
  Eigen::VectorXd residuals(x.rows());
  residuals(0) = sigma(0) - x(0);
  residuals(1) = sigma(1) - x(1);
  residuals(3) = sigma(3) - x(3);

  auto yaw_residual = sigma(2) - x(2);

  while (yaw_residual < -M_PI) {
    yaw_residual += (2*M_PI);
  }

  while (yaw_residual > M_PI) {
    yaw_residual -= (2*M_PI);
  }
  //assert((yaw_residual < M_PI) && (yaw_residual > -M_PI));
  residuals(2) = yaw_residual;
  return residuals;
}

Eigen::VectorXd mean_fn_x(Eigen::MatrixXd const& sigmas, Eigen::VectorXd const& Wm)
{
  // State mean when state is represented by [x y yaw v].
  Eigen::VectorXd mean(sigmas.cols());
  mean.setZero();
  double ss = 0.0;
  double sc = 0.0;

  for (auto i=0; i< sigmas.rows(); ++i) {
    Eigen::VectorXd s = sigmas.row(i);

    mean(0) += s(0) * Wm(i);
    mean(1) += s(1) * Wm(i);
    mean(3) += s(3) * Wm(i);
    ss += std::sin(s(2)) * Wm(i);
    sc += std::cos(s(2)) * Wm(i);
  }
  mean(2) = std::atan2(ss,sc);
  return mean;
}

Eigen::VectorXd mean_fn(Eigen::MatrixXd const& sigmas, Eigen::VectorXd const& Wm)
{
  return sigmas.transpose() * Wm;
}

auto subtract_fn = residuals;
auto subtract_fn_x = residuals_x;
}


TEST(UKFTest, NumSigmaTest) {
  auto points = ukf::MerweScaledSigmaPoints(2, 0.1,0.1,0.1, ukf::subtract_fn);
  auto n_sigmas = points.num_sigmas();
  EXPECT_EQ(n_sigmas, 5);
}


TEST(UKFTest, SigmaPoints2DTest) {
  auto n = 2;
  auto alpha = 1e-3;
  auto beta = 2.0;
  auto kappa = 0.0;
  auto points = ukf::MerweScaledSigmaPoints(n, alpha, beta, kappa, ukf::subtract_fn);

  Eigen::Vector2d x;
  x << 1.,2.;

  Eigen::Matrix2d P;
  P << 2.,1.2,
       1.2,2.;

  auto sigmas = points.sigma_points(x, P);
  Eigen::Matrix<double, 5, 2> sigmas_expected;

  sigmas_expected << 1., 2.,
                    1.002, 2.0012,
                    1., 2.0016,
                    0.9980, 1.9988,
                    1., 1.9984;

  ASSERT_TRUE((sigmas - sigmas_expected).norm() < 1e-5);
}


TEST(UKFTest, SigmaPointsTest) {
  auto points = ukf::MerweScaledSigmaPoints(4, 1e-3,2.0,0.0, ukf::subtract_fn);
  Eigen::Vector4d x = Eigen::Vector4d::Zero();
  Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
  auto sigmas = points.sigma_points(x, P);

  Eigen::Matrix<double, 9, 4> sigmas_expected;
  sigmas_expected << 0., 0., 0., 0.,
                    0.0020, 0., 0., 0.,
                    0., 0.0020, 0., 0.,
                    0., 0., 0.0020, 0.,
                    0., 0., 0., 0.0020,
                    -0.0020, 0., 0., 0.,
                    0., -0.0020, 0., 0.,
                    0., 0., -0.0020, 0.,
                    0., 0., 0., -0.0020;

  ASSERT_TRUE((sigmas - sigmas_expected).norm() < 1e-5);
  ASSERT_TRUE(std::abs(points.Wm.sum()-1.) < 1e-5);
}


TEST(UKFTest, ComputeWeightsTest) {
  auto n = 2;
  auto alpha = 1e-3;
  auto beta = 2.0;
  auto kappa = 0.0;
  auto points = ukf::MerweScaledSigmaPoints(n, alpha, beta, kappa, ukf::subtract_fn);
  auto wm = points.Wm;

  Eigen::Matrix<double,5,1> wm_expected;
  wm_expected << -999998.9999712,
                  249999.9999928,
                  249999.9999928,
                  249999.9999928,
                  249999.9999928;

  ASSERT_TRUE((wm - wm_expected).norm() < 1e-5);
}


TEST(UKFTest, UnscentedTransformTest) {
  auto n = 2;
  auto alpha = 1e-3;
  auto beta = 2.0;
  auto kappa = 0.0;
  auto points = ukf::MerweScaledSigmaPoints(n, alpha, beta, kappa, ukf::subtract_fn);

  Eigen::Vector2d x;
  x << 1.,2.;

  Eigen::Matrix2d P;
  P << 2.,1.2,
       1.2,2.;

  Eigen::Matrix2d noise_cov;
  noise_cov << 0.,0.,
               0.,0.;

  auto sigmas = points.sigma_points(x, P);

  auto pair = ukf::unscented_transform(sigmas, points.Wm, points.Wc, noise_cov, ukf::residuals, ukf::mean_fn);

  ASSERT_TRUE((std::get<0>(pair) - x).norm() < 1e-5);
  ASSERT_TRUE((std::get<1>(pair) - P).norm() < 1e-5);
}


TEST(UKFTest, ResidualsTest) {
  auto n = 2;
  auto alpha = 0.001;
  auto beta = 2.0;
  auto kappa = 0.0;
  auto points = ukf::MerweScaledSigmaPoints(n, alpha, beta, kappa, ukf::subtract_fn);

  Eigen::Vector2d x;
  x << 1.,2.;

  Eigen::Matrix2d P;
  P << 2.,1.2,
       1.2,2.;

  auto sigmas = points.sigma_points(x, P);

  Eigen::MatrixXd residuals = ukf::residuals(sigmas.row(1), x);
  Eigen::Matrix<double,2,1> residuals_expected;
  residuals_expected << 0.002,0.0012;
  ASSERT_TRUE((residuals - residuals_expected).norm() < 1e-5);
}


TEST(UKFTest, UKFPredictTest) {
  auto n = 2;
  auto alpha = 0.001;
  auto beta = 2.0;
  auto kappa = 0.0;
  auto points = ukf::MerweScaledSigmaPoints(n, alpha, beta, kappa, ukf::subtract_fn);

  Eigen::Vector2d x;
  x << 1.,2.;

  Eigen::Matrix2d P;
  P << 2.,1.2,
       1.2,2.;

  Eigen::Vector2d u;
  u << 1.0, 0.1;

  Eigen::VectorXd  z = Eigen::VectorXd::Zero(n);

  auto dim_z = n;
  auto dt = 0.1;
  auto ukf = ukf::UnscentedKalmanFilter(
                n,
                dim_z,
                dt,
                points,
                ukf::wrap_angles_default,
                ukf::linear_motion_model,
                ukf::observation_model,
                ukf::residuals,
                ukf::residuals,
                ukf::mean_fn,
                ukf::mean_fn);

  ukf.predict(u);
  ukf.update(z);
  //TODO: FINISH TESTING
}


TEST(UKFTest, CrossVarianceTest) {
  auto n = 2;
  auto dim_z = 2;
  auto dt = 0.1;
  auto alpha = 0.001;
  auto beta = 2.0;
  auto kappa = 0.0;
  auto points = ukf::MerweScaledSigmaPoints(n, alpha, beta, kappa, ukf::subtract_fn);

  Eigen::Vector2d x;
  x << 1.,2.;

  Eigen::Matrix2d P;
  P << 2.,1.2,
       1.2,2.;

  // control input
  Eigen::Vector2d u;
  u << 1.0, 0.1;

  Eigen::VectorXd  z = Eigen::VectorXd::Zero(n);

  // Get sigmas_f (process)
  Eigen::MatrixXd sigmas = points.sigma_points(x, P);
  auto n_sigmas = points.num_sigmas();
  Eigen::MatrixXd sigmas_f = Eigen::MatrixXd::Zero(n_sigmas, n);

  // Pass the sigmas through the process model.
  for (auto i=0; i<n_sigmas; i++) {
    sigmas_f.row(i) = ukf::linear_motion_model(sigmas.row(i), u, dt);
  }

  // Get sigmas_h (measurement)
  Eigen::MatrixXd sigmas_h = Eigen::MatrixXd::Zero(n_sigmas, dim_z);

  // Pass the sigmas processed by the process(motion) model through the observation model.
  for (auto i=0; i<n_sigmas; i++) {
    sigmas_h.row(i) = ukf::observation_model(sigmas_f.row(i));
  }

  auto ukf = ukf::UnscentedKalmanFilter(
                n,
                dim_z,
                dt,
                points,
                ukf::wrap_angles_default,
                ukf::linear_motion_model,
                ukf::observation_model,
                ukf::residuals,
                ukf::residuals,
                ukf::mean_fn,
                ukf::mean_fn);

  auto CV = ukf.cross_variance(x, z,sigmas_f, sigmas_h);
  // TODO: Finish test
}


TEST(UKFTest, SimulationSimpleCVTest) {
  // Constant Velocity model
  auto n = 4; // [x, x_dot, y, y_dot]
  auto dt = 0.1;
  auto alpha = 0.1;
  auto beta = 2.0;
  auto kappa = 1.0;
  auto points = ukf::MerweScaledSigmaPoints(n, alpha, beta, kappa, ukf::subtract_fn);

  // Dont really need, but current implementation requires a u to pass to motion_model.
  Eigen::Vector2d u;
  u << 0.0, 0.0;

  // noisy control input
  Eigen::Vector2d ud;

  // observation z
  Eigen::Vector2d z;

  // dead reckoning
  Eigen::Vector4d xDR;
  xDR << 0.1,1.0,0.1,1.0;

  // ground truth reading
  Eigen::Vector4d xTrue;
  xTrue << 0.1,1.0,0.1,1.0;

  Eigen::Vector4d xEst;
  xEst << 0.1,1.0,0.1,1.0;

  std::vector<Eigen::Vector4d> hxDR;
  std::vector<Eigen::Vector4d> hxTrue;
  std::vector<Eigen::Vector4d> hxEst;
  std::vector<Eigen::Vector2d> hz;

  Eigen::Matrix4d PEst = Eigen::Matrix4d::Identity();

  // Motion model covariance
  Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
  Q(0,0)=0.25 * (dt*dt*dt*dt) * 0.02;
  Q(0,1)=0.5 * (dt*dt*dt) * 0.02;
  Q(1,0)=0.5 * (dt*dt*dt) * 0.02;
  Q(1,1)=(dt*dt) * 0.02;
  Q(2,2)=0.25 * (dt*dt*dt*dt) * 0.02;
  Q(2,3)=0.5 * (dt*dt*dt) * 0.02;
  Q(3,2)=0.5 * (dt*dt*dt) * 0.02;
  Q(3,3)=(dt*dt) * 0.02;

  Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
  R(0,0)=0.09;
  R(1,1)=0.09;

  // Motion model simulation error
  Eigen::Matrix2d Qsim = Eigen::Matrix2d::Identity();
  Qsim(0,0)=0.25 * (dt*dt*dt*dt) * 0.02;
  Qsim(1,1)=(dt*dt) * 0.02;

  // Observation model simulation error
  Eigen::Matrix2d Rsim = Eigen::Matrix2d::Identity();
  Rsim(0,0)=0.09;
  Rsim(1,1)=0.09;

  auto ukf = ukf::UnscentedKalmanFilter(
                n,
                z.rows(),
                dt,
                points,
                ukf::wrap_angles_default,
                ukf::motion_model_cv,
                ukf::observation_model_cv,
                ukf::residuals,
                ukf::residuals,
                ukf::mean_fn,
                ukf::mean_fn);
    ukf.Q = Q;
    ukf.R = R;
    auto time = 0.0;

    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> gaussian_d{0,1};

    cv::namedWindow("ukf");
    while (time < 50.0) {
      time += dt;

      ud(0) = u(0) + gaussian_d(gen) * Qsim(0,0);
      ud(1) = u(1) + gaussian_d(gen) * Qsim(1,1);

      xTrue = ukf::motion_model_cv(xTrue, u, dt);
      //std::cout << "xTrue" << xTrue << std::endl;
      xDR = ukf::motion_model_cv(xDR, ud, dt);

      z(0) = time + gaussian_d(gen) * Rsim(0,0);
      z(1) = time + gaussian_d(gen) * Rsim(1,1);

      ukf.predict(ud);
      ukf.update(z);

      auto xEst = ukf.x_post;

      hxDR.push_back(xDR);
      hxTrue.push_back(xTrue);
      hxEst.push_back(xEst);
      hz.push_back(z);

      //visualization

      cv::Mat bg(700,1500, CV_8UC3, cv::Scalar(255,255,255));

      for(unsigned int j=0; j<hxDR.size(); j++){

        // Green groundtruth
        // State is found at index 0 and 2.
        Eigen::Vector2d x_true;
        x_true << hxTrue[j](0), hxTrue[j](2);
        cv::circle(bg, rviz::cv_offset(x_true, bg.cols, bg.rows),
                   7, cv::Scalar(0,255,0), 2);

        // blue estimation
        Eigen::Vector2d x_est;
        x_est << hxEst[j](0),hxEst[j](2);
        cv::circle(bg, rviz::cv_offset(x_est, bg.cols, bg.rows),
                   10, cv::Scalar(255,0,0), 5);

        // black dead reckoning
        Eigen::Vector2d x_dr;
        x_dr << hxDR[j](0),hxDR[j](2);
        cv::circle(bg, rviz::cv_offset(x_dr, bg.cols, bg.rows),
                   7, cv::Scalar(0, 0, 0), -1);
      }

      // red observation
      for(unsigned int i=0; i<hz.size(); i++){
        cv::circle(bg, rviz::cv_offset(hz[i], bg.cols, bg.rows),
                 7, cv::Scalar(0, 0, 255), -1);
      }

      Eigen::Vector2d ellipse_x_est;
      ellipse_x_est << xEst(0), xEst(2);
      rviz::ellipse_drawing(bg, PEst.block(0,0,2,2), ellipse_x_est);

      cv::imshow("ukf", bg);
      cv::waitKey(5);

    }
}


TEST(UKFTest, SimulationTest) {
  auto n = 4; // [x y yaw v]'
  auto dt = 0.1;
  auto alpha = 0.001;
  auto beta = 2.0;
  auto kappa = 0.0;
  auto points = ukf::MerweScaledSigmaPoints(n, alpha, beta, kappa, ukf::subtract_fn_x);

  // control input v (m/s), yaw_rate (rad/s)
  Eigen::Vector2d u;
  u << 1.0, 0.1;

  // noisy control input
  Eigen::Vector2d ud;

  // observation z
  Eigen::Vector2d z;

  // dead reckoning
  Eigen::Vector4d xDR;
  xDR << 0.1,0.1,0.0,1.0;

  // ground truth reading
  Eigen::Vector4d xTrue;
  xTrue << 0.1,0.1,0.0,1.0;;

  Eigen::Vector4d xEst;
  xEst << 0.1,0.1,0.0,1.0;;

  std::vector<Eigen::Vector4d> hxDR;
  std::vector<Eigen::Vector4d> hxTrue;
  std::vector<Eigen::Vector4d> hxEst;
  std::vector<Eigen::Vector2d> hz;

  Eigen::Matrix4d PEst = Eigen::Matrix4d::Identity();

  // Motional model covariance
  Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
  Q(0,0)=0.25 * (dt*dt*dt*dt) * 0.02;
  Q(0,1)=0.5 * (dt*dt*dt) * 0.02;
  Q(1,0)=0.5 * (dt*dt*dt) * 0.02;
  Q(1,1)=(dt*dt) * 0.02;
  Q(2,2)=0.25 * (dt*dt*dt*dt) * 0.02;
  Q(2,3)=0.5 * (dt*dt*dt) * 0.02;
  Q(3,2)=0.5 * (dt*dt*dt) * 0.02;
  Q(3,3)=(dt*dt) * 0.02;

  Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
  R(0,0)=0.09;
  R(1,1)=0.09;

  // Motion model simulation error
  Eigen::Matrix2d Qsim = Eigen::Matrix2d::Identity();
  Qsim(0,0)=0.09;
  Qsim(1,1)=0.09;

  // Observation model simulation error
  Eigen::Matrix2d Rsim = Eigen::Matrix2d::Identity();
  Rsim(0,0)=0.09;
  Rsim(1,1)=0.09;

  auto ukf = ukf::UnscentedKalmanFilter(
                n,
                z.rows(),
                dt,
                points,
                ukf::wrap_angles,
                ukf::motion_model_4x4,
                ukf::observation_model_2x4,
                ukf::residuals_x,
                ukf::residuals,
                ukf::mean_fn_x,
                ukf::mean_fn);

  // Initial settings.
  ukf.x = xTrue;
  ukf.Q = Q;
  ukf.R = R;

  auto time = 0.0;

  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> gaussian_d{0,1};

  cv::namedWindow("ukf");
  while (time < 100.0) {
    time += dt;

    ud(0) = u(0) + gaussian_d(gen) * Qsim(0,0);
    ud(1) = u(1) + gaussian_d(gen) * Qsim(1,1);

    xTrue = ukf::motion_model_4x4(xTrue, u, dt);

    xDR = ukf::motion_model_4x4(xDR, ud, dt);

    z(0) = xTrue(0) + gaussian_d(gen) * Rsim(0,0);
    z(1) = xTrue(1) + gaussian_d(gen) * Rsim(1,1);

    ukf.predict(ud);
    ukf.update(z);

    auto xEst = ukf.x_post;

    hxDR.push_back(xDR);
    hxTrue.push_back(xTrue);
    hxEst.push_back(xEst);
    hz.push_back(z);

    //visualization
    cv::Mat bg(700,1500, CV_8UC3, cv::Scalar(255,255,255));
    for(unsigned int j=0; j<hxDR.size(); j++){

      // green groundtruth
      cv::circle(bg, rviz::cv_offset(hxTrue[j].head(2), bg.cols, bg.rows),
                 7, cv::Scalar(0,255,0), -1);

      // blue estimation
      cv::circle(bg, rviz::cv_offset(hxEst[j].head(2), bg.cols, bg.rows),
                 10, cv::Scalar(255,0,0), 5);

      // black dead reckoning
      cv::circle(bg, rviz::cv_offset(hxDR[j].head(2), bg.cols, bg.rows),
                 7, cv::Scalar(0, 0, 0), -1);
    }

    // red observation
    for(unsigned int i=0; i<hz.size(); i++){
      cv::circle(bg, rviz::cv_offset(hz[i], bg.cols, bg.rows),
               7, cv::Scalar(0, 0, 255), -1);
    }

    rviz::ellipse_drawing(bg, PEst.block(0,0,2,2), xEst.head(2));
    cv::imshow("ukf", bg);
    cv::waitKey(5);
  }
}


TEST(UKFTest, InitTest) {
  auto dim_x = 4;
  auto dim_z = 2;
  auto dt = 1.0;
  auto alpha = 1e-3;
  auto beta = 2.0;
  auto kappa = 0.0;
  auto points = ukf::MerweScaledSigmaPoints(dim_x, alpha, beta, kappa, ukf::subtract_fn);

  ukf::UnscentedKalmanFilter ukf = ukf::UnscentedKalmanFilter(
                                                dim_x,
                                                dim_z,
                                                dt,
                                                points,
                                                ukf::wrap_angles_default,
                                                ukf::linear_motion_model,
                                                ukf::observation_model,
                                                ukf::residuals,
                                                ukf::residuals,
                                                ukf::mean_fn,
                                                ukf::mean_fn);
  uint dim = ukf.x.size();
  EXPECT_EQ(dim, 4);
}
