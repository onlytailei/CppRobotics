#include "gtest/gtest.h"
#include "../include/unscented_kalman_filter.h"


TEST(UKFTest, NumSigmaTest) {
  auto points = cpprobotics::MerweScaledSigmaPoints(2, 0.1,0.1,0.1);
  auto n_sigmas = points.numSigmas();
  EXPECT_EQ(n_sigmas, 5);
}


TEST(UKFTest, SigmaPoints2DTest) {
  auto n = 2;
  auto alpha = 0.001;
  auto beta = 2.0;
  auto kappa = 0.0;

  auto points = cpprobotics::MerweScaledSigmaPoints(n, alpha, beta, kappa);

  Eigen::Vector2f x;
  x << 1.,2.;

  Eigen::Matrix2f P;
  P << 2.,1.2,
       1.2,2.;

  auto sigmas = points.sigmaPoints(x, P);

  Eigen::Matrix<float, 5, 2> sigma_expected;
  sigma_expected << 1., 2.,
                    1.00201, 2.00121,
                    1., 2.00161,
                    0.997987, 1.99879,
                    1., 1.99839;

  ASSERT_TRUE(sigmas.isApprox(sigma_expected));
}


TEST(UKFTest, SigmaPointsTest) {
  auto points = cpprobotics::MerweScaledSigmaPoints(4, 0.1,2.0,-1.0);

  Eigen::Vector4f x = Eigen::Vector4f::Zero();
  Eigen::Matrix4f P = Eigen::Matrix4f::Identity();
  auto sigmas = points.sigmaPoints(x, P);

  Eigen::Matrix<float, 9, 4> sigma_expected;
  sigma_expected << 0., 0., 0., 0.,
                    0.173205, 0., 0., 0.,
                    0., 0.173205, 0., 0.,
                    0., 0., 0.173205, 0.,
                    0., 0., 0., 0.173205,
                    -0.173205, 0., 0., 0.,
                    0., -0.173205, 0., 0.,
                    0., 0., -0.173205, 0.,
                    0., 0., 0.,-0.173205;

  ASSERT_TRUE(sigmas.isApprox(sigma_expected));
}


TEST(UKFTest, ComputeWeightsTest) {
  auto n = 2;
  auto alpha = 0.001;
  auto beta = 2.0;
  auto kappa = 0.0;

  auto points = cpprobotics::MerweScaledSigmaPoints(n, alpha, beta, kappa);
  auto wm = points.Wm;
  std::cout << wm << std::endl;
  // TODO:: finish this test
}


TEST(UKFTest, InitTest) {
  auto dim_x = 4;
  auto dim_z = 2;
  auto dt = 1.0;
  cpprobotics::UnscentedKalmanFilter ukf = cpprobotics::UnscentedKalmanFilter(dim_x, dim_z, dt);
  uint dim = ukf.x.size();

  EXPECT_EQ(dim, 4);
}
