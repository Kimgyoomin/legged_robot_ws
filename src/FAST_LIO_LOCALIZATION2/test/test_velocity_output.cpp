#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include "velocity_output.hpp"

TEST(VelocityOutput, RotatesWorldVelocityAndUsesPosteriorBias)
{
  const Eigen::Matrix3d rotation = Eigen::AngleAxisd(
      1.5707963267948966, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Matrix<double, 23, 23> p = Eigen::Matrix<double, 23, 23>::Zero();
  p(12, 12) = .01;
  p(13, 13) = .04;
  p(17, 17) = .02;
  const auto out = fastlio_output::body_velocity(rotation, {0., 1., 0.},
      {0., 0., .6}, {0., 0., .1}, p, {.001, .001, .001});
  EXPECT_NEAR(out.linear.x(), 1., 1e-12);
  EXPECT_NEAR(out.linear.y(), 0., 1e-12);
  EXPECT_NEAR(out.angular.z(), .5, 1e-12);
  EXPECT_NEAR(out.covariance(0, 0), .04, 1e-12);
  EXPECT_NEAR(out.covariance(1, 1), .01, 1e-12);
  EXPECT_NEAR(out.covariance(5, 5), .021, 1e-12);
}

TEST(VelocityOutput, OrientationUncertaintyContributesToTwist)
{
  Eigen::Matrix<double, 23, 23> p = Eigen::Matrix<double, 23, 23>::Zero();
  p(5, 5) = .03;
  const auto out = fastlio_output::body_velocity(Eigen::Matrix3d::Identity(),
      {2., 0., 0.}, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), p,
      Eigen::Vector3d::Zero());
  EXPECT_NEAR(out.covariance(1, 1), .12, 1e-12);
  EXPECT_TRUE(out.covariance.isApprox(out.covariance.transpose()));
}
