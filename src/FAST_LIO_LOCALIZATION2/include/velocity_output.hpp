#pragma once

#include <Eigen/Core>

namespace fastlio_output
{
inline Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d out;
  out << 0., -v.z(), v.y(), v.z(), 0., -v.x(), -v.y(), v.x(), 0.;
  return out;
}

struct BodyVelocity
{
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
  Eigen::Matrix<double, 6, 6> covariance;
};

// FAST-LIO's velocity is at the IMU origin in camera_init coordinates.
// Odometry twist must instead use child_frame_id (body) coordinates.
// State tangent order: pos, rot, extrinsic rot/trans, vel, bg, ba, gravity.
template<typename Covariance>
BodyVelocity body_velocity(
  const Eigen::Matrix3d &world_from_body, const Eigen::Vector3d &world_velocity,
  const Eigen::Vector3d &gyro, const Eigen::Vector3d &gyro_bias,
  const Covariance &state_covariance, const Eigen::Vector3d &gyro_variance)
{
  BodyVelocity out;
  out.linear = world_from_body.transpose() * world_velocity;
  out.angular = gyro - gyro_bias;
  Eigen::Matrix<double, 6, 23> jacobian = Eigen::Matrix<double, 6, 23>::Zero();
  // The rotation error uses a right SO(3) perturbation.
  jacobian.template block<3, 3>(0, 3) = skew(out.linear);
  jacobian.template block<3, 3>(0, 12) = world_from_body.transpose();
  jacobian.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
  out.covariance = jacobian * state_covariance * jacobian.transpose();
  // Approximation: treats gyro measurement noise as independent of the state.
  // This is model uncertainty, not a calibrated velocity quality guarantee.
  out.covariance.template block<3, 3>(3, 3).diagonal() += gyro_variance;
  return out;
}
}  // namespace fastlio_output
