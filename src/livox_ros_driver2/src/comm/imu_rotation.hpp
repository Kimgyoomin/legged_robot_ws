// MIT License. RUBI delivery adaptation, 2026.
#pragma once
#include <cmath>

namespace livox_ros {
// Same Rz(yaw) Ry(pitch) Rx(roll) convention as the upstream point transform.
template<class Imu> void rotate_imu(double roll, double pitch, double yaw, Imu &imu) {
  constexpr double rad = 3.14159265358979323846 / 180.;
  const double cr=std::cos(roll*rad), sr=std::sin(roll*rad);
  const double cp=std::cos(pitch*rad), sp=std::sin(pitch*rad);
  const double cy=std::cos(yaw*rad), sy=std::sin(yaw*rad);
  const double r[3][3] = {{cp*cy, sr*sp*cy-cr*sy, cr*sp*cy+sr*sy},
    {cp*sy, sr*sp*sy+cr*cy, cr*sp*sy-sr*cy}, {-sp,sr*cp,cr*cp}};
  const auto rotate = [&](float &x, float &y, float &z) {
    const double a=x,b=y,c=z;
    x=r[0][0]*a+r[0][1]*b+r[0][2]*c;
    y=r[1][0]*a+r[1][1]*b+r[1][2]*c;
    z=r[2][0]*a+r[2][1]*b+r[2][2]*c;
  };
  rotate(imu.gyro_x, imu.gyro_y, imu.gyro_z);
  rotate(imu.acc_x, imu.acc_y, imu.acc_z);
}
}  // namespace livox_ros
