#pragma once
#include <array>
#include <cmath>
#include <stdexcept>

namespace legged_navigation {
// Distances are from the sensor origin. Bearing is measured in base-frame axes.
// Only rotate the sensor-relative vector: translation would move the sector apex.
class SectorFilter {
 public:
  SectorFilter(double minimum, double maximum, double front_minimum, double half_angle) {
    if (!std::isfinite(minimum) || !std::isfinite(maximum) ||
        !std::isfinite(front_minimum) || !std::isfinite(half_angle) ||
        minimum < 0 || maximum <= minimum || front_minimum < 0 ||
        front_minimum > minimum || half_angle < 0 || half_angle > 90)
      throw std::invalid_argument("Invalid sector ranges or half-angle (0..90 degrees)");
    min2_ = minimum * minimum;
    max2_ = maximum * maximum;
    front2_ = front_minimum * front_minimum;
    enabled_ = half_angle > 0 && front_minimum < minimum;
    const double c = std::cos(half_angle * std::acos(-1.0) / 180.0);
    cos2_ = half_angle == 90 ? 0.0 : c * c;
  }

  bool keep(double x, double y, double z, const std::array<double, 6> & rotation_xy) const {
    const double d2 = x*x + y*y + z*z;
    if (!std::isfinite(d2) || d2 <= 0 || d2 > max2_) return false;
    if (d2 >= min2_) return true;
    if (!enabled_ || d2 < front2_) return false;
    const double bx = rotation_xy[0]*x + rotation_xy[1]*y + rotation_xy[2]*z;
    const double by = rotation_xy[3]*x + rotation_xy[4]*y + rotation_xy[5]*z;
    const double horizontal2 = bx*bx + by*by;
    // No atan2/sqrt per point. A vertical ray has no horizontal bearing.
    return bx >= 0 && horizontal2 > 0 && bx*bx >= cos2_*horizontal2;
  }

 private:
  double min2_, max2_, front2_, cos2_;
  bool enabled_;
};
}  // namespace legged_navigation
