#pragma once

#include <cmath>
#include <iostream>

namespace vk {
namespace cameras {

// This class implements the FOV distortion model of Deverneay and Faugeras,
// Straight lines have to be straight, 2001.
class AtanDistortion
{
public:

  AtanDistortion(const double& s)
    : s_(s)
    , s_inv_(1.0/s_)
    , tans_(2.0*std::tan(s_/2.0))
    , tans_inv_(1.0/tans_)
  {}

  ~AtanDistortion() = default;

	inline void distort(double& x, double& y) const
  {
    const double r = std::sqrt(x*x + y*y);
    const double factor = (r < 0.001) ? 1.0 : s_inv_* std::atan(r * tans_) / r;
    x *= factor;
    y *= factor;
  }

  inline void undistort(double& x, double& y) const
  {
    const double dist_r = std::sqrt(x*x + y*y);
    const double r = std::tan(dist_r * s_) * tans_inv_;
    const double d_factor = (dist_r > 0.01) ? r / dist_r : 1.0;
    x *= d_factor;
    y *= d_factor;
  }

  inline void print() const
  {
    std::cout << "  Distortion: Atan(" << s_ << ")" << std::endl;
  }

  const double s_;      // Radial distortion factor
  double s_inv_;
  double tans_;
  double tans_inv_;

};

} // namespace cameras
} // namespace vk
