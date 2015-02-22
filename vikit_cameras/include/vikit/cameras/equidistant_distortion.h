#pragma once

#include <cmath>
#include <iostream>

namespace vk {
namespace cameras {

// This class implements the distortion model described in the paper:
// "A Generic Camera Model and Calibration Method for Conventional, Wide-Angle,
// and Fish-Eye Lenses" by Juho Kannala and Sami S. Brandt, PAMI.
class EquidistantDistortion
{
public:

  EquidistantDistortion(
      const double& k1, const double& k2, const double& k3, const double& k4)
    : k1_(k1), k2_(k2), k3_(k3), k4_(k4)
  {}

  ~EquidistantDistortion() = default;

  inline void distort(double& x, double& y) const
  {
    const double r = std::sqrt(x*x + y*y);
    const double theta = std::atan(r);
    const double theta2 = theta * theta;
    const double theta4 = theta2 * theta2;
    const double theta6 = theta4 * theta2;
    const double theta8 = theta4 * theta4;
    const double thetad =
        theta*(1.0 + k1_*theta2 + k2_*theta4 + k3_*theta6 + k4_*theta8);
    const double scaling = (r > 1e-8) ? thetad / r : 1.0;
    x *= scaling;
    y *= scaling;
  }

  inline void undistort(double& x, double& y) const
  {
    const double thetad = std::sqrt(x*x + y*y);
    double theta = thetad;
    for(int i = 0; i < 5; ++i)
    {
      const double theta2 = theta * theta;
      const double theta4 = theta2 * theta2;
      const double theta6 = theta4 * theta2;
      const double theta8 = theta4 * theta4;
      theta = thetad / (1.0 + k1_*theta2 + k2_*theta4 + k3_*theta6 + k4_*theta8);
    }
    const double scaling = std::tan(theta) / thetad;
    x *= scaling;
    y *= scaling;
  }

  inline void print() const
  {
    std::cout << "  Distortion: Equidistant("
              << k1_ << ", " << k2_ << ", " << k3_ << ", " << k4_ << ")"
              << std::endl;
  }

  double k1_ = 0; // Radial distortion factor 1
  double k2_ = 0; // Radial distortion factor 2
  double k3_ = 0; // Radial distortion factor 3
  double k4_ = 0; // Radial distortion factor 4
};


} // namespace cameras
} // namespace vk
