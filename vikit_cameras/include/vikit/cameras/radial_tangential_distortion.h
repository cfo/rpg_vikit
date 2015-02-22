#pragma once

#include <iostream>

namespace vk {
namespace cameras {

// This class implements the radial and tangential distortion model used by
// OpenCV and ROS. Reference:
// docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
class RadialTangentialDistortion 
{
public:

  RadialTangentialDistortion(
      const double& k1, const double& k2, const double& p1, const double& p2)
    : k1_(k1), k2_(k2), p1_(p1), p2_(p2)
  {}

	~RadialTangentialDistortion() = default;

	inline void distort(double& x, double& y) const
  {
    const double xx = x * x;
    const double yy = y * y;
    const double xy = x * y;
    const double xy2 = 2.0*xy;
    const double r2 = xx + yy;
    const double cdist = (k1_ + k2_ * r2) * r2;
    x += x * cdist + p1_ * xy2 + p2_ * (r2 + 2.0 * xx);
    y += y * cdist + p2_ * xy2 + p1_ * (r2 + 2.0 * yy);
	}

  inline void undistort(double& x, double& y) const
  {
    double x0=x, y0=y;
    for(int i = 0; i < 5; ++i)
    {
      const double xx = x*x;
      const double yy = y*y;
      const double xy = x*y;
      const double xy2 = 2*xy;
      const double r2 = xx + yy;
      const double icdist = 1.0/(1.0 + (k1_ + k2_ * r2) * r2);
      const double dx = p1_ * xy2 + p2_ * (r2 + 2.0 * xx);
      const double dy = p2_ * xy2 + p1_ * (r2 + 2.0 * yy);
      x = (x0 - dx)*icdist;
      y = (y0 - dy)*icdist;
    }
  }

	inline void print() const 
	{
    std::cout << "  Distortion: RadTan("
              << k1_ << ", " << k2_ << ", " << p1_ << ", " << p2_ << ")"
              << std::endl;
	}

  double k1_ = 0; // Radial distortion factor 1
  double k2_ = 0; // Radial distortion factor 2
  double p1_ = 0; // Tangential distortion factor 1
  double p2_ = 0; // Tangential distortion factor 2
};

} // namespace cameras
} // namespace vk
