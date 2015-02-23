#include <iostream>
#include <vikit/cameras/pinhole_projection.h>

namespace vk {
namespace cameras {

template<typename Distortion>
PinholeProjection<Distortion>::PinholeProjection(
    double fx, double fy, double cx, double cy, distortion_t distortion)
  : fx_(fx), fy_(fy), fx_inv_(1.0/fx_), fy_inv_(1.0/fy_)
  , cx_(cx), cy_(cy), distortion_(distortion)
{}

template<typename Distortion>
Eigen::Vector3d PinholeProjection<Distortion>::camToWorld(
    const Eigen::Vector2d& px) const
{
  double x = (px[0]-cx_)*fx_inv_;
  double y = (px[1]-cy_)*fy_inv_;
  distortion_.undistort(x, y);
  return Eigen::Vector3d(x, y, 1.0);
}

template<typename Distortion>
Eigen::Vector2d PinholeProjection<Distortion>::worldToCam(
    const Eigen::Vector3d& xyz) const
{
  double u = xyz[0]/xyz[2];
  double v = xyz[1]/xyz[2];
  distortion_.distort(u, v);
  return Eigen::Vector2d(u*fx_+cx_, v*fy_+cy_);
}

template<typename Distortion>
double PinholeProjection<Distortion>::errorMultiplier() const
{
  return std::abs(fx_);
}

template<typename Distortion>
void PinholeProjection<Distortion>::print() const
{
  std::cout << "  Projection = Pinhole" << std::endl;
  std::cout << "  Focal length = (" << fx_ << ", " << fy_ << ")" << std::endl;
  std::cout << "  Principal point = (" << cx_ << ", " << cy_ << ")" << std::endl;
  distortion_.print();
}

} // namespace cameras
} // namespace vk
