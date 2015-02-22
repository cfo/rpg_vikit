#pragma once

#include <vikit/cameras/no_distortion.h>
#include <Eigen/Core>

namespace vk {
namespace cameras {

template<typename Distortion>
class PinholeProjection
{
public:
  typedef Distortion distortion_t;

  PinholeProjection() = default;

  PinholeProjection(
      double fx, double fy, double cx, double cy, distortion_t distortion);

  ~PinholeProjection() = default;

  // Computes bearing vector from pixel coordinates. Z-component of the returned
  // bearing vector is 1. IMPORTANT: returned vector is NOT of unit length!
  Eigen::Vector3d camToWorld(const Eigen::Vector2d& px) const;

  // Computes pixel coordinates from bearing vector.
  Eigen::Vector2d worldToCam(const Eigen::Vector3d& xyz) const;

  // Returns focal length (transforms unit plane error to pixel error).
  double errorMultiplier() const;

  void print() const;

  double fx_ = 1; // Focal length x.
  double fy_ = 1; // Focal length y.
  double fx_inv_ = 1; // Inverse focal length x
  double fy_inv_ = 1; // Inverse focal length y
  double cx_ = 0; // Principle point x.
  double cy_ = 0; // Principle point y.
  distortion_t distortion_;
};

} // namespace cameras
} // namespace vk

#include <vikit/cameras/implementation/pinhole_projection.hpp>
