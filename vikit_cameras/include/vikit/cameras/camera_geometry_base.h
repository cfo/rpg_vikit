#pragma once

#include <string>
#include <memory>
#include <Eigen/Core>
#include <sophus/se3.h>

namespace vk {
namespace cameras {

class CameraGeometryBase
{
public:
  typedef std::shared_ptr<CameraGeometryBase> Ptr;

  CameraGeometryBase(
      const int width,
      const int height,
      const std::string& cam_name,
      const Sophus::SE3& T_body_cam)
    : width_(width), height_(height), name_(cam_name)
    , T_body_cam_(T_body_cam), T_cam_body_(T_body_cam.inverse())
  {}

  virtual ~CameraGeometryBase() = default;

  // Computes bearing vector from pixel coordinates. Z-component of the returned
  // bearing vector is 1. IMPORTANT: returned vector is NOT of unit length!
  virtual Eigen::Vector3d camToWorld(const Eigen::Vector2d& px) const = 0;

  // Computes pixel coordinates from bearing vector.
  virtual Eigen::Vector2d worldToCam(const Eigen::Vector3d& xyz) const = 0;

  // Width of the image in pixels.
  inline const int& width() const { return width_; }

  // Height of the image in pixels.
  inline const int& height() const { return height_; }

  // Name of the camera.
  inline const std::string& name() const { return name_; }

  // Get camera pose expressed in body coordinates.
  inline const Sophus::SE3& T_body_cam() const { return T_body_cam_; }

  // Get body pose expressed in camera coordinates.
  inline const Sophus::SE3& T_cam_body() const { return T_cam_body_; }

  // Check if a pixel is within the image boundaries.
  inline bool isInFrame(const Eigen::Vector2i& px, const int boundary=0) const
  {
    if(px[0] >= boundary && px[0] < width_-boundary
        && px[1] >= boundary && px[1] < height_-boundary)
      return true;
    return false;
  }

  // Check if a pixel is within image boundaries at specified pyramid level.
  inline bool isInFrame(const Eigen::Vector2i& px, const int boundary, const int level) const
  {
    if(px[0] >= boundary && px[0] < width_/(1<<level)-boundary
        && px[1] >= boundary && px[1] <height_/(1<<level)-boundary)
      return true;
    return false;
  }

  // Print camera info
  virtual void print(const std::string& s = "Camera: ") const = 0;

  // Equivalent to focal length for projective cameras and factor for
  // omni-directional cameras that transforms small angular error to pixel-error.
  virtual double errorMultiplier2() const = 0;

protected:
  const int width_;
  const int height_;
  const std::string name_;
  const Sophus::SE3 T_body_cam_; // Relative transformation between camera and body.
  const Sophus::SE3 T_cam_body_; // Relative transformation between body and camera.

};

} // namespace cameras
} // namespace vk
