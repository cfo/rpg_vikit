#pragma once

#include <string>
#include <memory>
#include <Eigen/Core>
#include <sophus/se3.h>
#include <vikit/cameras/camera_geometry_base.h>

namespace vk {
namespace cameras {

template<typename Projection>
class CameraGeometry : public CameraGeometryBase
{
public:
  typedef Projection projection_t;

  CameraGeometry(
      const int width,
      const int height,
      const std::string& cam_name,
      const Sophus::SE3& T_body_cam,
      const projection_t& projection);

  virtual ~CameraGeometry() = default;

  virtual Eigen::Vector3d camToWorld(const Eigen::Vector2d& px) const override;

  virtual Eigen::Vector2d worldToCam(const Eigen::Vector3d& xyz) const override;

  virtual void print(const std::string& s = "Camera: ") const override;

  virtual double errorMultiplier2() const override;

private:
  projection_t projection_;
};

} // namespace cameras
} // namespace vk

#include <vikit/cameras/implementation/camera_geometry.hpp>
