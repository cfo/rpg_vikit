#include <iostream>
#include <vikit/cameras/camera_geometry.h>

namespace vk {
namespace cameras {

template<typename Projection>
CameraGeometry<Projection>::CameraGeometry(
    const int width,
    const int height,
    const std::string& cam_name,
    const Sophus::SE3& T_body_cam,
    const projection_t& projection)
  : CameraGeometryBase(width, height, cam_name, T_body_cam)
  , projection_(projection)
{}

template<typename Projection>
Eigen::Vector3d CameraGeometry<Projection>::camToWorld(
    const Eigen::Vector2d& px) const
{
  return projection_.camToWorld(px);
}

template<typename Projection>
Eigen::Vector2d CameraGeometry<Projection>::worldToCam(const Eigen::Vector3d& xyz) const
{
  return projection_.worldToCam(xyz);
}

template<typename Projection>
void CameraGeometry<Projection>::print(const std::string& s) const
{
  std::cout << s << std::endl
            << "  name = " << name_ << std::endl
            << "  size = [" << width_ << ", " << height_ << "]" << std::endl;
  projection_.print();
  std::cout << "  T_cam_body = [ tx: " << T_cam_body_.translation().x()
                           << ", ty: " << T_cam_body_.translation().y()
                           << ", tz: " << T_cam_body_.translation().z()
                           << ", qx: " << T_cam_body_.unit_quaternion().x()
                           << ", qy: " << T_cam_body_.unit_quaternion().y()
                           << ", qz: " << T_cam_body_.unit_quaternion().z()
                           << ", qw: " << T_cam_body_.unit_quaternion().w()
                           << "]" << std::endl;
}

template<typename Projection>
double CameraGeometry<Projection>::errorMultiplier() const
{
  return projection_.errorMultiplier();
}

template<typename Projection>
template<typename T>
const T* const CameraGeometry<Projection>::projection() const
{
  return dynamic_cast<const T* const>(&projection_);
}

} // namespace cameras
} // namespace vk
