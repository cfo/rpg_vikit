#include <stdexcept>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/omni_camera.h>

namespace vk {

AbstractCamera::AbstractCamera(
    const int width,
    const int height,
    const Sophus::SE3& T_imu_cam)
  : width_(width)
  , height_(height)
  , T_imu_cam_(T_imu_cam)
  , T_cam_imu_(T_imu_cam.inverse())
{}

AbstractCamera::~AbstractCamera()
{}

AbstractCamera::Ptr AbstractCamera::loadCameraFromYamlFile(
    const std::string& filename,
    const bool verbose)
{
#ifdef VIKIT_USE_YAML
  // TODO
#else
  throw std::runtime_error("Can't load camera because vikit is not compiled with YAML");
#endif
  return nullptr;
}

} // namespace vk
