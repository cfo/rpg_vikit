#include <stdexcept>
#include <iostream>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/omni_camera.h>
#include <yaml-cpp/yaml.h>

namespace vk {

AbstractCamera::AbstractCamera(
    const int width,
    const int height,
    const std::string& cam_name,
    const Sophus::SE3& T_imu_cam)
  : width_(width)
  , height_(height)
  , name_(cam_name)
  , T_imu_cam_(T_imu_cam)
  , T_cam_imu_(T_imu_cam.inverse())
{}

AbstractCamera::~AbstractCamera()
{}

AbstractCamera::Ptr AbstractCamera::loadCameraFromYamlFile(
    const std::string& filename,
    const std::string& cam_name,
    const bool verbose)
{
#ifdef VIKIT_USE_YAML
  if(verbose)
    std::cout << "Camera: Load from YAML file: " << filename << std::endl;

  YAML::Node data = YAML::LoadFile(filename);
  if(!data)
  {
    std::cerr << "Could not load camera from file: " << filename << std::endl;
    return nullptr;
  }

  // The YAML File may contain multiple cameras, specify which one to load
  data = data[cam_name];
  if(!data)
  {
    std::cerr << "Camera with name ' "<< cam_name << "' does not exist in "
              << "file: " << filename << std::endl;
    return nullptr;
  }

  // load imu camera transformation
  Sophus::SE3 T_cam_imu(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  if(data["T_cam_imu"].IsDefined())
  {
    T_cam_imu = SE3(
        Quaterniond(
            data["T_cam_imu"]["qw"].as<double>(),
            data["T_cam_imu"]["qx"].as<double>(),
            data["T_cam_imu"]["qy"].as<double>(),
            data["T_cam_imu"]["qz"].as<double>()),
        Vector3d(data["T_cam_imu"]["tx"].as<double>(),
            data["T_cam_imu"]["ty"].as<double>(),
            data["T_cam_imu"]["tz"].as<double>()));
    if(verbose)
      std::cout << "T_imu_cam = " << std::endl
                << T_cam_imu.inverse().matrix() << std::endl
                << "t_cam_imu = " << T_cam_imu.translation().transpose() << std::endl
                << "R_cam_imu = " << std::endl
                << T_cam_imu.rotation_matrix() << std::endl;
  }
  else
  {
    if(verbose)
      std::cout << "Camera-IMU Transformation is not specified." << std::endl;
  }

  // load camera
  AbstractCamera::Ptr cam;
  std::string cam_model = data["cam_model"].as<std::string>();
  if(cam_model == "Pinhole")
  {
    cam.reset(new vk::PinholeCamera(
                data["cam_width"].as<int>(),
                data["cam_height"].as<int>(),
                data["cam_fx"].as<double>(),
                data["cam_fy"].as<double>(),
                data["cam_cx"].as<double>(),
                data["cam_cy"].as<double>(),
                data["cam_d0"].as<double>(),
                data["cam_d1"].as<double>(),
                data["cam_d2"].as<double>(),
                data["cam_d3"].as<double>(),
                data["cam_d4"].as<double>(),
                cam_name,
                T_cam_imu.inverse()));
  }
  else if(cam_model == "ATAN")
  {
    cam.reset(new vk::ATANCamera(
                data["cam_width"].as<int>(),
                data["cam_height"].as<int>(),
                data["cam_fx"].as<double>(),
                data["cam_fy"].as<double>(),
                data["cam_cx"].as<double>(),
                data["cam_cy"].as<double>(),
                data["cam_d0"].as<double>(),
                cam_name,
                T_cam_imu.inverse()));
  }
  else if(cam_model == "OCam")
  {
    cam.reset(new vk::OmniCamera(
                data["cam_width"].as<int>(),
                data["cam_height"].as<int>(),
                data["cam_calib_file"].as<std::string>(),
                cam_name,
                T_cam_imu.inverse()));
  }
  else
  {
    std::cerr << "Camera model '"<< cam_model << "'dot exist." << std::endl;
    return nullptr;
  }

  return cam;

#else
  throw std::runtime_error("Can't load camera because vikit is not compiled with YAML");
#endif
}

} // namespace vk
