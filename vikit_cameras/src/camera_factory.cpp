#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <vikit/cameras.h>
#include <vikit/cameras/camera_factory.h>

namespace vk {
namespace cameras {
namespace factory {

CameraGeometryBase::Ptr loadFromYAML(
    const std::string& filename,
    const std::string& cam_name)
{
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
  Sophus::SE3 T_imu_cam(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  if(data["T_imu_cam"].IsDefined())
  {
    T_imu_cam = Sophus::SE3(
        Eigen::Quaterniond(
            data["T_imu_cam"]["qw"].as<double>(),
            data["T_imu_cam"]["qx"].as<double>(),
            data["T_imu_cam"]["qy"].as<double>(),
            data["T_imu_cam"]["qz"].as<double>()),
        Eigen::Vector3d(
            data["T_imu_cam"]["tx"].as<double>(),
            data["T_imu_cam"]["ty"].as<double>(),
            data["T_imu_cam"]["tz"].as<double>()));
  }
  else if(data["T_cam_imu"].IsDefined())
  {
    T_imu_cam = Sophus::SE3(
        Eigen::Quaterniond(
            data["T_cam_imu"]["qw"].as<double>(),
            data["T_cam_imu"]["qx"].as<double>(),
            data["T_cam_imu"]["qy"].as<double>(),
            data["T_cam_imu"]["qz"].as<double>()),
        Eigen::Vector3d(
            data["T_cam_imu"]["tx"].as<double>(),
            data["T_cam_imu"]["ty"].as<double>(),
            data["T_cam_imu"]["tz"].as<double>())).inverse();
  }

  // load camera
  CameraGeometryBase::Ptr cam;
  std::string cam_model = data["cam_model"].as<std::string>();
  if(cam_model == "PinholeRadTan")
  {
    double d0 = data["cam_d0"].IsDefined() ? data["cam_d0"].as<double>() : 0.0;
    double d1 = data["cam_d1"].IsDefined() ? data["cam_d1"].as<double>() : 0.0;
    double d2 = data["cam_d2"].IsDefined() ? data["cam_d2"].as<double>() : 0.0;
    double d3 = data["cam_d3"].IsDefined() ? data["cam_d3"].as<double>() : 0.0;
    if(d0 == 0 && d1 == 0 && d2 == 0 && d3 == 0)
    {
      cam_model = "Pinhole";
    }
    else
    {
      cam.reset(new PinholeRadTanGeometry(
                  data["cam_width"].as<int>(),
                  data["cam_height"].as<int>(),
                  cam_name,
                  T_imu_cam,
                  PinholeProjection<RadialTangentialDistortion>(
                    data["cam_fx"].as<double>(),
                    data["cam_fy"].as<double>(),
                    data["cam_cx"].as<double>(),
                    data["cam_cy"].as<double>(),
                    RadialTangentialDistortion(d0, d1, d2, d3))));
    }
  }
  if(cam_model == "Pinhole")
  {
    cam.reset(new PinholeGeometry(
                data["cam_width"].as<int>(),
                data["cam_height"].as<int>(),
                cam_name,
                T_imu_cam,
                PinholeProjection<NoDistortion>(
                  data["cam_fx"].as<double>(),
                  data["cam_fy"].as<double>(),
                  data["cam_cx"].as<double>(),
                  data["cam_cy"].as<double>(),
                  NoDistortion())));
  }
  else if(cam_model == "PinholeEquidistant")
  {
    cam.reset(new PinholeEquidistantGeometry(
                data["cam_width"].as<int>(),
                data["cam_height"].as<int>(),
                cam_name,
                T_imu_cam,
                PinholeProjection<EquidistantDistortion>(
                  data["cam_fx"].as<double>(),
                  data["cam_fy"].as<double>(),
                  data["cam_cx"].as<double>(),
                  data["cam_cy"].as<double>(),
                  EquidistantDistortion(
                    data["cam_d0"].as<double>(),
                    data["cam_d1"].as<double>(),
                    data["cam_d2"].as<double>(),
                    data["cam_d3"].as<double>()))));
  }
  else if(cam_model == "ATAN")
  {
    cam.reset(new PinholeAtanGeometry(
                data["cam_width"].as<int>(),
                data["cam_height"].as<int>(),
                cam_name,
                T_imu_cam,
                PinholeProjection<AtanDistortion>(
                  data["cam_fx"].as<double>(),
                  data["cam_fy"].as<double>(),
                  data["cam_cx"].as<double>(),
                  data["cam_cy"].as<double>(),
                  AtanDistortion(
                    data["cam_d0"].as<double>()))));
  }
  else if(cam_model == "OCam")
  {
    std::cerr << "OCam model not yet implemented." << std::endl;
    return nullptr;
  }
  else if(cam_model != "PinholeRadTan")
  {
    std::cerr << "Camera model '"<< cam_model << "' doesn't exist." << std::endl;
    return nullptr;
  }

  return cam;
}

} // namespace factory
} // namespace cameras
} // namespace vk
