/*
 * pinhole_camera.cpp
 *
 *  Created on: Jul 24, 2012
 *      Author: cforster
 */

#include <iostream>
#include <vikit/pinhole_equidistant_camera.h>
#include <vikit/math_utils.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace vk {

using namespace Eigen;

PinholeEquidistantCamera::PinholeEquidistantCamera(
    const double width, const double height,
    const double fx, const double fy,
    const double cx, const double cy,
    const double d0, const double d1,
    const double d2, const double d3,
    const std::string& cam_name,
    const Sophus::SE3& T_imu_cam)
  : AbstractCamera(width, height, cam_name, T_imu_cam)
  , fx_(fx)
  , fy_(fy)
  , cx_(cx)
  , cy_(cy)
  , d_(d0, d1, d2, d3)
{}

PinholeEquidistantCamera::~PinholeEquidistantCamera()
{}

Vector3d PinholeEquidistantCamera::cam2world(const double& u, const double& v) const
{
  double x = (u - cx_) / fx_;
  double y = (v - cy_) / fy_;
  double theta, theta2, theta4, theta6, theta8;
  const double thetad = std::sqrt(x * x + y * y);
  theta = thetad; // initial guess
  for(int j = 0; j < 20; ++j)
  {
    theta2 = theta * theta;
    theta4 = theta2 * theta2;
    theta6 = theta4 * theta2;
    theta8 = theta4 * theta4;
    theta = thetad / (1 + d_[0] * theta2 + d_[1] * theta4 + d_[2] * theta6 + d_[3] * theta8);
  }
  const double scaling = std::tan(theta) / thetad;
  x *= scaling;
  y *= scaling;
  return Vector3d(x,y,1.0).normalized();
}

Vector3d PinholeEquidistantCamera::cam2world (const Vector2d& uv) const
{
  return cam2world(uv[0], uv[1]);
}

Vector2d PinholeEquidistantCamera::world2cam(const Vector3d& xyz) const
{
  return world2cam(project2d(xyz));
}

Vector2d PinholeEquidistantCamera::world2cam(const Vector2d& uv) const
{
  const double r = std::sqrt(uv[0]*uv[0] + uv[1]*uv[1]);
  const double theta = std::atan(r);
  const double theta2 = theta * theta;
  const double theta4 = theta2 * theta2;
  const double theta6 = theta4 * theta2;
  const double theta8 = theta4 * theta4;
  const double thetad =
      theta * (1.0 + d_[0] * theta2 + d_[1] * theta4 + d_[2] * theta6 + d_[3] * theta8);
  const double scaling = (r > 1e-8) ? thetad / r : 1.0;
  return Vector2d(uv[0]*scaling*fx_ + cx_, uv[1]*scaling*fy_ + cy_);
}

void PinholeEquidistantCamera::print(const std::string& s) const
{
  std::cout << s << std::endl
            << "  type = PinholeEquidistant " << std::endl
            << "  name = " << name_ << std::endl
            << "  size = [" << width_ << ", " << height_ << "]" << std::endl
            << "  focal_length =  [" << fx_ << ", " << fy_ << "]" << std::endl
            << "  center = [" << cx_ << ", " << cy_ << "]" << std::endl
            << "  distortion = [" << d_[0] << ", "
                                  << d_[1] << ", "
                                  << d_[2] << ", "
                                  << d_[3] << "]" << std::endl
            << "  T_cam_imu = [ tx: " << T_cam_imu_.translation().x()
                          << ", ty: " << T_cam_imu_.translation().y()
                          << ", tz: " << T_cam_imu_.translation().z()
                          << ", qx: " << T_cam_imu_.unit_quaternion().x()
                          << ", qy: " << T_cam_imu_.unit_quaternion().y()
                          << ", qz: " << T_cam_imu_.unit_quaternion().z()
                          << ", qw: " << T_cam_imu_.unit_quaternion().w()
                          << "]" << std::endl;
}

} // end namespace vk
