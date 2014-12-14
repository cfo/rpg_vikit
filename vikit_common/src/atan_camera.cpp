/*
 * atan_camera.cpp
 *
 *  Created on: Aug 21, 2012
 *      Author: cforster
 */


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vikit/atan_camera.h>
#include <vikit/math_utils.h>

namespace vk {

using namespace Eigen;

ATANCamera::ATANCamera(
    const double width, const double height,
    const double fx, const double fy,
    const double cx, const double cy,
    const double s,
    const std::string& cam_name,
    const Sophus::SE3& T_imu_cam)
  : AbstractCamera(width, height, cam_name, T_imu_cam)
  , fx_(width*fx)
  , fy_(height*fy)
  , fx_inv_(1.0/fx_)
  , fy_inv_(1.0/fy_)
  , cx_(cx*width - 0.5)
  , cy_(cy*height - 0.5)
  , s_(s)
  , s_inv_(1.0/s_)
{
  if(s_ != 0.0)
  {
    tans_ = 2.0 * std::tan(s_ / 2.0);
    tans_inv_ = 1.0 / tans_;
    s_inv_ = 1.0 / s_;
    distortion_ = true;
  }
  else
  {
    s_inv_ = 0.0;
    tans_ = 0.0;
    distortion_ = false;
  }
}

ATANCamera::~ATANCamera()
{}

Vector3d ATANCamera::cam2world(const double& x, const double& y) const
{
  const Vector2d dist_cam((x - cx_) * fx_inv_,
                          (y - cy_) * fy_inv_);
  const double dist_r = dist_cam.norm();
  const double r = invrtrans(dist_r);
  double d_factor;
  if(dist_r > 0.01)
    d_factor =  r / dist_r;
  else
    d_factor = 1.0;
  return unproject2d(d_factor * dist_cam).normalized();
}

Vector3d ATANCamera::cam2world (const Vector2d& px) const
{
  return cam2world(px[0], px[1]);
}

Vector2d ATANCamera::world2cam(const Vector3d& xyz_c) const
{
  return world2cam(project2d(xyz_c));
}

Vector2d ATANCamera::world2cam(const Vector2d& uv) const
{
  const double r = uv.norm();
  const double factor = rtrans_factor(r);
  const Vector2d dist_cam = factor * uv;
  return Vector2d(cx_ + fx_ * dist_cam[0],
                  cy_ + fy_ * dist_cam[1]);
}

void ATANCamera::print(const std::string& s) const
{
  std::cout << s << std::endl
            << "  type = ATAN " << std::endl
            << "  name = " << name_ << std::endl
            << "  size = [" << width_ << ", " << height_ << "]" << std::endl
            << "  focal_length =  [" << fx_ << ", " << fy_ << "]" << std::endl
            << "  center = [" << cx_ << ", " << cy_ << "]" << std::endl
            << "  distortion = " << s_ << std::endl
            << "  T_cam_imu = [ tx: " << T_cam_imu_.translation().x()
                          << ", ty: " << T_cam_imu_.translation().y()
                          << ", tz: " << T_cam_imu_.translation().z()
                          << ", qx: " << T_cam_imu_.unit_quaternion().x()
                          << ", qy: " << T_cam_imu_.unit_quaternion().y()
                          << ", qz: " << T_cam_imu_.unit_quaternion().z()
                          << ", qw: " << T_cam_imu_.unit_quaternion().w()
                          << "]" << std::endl;
}

} // namespace vk
