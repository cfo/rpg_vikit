/*
 * pinhole_camera.cpp
 *
 *  Created on: Jul 24, 2012
 *      Author: cforster
 */

#include <iostream>
#include <vikit/pinhole_camera.h>
#include <vikit/math_utils.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace vk {

using namespace Eigen;

PinholeCamera::PinholeCamera(
    const double width, const double height,
    const double fx, const double fy,
    const double cx, const double cy,
    const double d0, const double d1, const double d2,
    const double d3, const double d4,
    const std::string& cam_name,
    const Sophus::SE3& T_imu_cam)
  : AbstractCamera(width, height, cam_name, T_imu_cam),
    fx_(fx), fy_(fy), cx_(cx), cy_(cy),
    distortion_(std::abs(d0) > 0.0000001),
    undist_map1_(height_, width_, CV_16SC2),
    undist_map2_(height_, width_, CV_16SC2)
{
  d_[0] = d0; d_[1] = d1; d_[2] = d2; d_[3] = d3; d_[4] = d4;
  cvK_ = (cv::Mat_<float>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
  cvD_ = (cv::Mat_<float>(1, 5) << d_[0], d_[1], d_[2], d_[3], d_[4]);
  cv::initUndistortRectifyMap(cvK_, cvD_, cv::Mat_<double>::eye(3,3), cvK_,
                              cv::Size(width_, height_), CV_16SC2, undist_map1_, undist_map2_);
  K_ << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0;
  K_inv_ = K_.inverse();
}

PinholeCamera::~PinholeCamera()
{}

Vector3d PinholeCamera::cam2world(const double& u, const double& v) const
{
  Vector3d xyz;
  if(!distortion_)
  {
    xyz[0] = (u - cx_)/fx_;
    xyz[1] = (v - cy_)/fy_;
    xyz[2] = 1.0;
  }
  else
  {
    double x0,x,y0,y;
    x0 = x = (u - cx_) / fx_;
    y0 = y = (v - cy_) / fy_;
    for(int j = 0; j < 5; j++ ) // copied from opencv/cvundistort.cpp : cv::undistortPoints()
    {
       const double r2 = x*x + y*y;
       const double icdist = 1.0/(1.0 + ((d_[4]*r2 + d_[1])*r2 + d_[0])*r2);
       const double deltaX = 2.0*d_[2]*x*y + d_[3]*(r2 + 2.0*x*x);
       const double deltaY = d_[2]*(r2 + 2.0*y*y) + 2.0*d_[3]*x*y;
       x = (x0 - deltaX)*icdist;
       y = (y0 - deltaY)*icdist;
    }
    xyz[0] = x;
    xyz[1] = y;
    xyz[2] = 1.0;
  }
  return xyz.normalized();
}

Vector3d PinholeCamera::cam2world (const Vector2d& uv) const
{
  return cam2world(uv[0], uv[1]);
}

Vector2d PinholeCamera::world2cam(const Vector3d& xyz) const
{
  return world2cam(project2d(xyz));
}

Vector2d PinholeCamera::world2cam(const Vector2d& uv) const
{
  Vector2d px;
  if(!distortion_)
  {
    px[0] = fx_*uv[0] + cx_;
    px[1] = fy_*uv[1] + cy_;
  }
  else
  {
    double x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;
    x = uv[0];
    y = uv[1];
    r2 = x*x + y*y;
    r4 = r2*r2;
    r6 = r4*r2;
    a1 = 2*x*y;
    a2 = r2 + 2*x*x;
    a3 = r2 + 2*y*y;
    cdist = 1 + d_[0]*r2 + d_[1]*r4 + d_[4]*r6;
    xd = x*cdist + d_[2]*a1 + d_[3]*a2;
    yd = y*cdist + d_[2]*a3 + d_[3]*a1;
    px[0] = xd*fx_ + cx_;
    px[1] = yd*fy_ + cy_;
  }
  return px;
}

void PinholeCamera::undistortImage(const cv::Mat& raw, cv::Mat& rectified)
{
  if(distortion_)
    cv::remap(raw, rectified, undist_map1_, undist_map2_, cv::INTER_LINEAR);
  else
    rectified = raw.clone();
}

void PinholeCamera::print(const std::string& s) const
{
  std::cout << s << std::endl
            << "  type = Pinhole " << std::endl
            << "  name = " << name_ << std::endl
            << "  size = [" << width_ << ", " << height_ << "]" << std::endl
            << "  focal_length =  [" << fx_ << ", " << fy_ << "]" << std::endl
            << "  center = [" << cx_ << ", " << cy_ << "]" << std::endl
            << "  distortion = [" << d_[0] << ", "
                                  << d_[1] << ", "
                                  << d_[2] << ", "
                                  << d_[3] << ", "
                                  << d_[4] << "]" << std::endl
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
