/*
 * pinhole_camera.h
 *
 *  Created on: Jul 24, 2012
 *      Author: cforster
 */

#ifndef VIKIT_PINHOLE_CAMERA_H_
#define VIKIT_PINHOLE_CAMERA_H_

#include <cmath>
#include <vikit/abstract_camera.h>
#include <opencv2/core/core.hpp>

namespace vk {

class PinholeCamera : public AbstractCamera
{
private:
  const double fx_, fy_;
  const double cx_, cy_;
  bool distortion_;             //!< is it pure pinhole model or has it radial distortion?
  double d_[5];                 //!< distortion parameters, see http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
  cv::Mat cvK_, cvD_;
  cv::Mat undist_map1_, undist_map2_;
  Eigen::Matrix3d K_;
  Eigen::Matrix3d K_inv_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PinholeCamera(
      const double width, const double height,
      const double fx, const double fy,
      const double cx, const double cy,
      const double d0=0.0, const double d1=0.0, const double d2=0.0,
      const double d3=0.0, const double d4=0.0,
      const std::string& cam_name = "cam0",
      const Sophus::SE3& T_imu_cam =
        Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));

  ~PinholeCamera();

  void initUnistortionMap();

  /// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
  virtual Eigen::Vector3d cam2world(const double& x, const double& y) const;

  /// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
  virtual Eigen::Vector3d cam2world(const Eigen::Vector2d& px) const;

  /// Project from world coordinates to camera coordinates.
  virtual Eigen::Vector2d world2cam(const Eigen::Vector3d& xyz_c) const;

  /// Projects unit plane coordinates to camera coordinates.
  virtual Eigen::Vector2d world2cam(const Eigen::Vector2d& uv) const;

  const Eigen::Vector2d focal_length() const
  {
    return Eigen::Vector2d(fx_, fy_);
  }

  virtual double errorMultiplier2() const
  {
    return std::abs(fx_);
  }

  virtual double errorMultiplier() const
  {
    return std::abs(4.0*fx_*fy_);
  }

  /// Print camera info.
  virtual void print(const std::string& s = "Camera: ") const;

  inline const Eigen::Matrix3d& K() const { return K_; }
  inline const Eigen::Matrix3d& K_inv() const { return K_inv_; }
  inline double fx() const { return fx_; }
  inline double fy() const { return fy_; }
  inline double cx() const { return cx_; }
  inline double cy() const { return cy_; }
  inline double d0() const { return d_[0]; }
  inline double d1() const { return d_[1]; }
  inline double d2() const { return d_[2]; }
  inline double d3() const { return d_[3]; }
  inline double d4() const { return d_[4]; }

  void undistortImage(const cv::Mat& raw, cv::Mat& rectified);

};

} // end namespace vk


#endif // VIKIT_PINHOLE_CAMERA_H_
