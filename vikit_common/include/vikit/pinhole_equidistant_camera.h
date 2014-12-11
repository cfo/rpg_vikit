/*
 * pinhole_camera.h
 *
 *  Created on: Dec 11, 2014
 *      Author: cforster
 */

#ifndef VIKIT_PINHOLE_EQUIDISTANT_CAMERA_H_
#define VIKIT_PINHOLE_EQUIDISTANT_CAMERA_H_

#include <vikit/abstract_camera.h>

namespace vk {

/// Implements: "A Generic Camera Model and Calibration Method for Conventional,
/// Wide-Angle, and Fish-Eye Lenses" by Juho Kannala and Sami S. Brandt, PAMI'06
class PinholeEquidistantCamera : public AbstractCamera
{
private:
  const double fx_, fy_;
  const double cx_, cy_;
  const Eigen::Vector4d d_;           //!< distortion parameters

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PinholeEquidistantCamera(
      const double width, const double height,
      const double fx, const double fy,
      const double cx, const double cy,
      const double d0=0.0, const double d1=0.0,
      const double d2=0.0, const double d3=0.0,
      const std::string& cam_name = "cam0",
      const Sophus::SE3& T_imu_cam =
        Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));

  ~PinholeEquidistantCamera();

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
};

} // end namespace vk


#endif // VIKIT_PINHOLE_EQUIDISTANT_CAMERA_H_
