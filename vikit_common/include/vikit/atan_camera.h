/*
 * atan_camera.h
 *
 *  Created on: Aug 21, 2012
 *      Author: cforster
 *
 *  This class implements the FOV distortion model of Deverneay and Faugeras,
 *  Straight lines have to be straight, 2001.
 *
 *  The code is an implementation of the ATAN class in PTAM by Georg Klein using Eigen.
 */

#ifndef VIKIT_ATAN_CAMERA_H_
#define VIKIT_ATAN_CAMERA_H_

#include <cmath>
#include <vikit/abstract_camera.h>

namespace vk {

class ATANCamera : public AbstractCamera {

private:
  double fx_, fy_;                      //!< focal length
  double fx_inv_, fy_inv_;              //!< inverse focal length
  double cx_, cy_;                      //!< projection center
  double s_, s_inv_;                    //!< distortion model coeff
  double tans_;                         //!< distortion model coeff
  double tans_inv_;                     //!< distortion model coeff
  bool distortion_;                     //!< use distortion model?

  //! Radial distortion transformation factor: returns ration of distorted / undistorted radius.
  inline double rtrans_factor(const double r) const
  {
    if(r < 0.001 || s_ == 0.0)
      return 1.0;
    else
      return (s_inv_* std::atan(r * tans_) / r);
  }

  //! Inverse radial distortion: returns un-distorted radius from distorted.
  inline double invrtrans(const double r) const
  {
    if(s_ == 0.0)
      return r;
    return (std::tan(r * s_) * tans_inv_);
  }

public:

  ATANCamera(
      const double width,
      const double height,
      const double fx,
      const double fy,
      const double dx,
      const double dy,
      const double s,
      const Sophus::SE3& T_imu_cam =
        Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));

  virtual ~ATANCamera();

  virtual Eigen::Vector3d cam2world(const double& x, const double& y) const;

  virtual Eigen::Vector3d cam2world(const Eigen::Vector2d& px) const;

  virtual Eigen::Vector2d world2cam(const Eigen::Vector3d& xyz_c) const;

  virtual Eigen::Vector2d world2cam(const Eigen::Vector2d& uv) const;

  const Eigen::Vector2d focal_length() const
  {
    return Eigen::Vector2d(fx_, fy_);
  }

  virtual double errorMultiplier2() const
  {
    return fx_;
  }

  virtual double errorMultiplier() const
  {
    return 4.0*fx_*fy_;
  }
};

} // end namespace vk

#endif // VIKIT_ATAN_CAMERA_H_
