/*
 * OcamProjector.h
 *
 *  Created on: Sep 22, 2010
 *      Author: laurent kneip
 */

#ifndef VIKIT_OCAMPROJECTOR_H_
#define VIKIT_OCAMPROJECTOR_H_

#include <vikit/abstract_camera.h>
#include <vikit/math_utils.h>

#define CMV_MAX_BUF 1024
#define MAX_POL_LENGTH 64

namespace vk {

struct ocam_model
{
  double pol[MAX_POL_LENGTH];    	// the polynomial coefficients: pol[0] + x"pol[1] + x^2*pol[2] + ... + x^(N-1)*pol[N-1]
  int length_pol;                	// length of polynomial
  double invpol[MAX_POL_LENGTH]; 	// the coefficients of the inverse polynomial
  int length_invpol;             	// length of inverse polynomial
  double xc;				// row coordinate of the center
  double yc;         			// column coordinate of the center
  double c;				// affine parameter
  double d;				// affine parameter
  double e;				// affine parameter
  int width;				// image width
  int height;				// image height
};

class OmniCamera : public AbstractCamera
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OmniCamera(
      const int width,
      const int height,
      const std::string& calibfile,
      const std::string& cam_name = "cam0",
      const Sophus::SE3& T_imu_cam =
        Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));

  virtual ~OmniCamera();

  /// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
  virtual Eigen::Vector3d cam2world(const double& x, const double& y) const;

  /// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
  virtual Eigen::Vector3d cam2world(const Vector2d& px) const;

  /// Project from world coordinates to camera coordinates.
  virtual Eigen::Vector2d world2cam(const Vector3d& xyz_c) const;

  /// Projects unit plane coordinates to camera coordinates.
  virtual Eigen::Vector2d world2cam(const Vector2d& uv) const;

  /// Print camera info.
  virtual void print(const std::string& s = "Camera: ") const;

  virtual double errorMultiplier2() const
  {
    return sqrt(error_multiplier_)/2;
  }

  virtual double errorMultiplier() const
  {
    return error_multiplier_;
  }

private:
  double error_multiplier_;
  struct ocam_model ocamModel;
  double computeErrorMultiplier();

};

} // end namespace vk

#endif // VIKIT_OCAMPROJECTOR_H_
