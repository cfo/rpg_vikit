/*
 * abstract_camera.h
 *
 *  Created on: Jul 23, 2012
 *      Author: cforster
 */

#ifndef VIKIT_ABSTRACT_CAMERA_H_
#define VIKIT_ABSTRACT_CAMERA_H_

#include <Eigen/Core>
#include <sophus/se3.h>
#include <memory> // std::shared_ptr

namespace vk {

class AbstractCamera
{
public:

  typedef std::shared_ptr<AbstractCamera> Ptr;

  const int width_;               ///< Width of the image in pixels.
  const int height_;              ///< Height of the image in pixels.
  const std::string name_;        ///< Camera name "e.g. forward_facing, cam0"
  const Sophus::SE3 T_imu_cam_;   ///< Relative transformation between camera and Inertial Measurement Unit (IMU).
  const Sophus::SE3 T_cam_imu_;   ///< Relative transformation between IMU and camera.

  /// Default constructor.
  AbstractCamera(
      const int width,
      const int height,
      const std::string& cam_name,
      const Sophus::SE3& T_imu_cam);

  /// Destructor.
  virtual ~AbstractCamera();

  /// Camera Factory: Load camera from file.
  static AbstractCamera::Ptr loadCameraFromYamlFile(
      const std::string& filename,
      const std::string& cam_name,
      const bool verbose);

  /// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
  virtual Eigen::Vector3d cam2world(const double& x, const double& y) const = 0;

  /// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
  virtual Eigen::Vector3d cam2world(const Eigen::Vector2d& px) const = 0;

  /// Project from world coordinates to camera coordinates.
  virtual Eigen::Vector2d world2cam(const Eigen::Vector3d& xyz_c) const = 0;

  /// Projects unit plane coordinates to camera coordinates.
  virtual Eigen::Vector2d world2cam(const Eigen::Vector2d& uv) const = 0;

  /// Equivalent to focal length for projective cameras and factor for
  /// omni-directional cameras that transforms unit-plane error to pixel-error.
  virtual double errorMultiplier2() const = 0;

  /// Square root of errorMultiplier2, used to transform angular error in
  /// omni-directional cameras to pixel error.
  virtual double errorMultiplier() const = 0;

  /// Print camera info.
  virtual void print(const std::string& s = "Camera: ") const = 0;

  /// Width of the image in pixels.
  inline const int& width() const { return width_; }

  /// Height of the image in pixels.
  inline const int& height() const { return height_; }

  /// Name of the camera
  inline const std::string& name() const { return name_; }

  /// Check if a pixel is within the image boundaries
  inline bool isInFrame(const Eigen::Vector2i& px, const int boundary=0) const
  {
    if(px[0] >= boundary && px[0] < width_-boundary
        && px[1] >= boundary && px[1] < height_-boundary)
      return true;
    return false;
  }

  /// Check if a pixel is within image boundaries at specified pyramid level
  inline bool isInFrame(const Eigen::Vector2i& px, const int boundary, const int level) const
  {
    if(px[0] >= boundary && px[0] < width_/(1<<level)-boundary
        && px[1] >= boundary && px[1] <height_/(1<<level)-boundary)
      return true;
    return false;
  }
};

} // end namespace vikit

#endif // VIKIT_ABSTRACT_CAMERA_H_
