#pragma once

#include <string>
#include <memory>
#include <Eigen/Core>
#include <kindr/minimal/quat-transformation.h>

namespace vk {

using Transformation = kindr::minimal::QuatTransformation;

namespace cameras {

/// \struct ProjectionResult
/// \brief This struct is returned by the camera projection methods and holds the result state
///        of the projection operation.
struct ProjectionResult {
  /// Possible projection state.
  enum class Status {
    /// Keypoint is visible and projection was successful.
    KEYPOINT_VISIBLE,
    /// Keypoint is NOT visible but projection was successful.
    KEYPOINT_OUTSIDE_IMAGE_BOX,
    /// The projected point lies behind the camera plane.
    POINT_BEHIND_CAMERA,
    /// The projection was unsuccessful.
    PROJECTION_INVALID,
    /// Default value after construction.
    UNINITIALIZED
  };
  // Make the enum values accessible from the outside without the additional indirection.
  static Status KEYPOINT_VISIBLE;
  static Status KEYPOINT_OUTSIDE_IMAGE_BOX;
  static Status POINT_BEHIND_CAMERA;
  static Status PROJECTION_INVALID;
  static Status UNINITIALIZED;

  constexpr ProjectionResult() : status_(Status::UNINITIALIZED) {};
  constexpr ProjectionResult(Status status) : status_(status) {};

  /// \brief ProjectionResult can be typecasted to bool and is true if the projected keypoint
  ///        is visible. Simplifies the check for a successful projection.
  ///        Example usage:
  /// @code
  ///          aslam::ProjectionResult ret = camera_->project3(Eigen::Vector3d(0, 0, -10), &keypoint);
  ///          if(ret) std::cout << "Projection was successful!\n";
  /// @endcode
  explicit operator bool() const { return isKeypointVisible(); };

  /// \brief Compare objects.
  bool operator==(const ProjectionResult& other) const { return status_ == other.status_; };

  /// \brief Compare projection status.
  bool operator==(const ProjectionResult::Status& other) const { return status_ == other; };

  /// \brief Convenience function to print the state using streams.
  friend std::ostream& operator<< (std::ostream& out, const ProjectionResult& state);

  /// \brief Check whether the projection was successful and the point is visible in the image.
  bool isKeypointVisible() const { return (status_ == Status::KEYPOINT_VISIBLE); };

  /// \brief Returns the exact state of the projection operation.
  ///        Example usage:
  /// @code
  ///          aslam::ProjectionResult ret = camera_->project3(Eigen::Vector3d(0, 0, -1), &keypoint);
  ///          if(ret.getDetailedStatus() == aslam::ProjectionResult::Status::KEYPOINT_OUTSIDE_IMAGE_BOX)
  ///            std::cout << "Point behind camera! Lets do something...\n";
  /// @endcode
  Status getDetailedStatus() const { return status_; };

 private:
  /// Stores the projection state.
  Status status_;
};

class CameraGeometryBase
{
public:
  typedef std::shared_ptr<CameraGeometryBase> Ptr;

  CameraGeometryBase(
      const int width,
      const int height,
      const std::string& cam_name,
      const Transformation& T_body_cam)
    : width_(width), height_(height), name_(cam_name)
    , T_body_cam_(T_body_cam), T_cam_body_(T_body_cam.inverse())
  {}

  virtual ~CameraGeometryBase() = default;

  // Computes bearing vector from pixel coordinates. Z-component of the returned
  // bearing vector is 1. IMPORTANT: returned vector is NOT of unit length!
  virtual Eigen::Vector3d camToWorld(const Eigen::Vector2d& px) const = 0;

  // Computes pixel coordinates from bearing vector.
  virtual Eigen::Vector2d worldToCam(const Eigen::Vector3d& xyz) const = 0;

  // Width of the image in pixels.
  inline const int& width() const { return width_; }

  // Height of the image in pixels.
  inline const int& height() const { return height_; }

  // Name of the camera.
  inline const std::string& name() const { return name_; }

  // Get camera pose expressed in body coordinates.
  inline const Transformation& T_body_cam() const { return T_body_cam_; }

  // Get body pose expressed in camera coordinates.
  inline const Transformation& T_cam_body() const { return T_cam_body_; }

  // Check if a pixel is within the image boundaries.
  inline bool isInFrame(const Eigen::Vector2i& px, const int boundary=0) const
  {
    return px[0] >= boundary
        && px[0] <  width_-boundary
        && px[1] >= boundary
        && px[1] <  height_-boundary;
  }

  // Check if a pixel is within image boundaries at specified pyramid level.
  inline bool isInFrame(const Eigen::Vector2i& px, const int boundary, const int level) const
  {
    return px[0] >= boundary
        && px[0] <  width_/(1<<level)-boundary
        && px[1] >= boundary
        && px[1] <  height_/(1<<level)-boundary;
  }

  // Print camera info
  virtual void print(const std::string& s = "Camera: ") const = 0;

  // Equivalent to focal length for projective cameras and factor for
  // omni-directional cameras that transforms small angular error to pixel-error.
  virtual double errorMultiplier() const = 0;

  // Set user-specific camera index.
  inline void setCamIndex(size_t index) { cam_index_ = index; }

  // Get user-specific camera index.
  inline size_t camIndex() const { return cam_index_; }

  /// Helper for transition to aslam_cv2. Replaces worldToCam().
  const ProjectionResult project3(
      const Eigen::Ref<const Eigen::Vector3d>& point_3d,
      Eigen::Vector2d* out_keypoint) const
  {
    *out_keypoint = worldToCam(point_3d);
    if(!isInFrame(out_keypoint->cast<int>()))
      return ProjectionResult::Status::KEYPOINT_OUTSIDE_IMAGE_BOX;
    return ProjectionResult::Status::KEYPOINT_VISIBLE;
  }

  /// Helper for transition to aslam_cv2. Replaces worldToCam().
  virtual bool backProject3(
      const Eigen::Ref<const Eigen::Vector2d>& keypoint,
      Eigen::Vector3d* out_point_3d) const
  {
    *out_point_3d = camToWorld(keypoint);
    return true;
  }

  /// Helper for transition to aslam_cv2.
  uint32_t imageWidth() const { return width_; }

  /// Helper for transition to aslam_cv2.
  uint32_t imageHeight() const { return height_; }

  /// Helper for transition to aslam_cv2.
  inline bool isKeypointVisible(const Eigen::Ref<const Eigen::Vector2d>& keypoint) const
  {
    return keypoint[0] >= 0.0
        && keypoint[1] >= 0.0
        && keypoint[0] < imageWidth()
        && keypoint[1] < imageHeight();
  }

  /// Helper for transition to aslam_cv2.
  inline bool isKeypointWithinBoundary(const Eigen::Ref<const Eigen::Vector2d>& keypoint, double boundary) const
  {
    return keypoint[0] >= boundary
        && keypoint[1] >= boundary
        && keypoint[0] < imageWidth()-boundary
        && keypoint[1] < imageHeight()-boundary;
  }

protected:
  const int width_;
  const int height_;
  const std::string name_;
  const Transformation T_body_cam_; // Relative transformation between camera and body.
  const Transformation T_cam_body_; // Relative transformation between body and camera.
  size_t cam_index_ = 0;         // Index of camera in camera array.
};

} // namespace cameras
} // namespace vk
