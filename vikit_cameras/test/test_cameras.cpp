#include <gtest/gtest.h>

#include <vikit/cameras/pinhole_projection.h>
#include <vikit/cameras/camera_geometry.h>

// distortion models
#include <vikit/cameras/no_distortion.h>
#include <vikit/cameras/atan_distortion.h>
#include <vikit/cameras/equidistant_distortion.h>
#include <vikit/cameras/radial_tangential_distortion.h>


namespace {

TEST(CamerasTest, AtanDistortion)
{
  double x = 0.5, y = 0.8;
  vk::cameras::AtanDistortion distortion_model(0.934479);
  distortion_model.distort(x, y);
  distortion_model.undistort(x, y);
  EXPECT_NEAR(x, 0.5, 1e-10);
  EXPECT_NEAR(y, 0.8, 1e-10);
}

TEST(CamerasTest, EquidistantDistortion)
{
  double x = 0.5, y = 0.8;
  vk::cameras::EquidistantDistortion distortion_model(
        -0.0027, 0.0241, -0.0430, 0.0311);
  distortion_model.distort(x, y);
  std::cout << "x = " << x << ", y = " << y << std::endl;
  distortion_model.undistort(x, y);
  EXPECT_NEAR(x, 0.5, 1e-10);
  EXPECT_NEAR(y, 0.8, 1e-10);
}

TEST(CamerasTest, RadialTangentialDistortion)
{
  double x = 0.5, y = 0.8;
  vk::cameras::RadialTangentialDistortion distortion_model(
        -0.3, 0.1, 9.52e-05, -0.00057);
  distortion_model.distort(x, y);
  distortion_model.undistort(x, y);
  // NOTE: with radtan we don't achieve same precision with only five iterations
  // at this pose (0.5, 0.8) far from image center
  EXPECT_NEAR(x, 0.5, 1e-2);
  EXPECT_NEAR(y, 0.8, 1e-2);
}

TEST(CamerasTest, PinholeConstructor)
{
  typedef vk::cameras::PinholeProjection<vk::cameras::NoDistortion> PinholeNoDist;
  typedef vk::cameras::PinholeProjection<vk::cameras::AtanDistortion> PinholeAtan;
  typedef vk::cameras::PinholeProjection<vk::cameras::EquidistantDistortion> PinholeEqui;
  typedef vk::cameras::PinholeProjection<vk::cameras::RadialTangentialDistortion> PinholeRadTan;

  PinholeNoDist cam1;
  PinholeNoDist cam2(
        12, 12, 6, 6, vk::cameras::NoDistortion());
  PinholeAtan cam3(
        12, 12, 6, 6, vk::cameras::AtanDistortion(0.934479));
  PinholeEqui cam4(
        12, 12, 6, 6, vk::cameras::EquidistantDistortion(-0.0027, 0.0241, -0.0430, 0.0311));
  PinholeRadTan cam5(
        12, 12, 6, 6, vk::cameras::RadialTangentialDistortion(-0.3, 0.1, 9.52e-05, -0.00057));

  vk::cameras::CameraGeometry<PinholeNoDist> geo1(10, 5, "cam1", Sophus::SE3(), cam2);
  vk::cameras::CameraGeometry<PinholeAtan> geo2(10, 5, "cam1", Sophus::SE3(), cam3);
  vk::cameras::CameraGeometry<PinholeEqui> geo3(10, 5, "cam1", Sophus::SE3(), cam4);
  vk::cameras::CameraGeometry<PinholeRadTan> geo4(10, 5, "cam1", Sophus::SE3(), cam5);
}

} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
