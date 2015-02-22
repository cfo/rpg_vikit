#include <gtest/gtest.h>
#include <vikit/cameras.h>
#include <vikit/cameras/camera_factory.h>


namespace test_utils {

std::string getFileDir()
{
  std::string filename(__FILE__);
  for (auto s = filename.rbegin(); s < filename.rend(); ++s)
    if(*s == '/')
      return std::string(filename.begin(), (s+1).base());
  std::cout << "ERROR getFileDir(): could not decompose string" << std::endl;
  return std::string("/");
}

} // namespace test_utils

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

TEST(CamerasTest, CameraFactoryNoDistortion)
{
  std::string data_dir = test_utils::getFileDir()+"/data";
  vk::cameras::CameraGeometryBase::Ptr cam1 =
      vk::cameras::factory::loadFromYAML(
        data_dir+"/calib_pinhole_nodistortion.yaml", "cam0");
  cam1->print("Test Load Camera No Distortion:");
}

TEST(CamerasTest, CameraFactoryAtan)
{
  std::string data_dir = test_utils::getFileDir()+"/data";
  vk::cameras::CameraGeometryBase::Ptr cam1 =
      vk::cameras::factory::loadFromYAML(
        data_dir+"/calib_pinhole_atan.yaml", "cam0");
  cam1->print("Test Load Camera Atan:");
}

TEST(CamerasTest, CameraFactoryEquidistant)
{
  std::string data_dir = test_utils::getFileDir()+"/data";
  vk::cameras::CameraGeometryBase::Ptr cam1 =
      vk::cameras::factory::loadFromYAML(
        data_dir+"/calib_pinhole_equidistant.yaml", "cam0");
  cam1->print("Test Load Camera Equidistant:");
}

TEST(CamerasTest, CameraFactoryRadTan)
{
  std::string data_dir = test_utils::getFileDir()+"/data";
  vk::cameras::CameraGeometryBase::Ptr cam1 =
      vk::cameras::factory::loadFromYAML(
        data_dir+"/calib_pinhole_radtan.yaml", "cam0");
  cam1->print("Test Load Camera Radial Tangential:");
}



} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
