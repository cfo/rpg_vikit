/*
 * blender_utils.h
 *
 *  Created on: Feb 13, 2014
 *      Author: cforster
 */

#ifndef VIKIT_BLENDER_UTILS_H_
#define VIKIT_BLENDER_UTILS_H_

#include <list>
#include <string>
#include <vikit/math_utils.h>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <Eigen/Core>

namespace vk {
namespace blender_utils {

void loadBlenderDepthmap(
    const std::string file_name,
    const int img_width,
    const int img_height,
    cv::Mat& z_map)
{
  std::ifstream file_stream(file_name.c_str());
  assert(file_stream.is_open());
  z_map = cv::Mat(img_height, img_width, CV_32FC1);
  float * img_ptr = z_map.ptr<float>();
  float depth;
  for(int y=0; y<img_height; ++y)
  {
    for(int x=0; x<img_width; ++x, ++img_ptr)
    {
      file_stream >> depth;

      // blender:
      *img_ptr = depth;

      // povray
      // *img_ptr = depth/100.0; // depth is in [cm], we want [m]

      if(file_stream.peek() == '\n' && x != img_width-1 && y != img_height-1)
        printf("WARNING: did not read the full depthmap!\n");
    }
  }
}

/*
bool getDepthmapNormalAtPoint(
    const Vector2i& px,
    const cv::Mat& depth,
    const int halfpatch_size,
    const vk::cameras::CameraGeometryBase::Ptr& cam,
    Vector3d& normal)
{
  assert(cam->imageWidth() == depth.cols && cam->imageHeight() == depth.rows);
  if(!cam->isInFrame(px, halfpatch_size+1))
    return false;

  const size_t n_meas = (halfpatch_size*2+1)*(halfpatch_size*2+1);
  std::list<Vector3d> pts;
  for(int y = px[1]-halfpatch_size; y<=px[1]+halfpatch_size; ++y)
    for(int x = px[0]-halfpatch_size; x<=px[0]+halfpatch_size; ++x)
      pts.push_back(cam->camToWorld(Vector2d(x,y)).normalized()*depth.at<float>(y,x));

  assert(n_meas == pts.size());
  Matrix<double, Dynamic, 4> A; A.resize(n_meas, Eigen::NoChange);
  Matrix<double, Dynamic, 1> b; b.resize(n_meas, Eigen::NoChange);

  size_t i = 0;
  for(std::list<Vector3d>::iterator it=pts.begin(); it!=pts.end(); ++it)
  {
    A.row(i) << it->x(), it->y(), it->z(), 1.0;
    b[i] = 0;
    ++i;
  }

  JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

  Matrix<double, 4, 4> V = svd.matrixV();
  normal = V.block<3,1>(0,3);
  normal.normalize();
  return true;
}
*/

} // namespace blender_utils
} // namespace vk

#endif // VIKIT_BLENDER_UTILS_H_
