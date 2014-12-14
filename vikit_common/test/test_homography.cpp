/*
 * test_patch_score.cpp
 *
 *  Created on: Dec 4, 2012
 *      Author: cforster
 */

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <vikit/math_utils.h>
#include <vikit/homography_decomp.h>

namespace {

void testHomography()
{
  cv::Matx33d K(640, 0.0,  320,
                0,    640, 240,
                0,    0,   1);
  cv::Matx33d H(2.649157564634028,  4.583875997496426,  70.694447785121326,
                  -1.072756858861583,  3.533262150437228,  1513.656999614321649,
                  0.001303887589576,  0.003042206876298,  1.000000000000000);

  //expected solution for the given homography and intrinsic matrices
  cv::Matx33d R_expected(0.43307983549125, 0.545749113549648, -0.717356090899523,
                      -0.85630229674426, 0.497582023798831, -0.138414255706431,
                       0.281404038139784, 0.67421809131173, 0.682818960388909);
  cv::Vec3d t_expected(1.826751712278038,  1.264718492450820,  0.195080809998819);
  cv::Vec3d n_expected(0.244875830334816, 0.480857890778889, 0.841909446789566);

  std::vector<cv::Mat> rotations;
  std::vector<cv::Mat> translations;
  std::vector<cv::Mat> normals;

  cv::decomposeHomographyMat(H, K, rotations, translations, normals);

  std::cout << "found " << rotations.size() << " decompositions" << std::endl;
  for(size_t i; i<rotations.size(); ++i)
  {
    // TODO
  }

}

} // namespace

int main(int argc, char **argv)
{
  testHomography();
  return 0;
}
