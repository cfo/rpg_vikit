/*
 * homography.cpp
 * Adaptation of PTAM-GPL HomographyInit class.
 * https://github.com/Oxford-PTAM/PTAM-GPL
 * Licence: GPLv3
 * Copyright 2008 Isis Innovation Limited
 *
 *  Created on: Sep 2, 2012
 *      by: cforster
 *
 * This class implements the homography decomposition of Faugeras and Lustman's
 * 1988 tech report. Code converted to Eigen from PTAM.
 *
 */

#ifndef VIKIT_HOMOGRAPHY_H_
#define VIKIT_HOMOGRAPHY_H_

#include <vector>
#include <Eigen/Core>

namespace vk {

struct Homography
{
  Eigen::Vector3d t_cur_ref;
  Eigen::Matrix3d R_cur_ref;
  Eigen::Vector3d n_cur;
  double score;
  Homography()
    : t_cur_ref()
    , R_cur_ref()
    , n_cur()
    , score(0.0)
  {}
};

/// Estimates Homography from corresponding feature bearing vectors.
/// Score of returned homography is set to the number of inliers.
Homography estimateHomography(
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& f_cur,
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& f_ref,
    const double focal_length,
    const double reproj_error_thresh,
    const size_t min_num_inliers);

} // namespace vk

#endif // VIKIT_HOMOGRAPHY_H_
