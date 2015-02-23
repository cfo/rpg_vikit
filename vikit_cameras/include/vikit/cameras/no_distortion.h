#pragma once

#include <iostream>

namespace vk {
namespace cameras {

class NoDistortion 
{
public:
  NoDistortion() = default;
  ~NoDistortion() = default;

  inline void distort(
      double& /*u*/, double& /*v*/) const
  {}

  inline void undistort(
      double& /*u*/, double& /*v*/) const
  {}

  inline void print() const
  {
    std::cout << "  Distortion: No" << std::endl;
  }
};


} // namespace cameras
} // namespace vk
