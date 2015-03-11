#pragma once

#include <string>
#include <iostream>

namespace vk {
namespace test_utils {

// usage: getFileDir(__FILE__)
std::string getFileDir(const std::string& filename)
{
  for (auto s = filename.rbegin(); s < filename.rend(); ++s)
    if(*s == '/')
      return std::string(filename.begin(), (s+1).base());
  std::cout << "ERROR vk::test_utils::getFileDir(): could not decompose string"
            << std::endl;
  return std::string("/");
}

} // namespace test_utils
} // namespace vk
