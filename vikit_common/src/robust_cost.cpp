#include <numeric>
#include <algorithm>
#include <vikit/robust_cost.h>
#include <vikit/math_utils.h>

namespace vk {
namespace robust_cost {

const float MADScaleEstimator::NORMALIZER = 1.48f; // 1 / 0.6745

float MADScaleEstimator::compute(std::vector<float>& errors) const
{
  // error must be in absolute values!
  return NORMALIZER * vk::getMedian(errors);
}

float NormalDistributionScaleEstimator::compute(std::vector<float>& errors) const
{
  const float mean = std::accumulate(errors.begin(), errors.end(), 0)/errors.size();
  float var = 0.0;
  for(const float d : errors)
    var += (d - mean) * (d - mean);
  return std::sqrt(var); // return standard deviation
}

const float TukeyWeightFunction::DEFAULT_B = 4.6851f;

TukeyWeightFunction::TukeyWeightFunction(const float b)
{
  configure(b);
}

float TukeyWeightFunction::value(const float& x) const
{
  const float x_square = x * x;
  if(x_square <= b_square)
  {
    const float tmp = 1.0f - x_square / b_square;
    return tmp * tmp;
  }
  else
  {
    return 0.0f;
  }
}

void TukeyWeightFunction::configure(const float& param)
{
  b_square = param * param;
}

const float HuberWeightFunction::DEFAULT_K = 1.345f;

HuberWeightFunction::HuberWeightFunction(const float k)
{
  configure(k);
}

void HuberWeightFunction::configure(const float& param)
{
  k = param;
}

float HuberWeightFunction::value(const float& t) const
{
  const float t_abs = std::abs(t);
  if(t_abs < k)
    return 1.0f;
  else
    return k / t_abs;
}

} // namespace robust_cost
} // namespace vk


