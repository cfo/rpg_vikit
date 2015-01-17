#ifndef VIKIT_SAMPLE_H_
#define VIKIT_SAMPLE_H_

#include <random>
#include <chrono>
#include <Eigen/Core>

namespace vk {

class Sample
{
public:
  static void setTimeBasedSeed();
  static int uniform(int from, int to);
  static double uniform();
  static double gaussian(double sigma);
  static std::ranlux24 gen_real;
  static std::mt19937 gen_int;
  static Eigen::Vector3d randomDirection3D();
  static Eigen::Vector2d randomDirection2D();
};

std::ranlux24 Sample::gen_real;
std::mt19937 Sample::gen_int;

void Sample::setTimeBasedSeed()
{
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  gen_real = std::ranlux24(seed);
  gen_int = std::mt19937(seed);
}

int Sample::uniform(int from, int to)
{
  std::uniform_int_distribution<int> distribution(from, to);
  return distribution(gen_int);
}

double Sample::uniform()
{
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  return distribution(gen_real);
}

double Sample::gaussian(double stddev)
{
  std::normal_distribution<double> distribution(0.0, stddev);
  return distribution(gen_real);
}

Eigen::Vector3d Sample::randomDirection3D()
{
  // equal-area projection according to:
  // https://math.stackexchange.com/questions/44689/how-to-find-a-random-axis-or-unit-vector-in-3d
  const double z = Sample::uniform()*2.0-1.0;
  const double t = Sample::uniform()*2.0*M_PI;
  const double r = std::sqrt(1.0 - z*z);
  const double x = r*std::cos(t);
  const double y = r*std::sin(t);
  return Eigen::Vector3d(x,y,z);
}

Eigen::Vector2d Sample::randomDirection2D()
{
  const double theta = Sample::uniform()*2.0*M_PI;
  return Eigen::Vector2d(std::cos(theta), std::sin(theta));
}

} // namespace vk

#endif // VIKIT_SAMPLE_H_
