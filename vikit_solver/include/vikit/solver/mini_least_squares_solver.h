/*
 * Abstract Nonlinear Least-Squares Solver Class
 *
 * nlls_solver.h
 *
 *  Created on: Nov 5, 2012
 *      Author: cforster
 */

#ifndef LM_SOLVER_H_
#define LM_SOLVER_H_

#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/StdVector>

namespace vk {
namespace solver {

using namespace Eigen;

enum class Strategy {
  GaussNewton,
  LevenbergMarquardt
};

struct MiniLeastSquaresSolverOptions
{
  /// Solver strategy.
  Strategy strategy = Strategy::GaussNewton;

  /// Damping parameter. If mu > 0, coefficient matrix is positive definite, this
  /// ensures that x is a descent direction. If mu is large, x is a short step in
  /// the steepest direction. This is good if the current iterate is far from the
  /// solution. If mu is small, LM approximates gauss newton iteration and we
  /// have (almost) quadratic convergence in the final stages.
  double mu_init = 0.01f;

  /// Increase factor of mu after fail
  double nu_init = 2.0;

  /// Max number of iterations
  size_t max_iter = 15;

  /// Max number of trials (used in LevenbergMarquardt)
  size_t max_trials = 5;

  /// Stop when error increases.
  bool stop_when_error_increases = false;

  /// Output Statistics
  bool verbose = false;

  /// Stop if update norm is smaller than eps
  double eps = 0.0000000001;
};


/**
 * \brief Abstract Class for solving nonlinear least-squares (NLLS) problems.
 *
 * The function implements two algorithms: Levenberg Marquardt and Gauss Newton
 *
 * Example implementations of this function can be found in the rpl_examples
 * package: img_align_2d.cpp, img_align_3d.cpp
 *
 * Template Parameters:
 * D  : dimension of the residual
 * T  : type of the model, e.g. SE2, SE3
 */
template <int D, typename T>
class MiniLeastSquaresSolver
{
public:
  typedef T State;
  typedef Matrix<double, D, D> HessianMatrix;
  typedef Matrix<double, D, 1> GradientVector;
  typedef Matrix<double, D, 1> UpdateVector;

public:

  MiniLeastSquaresSolverOptions solver_options_;

  MiniLeastSquaresSolver(const MiniLeastSquaresSolverOptions& options) :
    solver_options_(options)
  {}

  virtual ~MiniLeastSquaresSolver() = default;

  /// Calls the GaussNewton or LevenbergMarquardt optimization strategy
  void optimize(State& state);

  /// Gauss Newton optimization strategy
  void optimizeGaussNewton(State& state);

  /// Levenberg Marquardt optimization strategy
  void optimizeLevenbergMarquardt(State& state);

  /// Add prior to optimization.
  void setPrior(
      const State&  prior,
      const Matrix<double, D, D>&  Information);

  /// Reset all parameters to restart the optimization
  void reset();

  /// Get the squared error
  const double& getChi2() const;

  /// The Information matrix is equal to the inverse covariance matrix.
  const Matrix<double, D, D>& getInformationMatrix() const;

protected:

  /// If the flag linearize_system is set, the function must also compute the
  /// Jacobian and set the member variables H_, Jres_
  virtual double evaluateError(
      const State& state,
      HessianMatrix* H,
      GradientVector* g) = 0;

  /// Solve the linear system H*x = Jres. This function must set the update
  /// step in the member variable x_. Must return true if the system could be
  /// solved and false if it was singular.
  virtual bool solve(
      const HessianMatrix& H,
      const GradientVector& g,
      UpdateVector& dx);

  /// Apply the perturbation dx to the state.
  virtual void update(
      const State& state,
      const UpdateVector& dx,
      State& new_state) = 0;

  virtual void applyPrior(const State& /*current_model*/)
  {}

  virtual void startIteration()
  {}

  virtual void finishIteration()
  {}

  virtual void finishTrial()
  {}

  HessianMatrix  H_;        ///< Hessian or approximation Jacobian*Jacobian^T.
  GradientVector g_;        ///< Jacobian*residual.
  UpdateVector   dx_;       ///< Update step.
  bool have_prior_ = false;
  State prior_;
  Matrix<double, D, D> I_prior_; ///< Prior information matrix (inverse covariance)
  double chi2_ = 0.0;
  double rho_ = 0.0;
  double mu_ = 0.01;        ///< Damping parameter.
  double nu_ = 2.0;         ///< Factor that specifies how much we increase mu at every trial.
  size_t n_meas_ = 0;       ///< Number of measurements.
  bool stop_ = false;       ///< Stop flag.
  size_t iter_ = 0;         ///< Current Iteration.
  size_t trials_ = 0;       ///< Current number of trials.

};

} // namespace solver
} // namespace vk

#include "implementation/mini_least_squares_solver.hpp"

#endif /* LM_SOLVER_H_ */
