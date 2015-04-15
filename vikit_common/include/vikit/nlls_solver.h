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

#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vikit/math_utils.h>
#include <vikit/robust_cost.h>

namespace vk {

using namespace Eigen;

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
class MiniLeastSquaresSolver {

public:
  typedef T State;
  typedef Matrix<double, D, D> HessianMatrix;
  typedef Matrix<double, D, 1> GradientVector;
  typedef Matrix<double, D, 1> UpdateVector;
  enum Method{GaussNewton, LevenbergMarquardt};

protected:
  HessianMatrix  H_;     ///< Hessian or approximation Jacobian*Jacobian^T.
  GradientVector Jres_;  ///< Jacobian*residual.
  UpdateVector   x_;     ///< Update step.

  bool                  have_prior_;
  State prior_;
  Matrix<double, D, D>  I_prior_; //!< Prior information matrix (inverse covariance)
  double                chi2_;
  double                rho_;
  Method                method_;

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

  virtual void
  update                (const State& old_model, State& new_model) = 0;

  virtual void
  applyPrior            (const State& /*current_model*/) { }

  virtual void
  startIteration        () { }

  virtual void
  finishIteration       () { }

  virtual void
  finishTrial           () { }

public:

  /// Damping parameter. If mu > 0, coefficient matrix is positive definite, this
  /// ensures that x is a descent direction. If mu is large, x is a short step in
  /// the steepest direction. This is good if the current iterate is far from the
  /// solution. If mu is small, LM approximates gauss newton iteration and we
  /// have (almost) quadratic convergence in the final stages.
  double                mu_init_, mu_;
  double                nu_init_, nu_;          //!< Increase factor of mu after fail
  size_t                n_iter_init_, n_iter_;  //!< Number of Iterations
  size_t                n_trials_;              //!< Number of trials
  size_t                n_trials_max_;          //!< Max number of trials
  size_t                n_meas_;                //!< Number of measurements
  bool                  stop_;                  //!< Stop flag
  bool                  stop_when_error_increases_;
  bool                  verbose_;               //!< Output Statistics
  double                eps_;                   //!< Stop if update norm is smaller than eps
  size_t                iter_;                  //!< Current Iteration

  MiniLeastSquaresSolver() :
    have_prior_(false),
    method_(LevenbergMarquardt),
    mu_init_(0.01f),
    mu_(mu_init_),
    nu_init_(2.0),
    nu_(nu_init_),
    n_iter_init_(15),
    n_iter_(n_iter_init_),
    n_trials_(0),
    n_trials_max_(5),
    n_meas_(0),
    stop_(false),
    stop_when_error_increases_(false),
    verbose_(true),
    eps_(0.0000000001),
    iter_(0)
  {}

  virtual ~MiniLeastSquaresSolver() {}

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
};

} // end namespace vk

#include "nlls_solver_impl.hpp"

#endif /* LM_SOLVER_H_ */
