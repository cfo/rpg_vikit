/*
 * Abstract Nonlinear Least-Squares Solver Class
 *
 * nlls_solver.h
 *
 *  Created on: Nov 5, 2012
 *      Author: cforster
 */

#ifndef LM_SOLVER_IMPL_HPP_
#define LM_SOLVER_IMPL_HPP_

#include <stdexcept>
#include <vikit/nlls_solver.h>

namespace vk {

template <int D, typename T>
void MiniLeastSquaresSolver<D, T>::optimize(State& state)
{
  if(method_ == GaussNewton)
    optimizeGaussNewton(state);
  else if(method_ == LevenbergMarquardt)
    optimizeLevenbergMarquardt(state);
}

template <int D, typename T>
void MiniLeastSquaresSolver<D, T>::optimizeGaussNewton(State& state)
{
  // Save the old model to rollback in case of unsuccessful update
  State old_model(state);

  // perform iterative estimation
  for (iter_ = 0; iter_<n_iter_; ++iter_)
  {
    rho_ = 0;
    startIteration();

    H_.setZero();
    Jres_.setZero();

    // compute initial error
    n_meas_ = 0;
    double new_chi2 = evaluateError(state, &H_, &Jres_);

    // add prior
    if(have_prior_)
      applyPrior(state);

    // solve the linear system
    if(!solve(H_, Jres_, x_))
    {
      // matrix was singular and could not be computed
      std::cout << "Matrix is close to singular! Stop Optimizing." << std::endl;
      std::cout << "H = " << H_ << std::endl;
      std::cout << "Jres = " << Jres_ << std::endl;
      stop_ = true;
    }

    // check if error increased since last optimization
    if((iter_ > 0 && new_chi2 > chi2_ && stop_when_error_increases_) || stop_)
    {
      if(verbose_)
      {
        std::cout << "It. " << iter_
                  << "\t Failure"
                  << "\t new_chi2 = " << new_chi2
                  << "\t n_meas = " << n_meas_
                  << "\t Error increased. Stop optimizing."
                  << std::endl;
      }
      state = old_model; // rollback
      break;
    }

    // update the model
    State new_state;
    update(state, new_state);
    old_model = state;
    state = new_state;
    chi2_ = new_chi2;
    double x_norm = vk::norm_max(x_);

    if(verbose_)
    {
      std::cout << "It. " << iter_
                << "\t Success"
                << "\t new_chi2 = " << new_chi2
                << "\t n_meas = " << n_meas_
                << "\t x_norm = " << x_norm
                << std::endl;
    }

    finishIteration();

    // stop when converged, i.e. update step too small
    if(x_norm < eps_)
    {
      if(verbose_)
      {
        std::cout << "Converged, x_norm " << x_norm << " < " << eps_ << std::endl;
      }
      break;
    }
  }
}

template <int D, typename T>
void MiniLeastSquaresSolver<D, T>::optimizeLevenbergMarquardt(State& state)
{
  // compute the initial error
  chi2_ = evaluateError(state, nullptr, nullptr);

  if(verbose_)
    std::cout << "init chi2 = " << chi2_
         << "\t n_meas = " << n_meas_
         << std::endl;

  // TODO: compute initial lambda
  // Hartley and Zisserman: "A typical init value of lambda is 10^-3 times the
  // average of the diagonal elements of J'J"

  // Compute Initial Lambda
  if(mu_ < 0)
  {
    double H_max_diag = 0;
    double tau = 1e-4;
    for(size_t j=0; j<D; ++j)
      H_max_diag = std::max(H_max_diag, std::fabs(H_(j,j)));
    mu_ = tau*H_max_diag;
  }

  // perform iterative estimation
  for (iter_ = 0; iter_<n_iter_; ++iter_)
  {
    rho_ = 0;
    startIteration();

    // try to compute and update, if it fails, try with increased mu
    n_trials_ = 0;
    do
    {
      // init variables
      State new_model;
      double new_chi2 = -1;
      H_.setZero();
      //H_ = mu_ * Matrix<double,D,D>::Identity(D,D);
      Jres_.setZero();

      // linearize
      n_meas_ = 0;
      evaluateError(state, &H_, &Jres_);

      // add damping term:
      H_ += (H_.diagonal()*mu_).asDiagonal();

      // add prior
      if(have_prior_)
        applyPrior(state);

      // solve the linear system to obtain small perturbation in direction of gradient
      if(solve(H_, Jres_, x_))
      {
        // apply perturbation to the state
        update(state, new_model);

        // compute error with new model and compare to old error
        n_meas_ = 0;
        new_chi2 = evaluateError(new_model, nullptr, nullptr);
        rho_ = chi2_-new_chi2;
      }
      else
      {
        // matrix was singular and could not be computed
        std::cout << "Matrix is close to singular!" << std::endl;
        std::cout << "H = " << H_ << std::endl;
        std::cout << "Jres = " << Jres_ << std::endl;
        rho_ = -1;
      }

      if(rho_>0)
      {
        // update decrased the error -> success
        state = new_model;
        chi2_ = new_chi2;
        stop_ = vk::norm_max(x_)<=eps_;
        mu_ *= std::max(1./3., std::min(1.-std::pow(2*rho_-1,3), 2./3.));
        nu_ = 2.;
        if(verbose_)
        {
          std::cout << "It. " << iter_
                    << "\t Trial " << n_trials_
                    << "\t Success"
                    << "\t n_meas = " << n_meas_
                    << "\t new_chi2 = " << new_chi2
                    << "\t mu = " << mu_
                    << "\t nu = " << nu_
                    << std::endl;
        }
      }
      else
      {
        // update increased the error -> fail
        mu_ *= nu_;
        nu_ *= 2.;
        ++n_trials_;
        if (n_trials_ >= n_trials_max_)
          stop_ = true;

        if(verbose_)
        {
          std::cout << "It. " << iter_
                    << "\t Trial " << n_trials_
                    << "\t Failure"
                    << "\t n_meas = " << n_meas_
                    << "\t new_chi2 = " << new_chi2
                    << "\t mu = " << mu_
                    << "\t nu = " << nu_
                    << std::endl;
        }
      }

      finishTrial();

    } while(!(rho_>0 || stop_));
    if (stop_)
      break;

    finishIteration();
  }
}

template <int D, typename T>
void MiniLeastSquaresSolver<D, T>::setPrior(
    const T&  prior,
    const Matrix<double, D, D>&  Information)
{
  have_prior_ = true;
  prior_ = prior;
  I_prior_ = Information;
}

template <int D, typename T>
void MiniLeastSquaresSolver<D, T>::reset()
{
  have_prior_ = false;
  chi2_ = 1e10;
  mu_ = mu_init_;
  nu_ = nu_init_;
  n_meas_ = 0;
  n_iter_ = n_iter_init_;
  iter_ = 0;
  stop_ = false;
}

template <int D, typename T>
inline const double& MiniLeastSquaresSolver<D, T>::getChi2() const
{
  return chi2_;
}

template <int D, typename T>
inline const vk::Matrix<double, D, D>& MiniLeastSquaresSolver<D, T>::getInformationMatrix() const
{
  return H_;
}

template <int D, typename T>
bool MiniLeastSquaresSolver<D, T>::solve(
    const HessianMatrix& H,
    const GradientVector& g,
    UpdateVector& dx)
{
  dx = H.ldlt().solve(g);
  if((bool) std::isnan((double) dx[0]))
    return false;
  return true;
}

} // namespace vk

#endif /* LM_SOLVER_IMPL_HPP_ */
