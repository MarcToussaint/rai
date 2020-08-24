/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "optimization.h"

/// functions \f$ \phi_t:(x_{t-k},..,x_t) \mapsto y\in\mathbb{R}^{m_t} \f$ over a chain \f$x_0,..,x_T\f$ of variables
struct KOrderMarkovFunction {
  virtual void set_x(const arr& x) = 0;

  /// returns $\f\phi(x), \nabla \phi(x)\f$ for a given time step t and a k+1 tuple of states \f$\bar x = (x_{t-k},..,x_t)\f$.
  /// This defines the cost function \f$f_t = \phi_t^\top \phi_t\f$ in the time slice. Optionally, the last dim_g entries of
  ///  \f$\phi\f$ are interpreted as inequality constraint function \f$g(\bar x)\f$ for time slice t
  virtual void phi_t(arr& phi, arr& J, ObjectiveTypeA& tt, uint t) = 0;

  //functions to get the parameters $T$, $k$ and $n$ of the $k$-order Markov Process
  virtual uint get_T() = 0;       ///< horizon (the total x-dimension is (T+1)*n )
  virtual uint get_k() = 0;       ///< the order of dependence: \f$ \phi=\phi(x_{t-k},..,x_t) \f$
  virtual uint dim_x() { uint d=0, T=get_T(); for(uint t=0; t<=T; t++) d+=dim_x(t); return d; }   ///< \f$ \sum_t \dim(x_t) \f$
  virtual uint dim_x(uint t) = 0;       ///< \f$ \dim(x_t) \f$
  virtual uint dim_phi(uint t) = 0; ///< \f$ \dim(\phi_t) \f$
  virtual uint dim_g(uint t) { return 0; } ///< number of inequality constraints at the end of \f$ \phi_t \f$ (before h terms)
  virtual uint dim_h(uint t) { return 0; } ///< number of equality constraints at the very end of \f$ \phi_t \f$
  virtual uint dim_g_h() { uint d=0, T=get_T(); for(uint t=0; t<=T; t++) d += dim_g(t) + dim_h(t); return d; }
  virtual StringA getPhiNames(uint t) { return StringA(); }
  virtual arr get_prefix() { arr x(get_k(), dim_x()); x.setZero(); return x; } ///< the augmentation \f$ (x_{t=-k},..,x_{t=-1}) \f$ that makes \f$ \phi_{0,..,k-1} \f$ well-defined
  virtual arr get_postfix() { return arr(); } ///< by default there is no definite final configuration

  /// optional: we make include kernel costs \f$ \sum_{i,j} k(i,j) x_i^\top x_j \f$ -- PRELIM, see examples/kOrderMarkov
  virtual bool hasKernel() { return false; }
  virtual double kernel(uint t0, uint t1) { NIY; } ///< a kernel adds additional costs: neg-log-likelihoods of a Gaussian Process

  virtual ~KOrderMarkovFunction() {}
};

ScalarFunction     conv_KOrderMarkovFunction2ScalarFunction(KOrderMarkovFunction& f);
VectorFunction     conv_KOrderMarkovFunction2VectorFunction(KOrderMarkovFunction& f);
//ConstrainedProblem conv_KOrderMarkovFunction2ConstrainedProblem(KOrderMarkovFunction& f);
