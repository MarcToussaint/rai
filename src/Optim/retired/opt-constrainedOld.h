/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

/// returns \f$f(x), \nabla f(x), \nabla^2 f(x), g(x), \nabla g(x)\f$ (giving NoArr as argument -> do not return this quantity)
typedef std::function<double(arr& df, arr& Hf,
                             arr& g, arr& Jg,
                             arr& h, arr& Jh, const arr& x)> ConstrainedProblem;

bool checkAllGradients(const ConstrainedProblem& P, const arr& x, double tolerance);

struct LagrangianProblem {
  /** The VectorFunction F describes the cost function f(x) as well as the constraints g(x)
      concatenated to one vector:
      phi(0) = cost,   phi(1,..,phi.N-1) = constraints */
  const ConstrainedProblem& P;

  //-- parameters of the unconstrained meta function F
  double muLB;       ///< log barrier weight
  double mu;         ///< squared penalty weight for inequalities g
  double nu;         ///< squared penalty weight for equalities h
  arr lambda;        ///< lagrange multipliers for inequalities g
  arr kappa;         ///< lagrange multiplier for equalities h

  //-- buffers to avoid recomputing gradients
  arr x; ///< point where P was last evaluated
  double f_x; ///< scalar value f(x)
  arr df_x, Hf_x, g_x, Jg_x, h_x, Jh_x; ///< everything else at x

  LagrangianProblem(const ConstrainedProblem& P):P(P), muLB(0.), mu(0.), nu(0.) {
    Lag = [this](arr& dL, arr& HL, const arr& x) -> double {
      return this->lagrangian(dL, HL, x);
    };
  }

  double lagrangian(arr& dL, arr& HL, const arr& x); ///< the unconstrained meta function F

  ScalarFunction Lag; ///< the unconstrained problem, typically the (augmented) Lagrangian with given lambda, mu, etc

//  operator const ScalarFunction&(){ return Lag; }

  void aulaUpdate(double lambdaStepsize=1., double muInc=1., double* L_x=nullptr, arr& dL_x=NoArr, arr& HL_x=NoArr);
  void anyTimeAulaUpdate(double lambdaStepsize=1., double muInc=1., double* L_x=nullptr, arr& dL_x=NoArr, arr& HL_x=NoArr);
  bool anyTimeAulaUpdateStopCriterion(const arr& dL_x);
};

uint optConstrained(arr& x, arr& dual, const ConstrainedProblem& P, OptOptions opt=NOOPT);
