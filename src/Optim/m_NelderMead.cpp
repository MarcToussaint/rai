#include "m_NelderMead.h"

using std::tie;
using rai::min_arg;
using rai::max_arg;

// =========================== original: https://github.com/develancer/nelder-mead =========================

#ifndef PTR_NELDER_MEAD_H
#define PTR_NELDER_MEAD_H

#include <array>
#include <climits>
#include <functional>

/**
 * Plain data object with output information from the run of nelder_mead routine.
 *
 * @tparam double floating-point type to be used, e.g. double
 * @tparam n the number of variables
 */
struct nelder_mead_result {
  arr xmin;
  double ynewlo;
  int icount;
  int numres;
  int ifault;
};

/**
 * This routine seeks the minimum value of a user-specified function.
 *
 * @tparam double floating-point type to be used, e.g. double
 * @tparam n the number of variables
 * @param fn the function to be minimized
 * @param start a starting point for the iteration
 * @param reqmin the terminating limit for the variance of function values
 * @param step determines the size and shape of the initial simplex;
 * the relative magnitudes of its elements should reflect the units of the variables
 * @param konvge the convergence check is carried out every konvge iterations
 * @param kcount the maximum number of function evaluations
 * @return structure with output information
 */
nelder_mead_result nelder_mead(
    const std::function<double(const arr &)> &fn,
    arr start,
    double reqmin,
    const arr &step,
    int kcount = INT_MAX
    ) {

  uint n=start.N;

  const double ccoeff = 0.5;
  double del = 1.;
  const double ecoeff = 2.0;
  const double eps = 0.001;
  const double rcoeff = 1.0;
  double y2star;
  double ystar;


  nelder_mead_result result;

  // Check the input parameters.
  CHECK_GE(reqmin, 0., "");
  CHECK_GE(n, 1, "");

  arr p(n + 1, n);
  arr pstar(n), p2star(n), pbar(n);
  arr y(n + 1);

  result.icount = 0;
  result.numres = 0;

  // Initial or restarted loop.
  for (;;) {
    p[n] = start;
    y(n) = fn(start);
    result.icount++;

    for(uint j = 0; j < n; j++) {
      p[j] = start;
      p(j,j) += step(j) * del;
      y(j) = fn(p[j]);
      result.icount++;
    }

    // The simplex construction is complete.

    // Find highest and lowest Y values.
    // YNEWLO = Y(IHI) indicates the vertex of the simplex to be replaced.
    double ylo;
    uint ilo;
    tie(ylo, ilo) = min_arg(y);

    cout <<"--nelderMead-- " <<result.icount <<" f: " <<ylo <<endl;

    // Inner loop.
    for (;;) {
      if (kcount <= result.icount) {
        break;
      }

      uint ihi;
      tie(result.ynewlo, ihi) = max_arg(y);

      // Calculate PBAR, the centroid of the simplex vertices
      // excepting the vertex with Y value YNEWLO.
      pbar = sum(p, 0);
      pbar -= p[ihi];
      pbar /= n;

      // Reflection through the centroid.
      pstar = pbar + rcoeff * (pbar - p[ihi]);
      ystar = fn(pstar);
      result.icount++;
      cout <<"--nelderMead-- " <<result.icount <<" f: " <<ystar <<endl;

      // Successful reflection, so extension.
      if (ystar < ylo) {
        p2star = pbar + ecoeff * (pstar - pbar);
        y2star = fn(p2star);
        result.icount++;
        cout <<"--nelderMead-- " <<result.icount <<" f: " <<y2star <<" (extended)" <<endl;

	// Check extension.
	if (ystar < y2star) {
	  p[ihi] = pstar;
	  y(ihi) = ystar;
	} else {
	  // Retain extension or contraction.
	  p[ihi] = p2star;
	  y(ihi) = y2star;
	}
      } else {
        // No extension.
        uint l = 0;
        for(uint i = 0; i < n + 1; i++) {
          if (ystar < y(i)) {
            l++;
          }
        }

	if (l > 1) {
	  p[ihi] = pstar;
	  y(ihi) = ystar;
	} else if (l == 0) {
	  // Contraction on the Y(IHI) side of the centroid.
	  p2star = pbar + ccoeff * (p[ihi] - pbar);
	  y2star = fn(p2star);
	  result.icount++;
	  cout <<"--nelderMead-- " <<result.icount <<" f: " <<y2star <<" (contracted)" <<endl;

	  // Contract the whole simplex.
	  if (y(ihi) < y2star) {
	    for(uint j = 0; j < n + 1; j++) {
	      p[j] = .5*(p[j]+p[ilo]);
	      result.xmin = p[j];
	      y(j) = fn(result.xmin);
	      result.icount++;
	    }
	    cout <<"--nelderMead-- " <<result.icount <<" f: " <<y2star <<" (contracted whole simplex)" <<endl;

	    tie(ylo,ilo) = min_arg(y);
	    continue;
	  } else {
	    // Retain contraction.
	    p[ihi] = p2star;
	    y(ihi) = y2star;
	  }
	} else if (l == 1) {
	  // Contraction on the reflection side of the centroid.
	  p2star = pbar + ccoeff * (pstar - pbar);
	  y2star = fn(p2star);
	  result.icount++;
	  cout <<"--nelderMead-- " <<result.icount <<" f: " <<y2star <<" (contracted reflection)" <<endl;

	  // Retain reflection?
	  if (y2star <= ystar) {
	    p[ihi] = p2star;
	    y(ihi) = y2star;
	  } else {
	    p[ihi] = pstar;
	    y(ihi) = ystar;
	  }
	}
      }

      // Check if YLO improved.
      if (y(ihi) < ylo) {
        ylo = y(ihi);
        ilo = ihi;
      }

      // Check to see if minimum reached.
#if 0
      if (result.icount <= kcount) {
        double x = sum(y) / (n + 1);
        double z = sumOfSqr(y - x) / n;
        cout <<"--nelderMead-- conv check: z=" <<z <<" reqmin=" <<reqmin <<endl;
        if(z <= reqmin) break;
      }
#endif
      double sig = sqrt(sum(vardiag(p)));
      cout <<"--nelderMead-- std dev: " <<sig <<" reqmin=" <<reqmin <<endl;
      if(sig<reqmin) break;
    }
    // Factorial tests to check that YNEWLO is a local minimum.
    result.xmin = p[ilo];
    result.ynewlo = y(ilo);
    break;
#if 0
    if (kcount < result.icount) {
      result.ifault = 2;
      break;
    }

    bool isLocalMinimum = true;
    for(uint i = 0; i < n; i++) {
      del = step(i) * eps;
      result.xmin(i) += del;
      double z = fn(result.xmin);
      result.icount++;
      if (z < result.ynewlo) {
        isLocalMinimum = false;
        break;
      }
      result.xmin(i) -= del + del;
      z = fn(result.xmin);
      result.icount++;
      if (z < result.ynewlo) {
        isLocalMinimum = false;
        break;
      }
      result.xmin(i) += del;
    }

    if(isLocalMinimum){
      result.ifault = 0;
      break;
    }

    // Restart the procedure.
    start = result.xmin;
    del = eps;
    result.numres++;
#endif
  }

  return result;
}

#endif // PTR_NELDER_MEAD_H

// ====================================== end of original ======================================

NelderMead::NelderMead(ScalarFunction _f, const arr& x_init) : f(_f){
  x=x_init;
}

shared_ptr<SolverReturn> NelderMead::solve(){
  uint n = x.N;

  auto f = [this](const arr& x) {
    return this->f(NoArr, NoArr, x);
  };

  arr step(n);
  step = .1;

  nelder_mead_result result = nelder_mead(
      f,
      x,
      1e-2, // the terminating limit for the variance of function values
      step
      );

  shared_ptr<SolverReturn> ret = make_shared<SolverReturn>();
  ret->x.setCarray(&result.xmin(0), n);
  ret->f = result.ynewlo;
  ret->evals = result.icount;
  ret->feasible=true;
  return ret;
}

