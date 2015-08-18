/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


/// @file
/// @ingroup group_Optim
/// @addtogroup group_Optim
/// @{

#ifndef MT_optimization_benchmarks_h
#define MT_optimization_benchmarks_h

#include "optimization.h"

extern ScalarFunction RosenbrockFunction();
extern ScalarFunction RastriginFunction();
extern ScalarFunction SquareFunction();
extern ScalarFunction SumFunction();
extern ScalarFunction HoleFunction();
extern ScalarFunction ChoiceFunction();

//===========================================================================

struct RandomLPFunction:ConstrainedProblemMix {
  arr randomG;
  virtual double fc(arr& phi, arr& J, TermTypeA& tt, const arr& x) {
    if(!randomG.N){
      randomG.resize(5*x.N+5,x.N+1);
      rndGauss(randomG, 1.);
      for(uint i=0;i<randomG.d0;i++){
        if(randomG(i,0)>0.) randomG(i,0)*=-1.; //ensure (0,0) is feasible
        randomG(i,0) -= .2;
      }
    }
    CHECK(randomG.d1==x.N+1,"you changed dimensionality!");

    phi.clear();
    tt.clear();
    if(&J) J.clear();

    phi.append() = sum(x); tt.append(fTT);
    if(&J) J.append(ones(1,x.N));

    phi.append( randomG * cat({1.},x) );
    if(&J) J.append( randomG.sub(0,-1,1,-1) );
    if(&J) J.reshape(J.N/x.N, x.N);
  }
};

//===========================================================================

struct ChoiceConstraintFunction:ConstrainedProblemMix {
  enum WhichConstraint { wedge2D=1, halfcircle2D, randomLinear, circleLine2D } which;
  uint n;
  arr randomG;
  ChoiceConstraintFunction() {
    which = (WhichConstraint) MT::getParameter<int>("constraintChoice");
    n = MT::getParameter<uint>("dim", 2);
    ConstrainedProblemMix::operator=( [this](arr& phi, arr& J, TermTypeA& tt, const arr& x) -> void {
      this->fc(phi, J, tt, x);
    } );
  }
  void fc(arr& phi, arr& J, TermTypeA& tt, const arr& x) {
    CHECK_EQ(x.N,n,"");
    phi.clear();  if(&tt) tt.clear();  if(&J) J.clear();

    arr H;
    phi.append( ChoiceFunction()(J, H, x) ); if(&tt) tt.append(fTT);
    //HALT("H not used yet!");

    switch(which) {
      case wedge2D:
        for(uint i=0;i<x.N;i++){ phi.append( -sum(x)+1.5*x(i)-.2 ); if(&tt) tt.append(ineqTT); }
        if(&J){ arr Jg(x.N, x.N); Jg=-1.; for(uint i=0;i<x.N;i++) Jg(i,i) = +.5; J.append(Jg); }
        break;
      case halfcircle2D:
        phi.append( sumOfSqr(x)-.25 );  if(&tt) tt.append( ineqTT );  if(&J) J.append( 2.*x ); //feasible=IN circle of radius .5
        phi.append( -x(0)-.2 );         if(&tt) tt.append( ineqTT );  if(&J){ J.append( zeros(x.N) ); J.elem(-x.N) = -1.; } //feasible=right of -.2
        break;
      case circleLine2D:
        phi.append( sumOfSqr(x)-.25 );  if(&tt) tt.append( ineqTT );  if(&J) J.append( 2.*x ); //feasible=IN circle of radius .5
        phi.append( x(0) );             if(&tt) tt.append( eqTT );    if(&J){ J.append( zeros(x.N) ); J.elem(-x.N) = 1.; }
        break;
      case randomLinear:{
        if(!randomG.N){
          randomG.resize(5*x.N+5, x.N+1);
          rndGauss(randomG, 1.);
          for(uint i=0;i<randomG.d0;i++){
            if(randomG(i,0)>0.) randomG(i,0)*=-1.; //ensure (0,0) is feasible
            randomG(i,0) -= .2;
          }
        }
        CHECK_EQ(randomG.d1, x.N+1, "you changed dimensionality");
        phi.append( randomG * cat({1.}, x) );
        if(&tt) tt.append( consts(ineqTT, randomG.d0) );
        if(&J) J.append( randomG.sub(0,-1,1,-1) );
      } break;
    }

    if(&J) J.reshape(J.N/x.N,x.N);
  }
 virtual uint dim_x(){
   return n;
 }
//  virtual uint dim_g(){
//    if(which==randomLinear) return ;
//    if(which==wedge2D) return n;
//    if(which==circleLine2D) return 1;
//    return 2;
//  }
//  virtual uint dim_h(){
//    if(which==circleLine2D) return 1;
//    return 0;
//  }
};

//===========================================================================

struct SimpleConstraintFunction:ConstrainedProblemMix {
  SimpleConstraintFunction(){
    ConstrainedProblemMix::operator=(
      [this](arr& phi, arr& J, TermTypeA& tt, const arr& x) -> void {
      this->fc(phi, J, tt, x);
    } );
  }
  virtual void fc(arr& phi, arr& J, TermTypeA& tt, const arr& _x) {
    CHECK(_x.N==2,"");
    tt = { sumOfSqrTT, sumOfSqrTT, ineqTT, ineqTT };
    phi.resize(4);
    if(&J){ J.resize(4, 2); J.setZero(); }

    //simple squared potential, displaced by 1
    arr x(_x);
    x(0) -= 1.;
    phi.subRange(0,1) = x;
    if(&J) J.setMatrixBlock(eye(2),0,0);
    x(0) += 1.;

    phi(2) = .25-sumOfSqr(x);  if(&J) J[2]() = -2.*x; //OUTSIDE the circle
    phi(3) = x(0);             if(&J) J(3,0) = 1.;
  }
};

//===========================================================================

struct SinusesFunction:VectorFunction {
  double a;
  double condition;
  SinusesFunction() {
    a = MT::getParameter<double>("SinusesFunction_a");
    condition = MT::getParameter<double>("condition");
 NIY 
  }
  virtual void fv(arr& phi, arr& J, const arr& x) {
    CHECK_EQ(x.N,2,"");
    phi.resize(4);
    phi(0) = sin(a*x(0));
    phi(1) = sin(a*condition*x(1));
    phi(2) = 2.*x(0);
    phi(3) = 2.*condition*x(1);
    if(&J) {
      J.resize(4,2);
      J.setZero();
      J(0,0) = cos(a*x(0))*a;
      J(1,1) = cos(a*condition*x(1))*a*condition;
      J(2,0) = 2.;
      J(3,1) = 2.*condition;
    }
  }
};

//===========================================================================

/// $f(x) = x^T C x$ where C has eigen values ranging from 1 to 'condition'
struct SquaredCost : VectorFunction {
  arr M; /// $C = M^T M $
  uint n;  /// dimensionality of $x$
  
  SquaredCost(uint n, double condition=100.);
  void initRandom(uint n, double condition=100.);
  
  void fv(arr& y, arr& J,const arr& x);
};

//===========================================================================

/// Same as SquaredCost but $x_i \gets atan(x_i)$ before evaluating the squared cost
struct NonlinearlyWarpedSquaredCost : VectorFunction {
  uint n;  /// dimensionality of $x$
  SquaredCost sq;
  
  NonlinearlyWarpedSquaredCost(uint n, double condition=100.);
  void initRandom(uint n, double condition=100.);
  
  void fv(arr& y, arr& J,const arr& x);
};

//===========================================================================

struct ParticleAroundWalls:KOrderMarkovFunction {
  //options of the problem
  uint T,k;
  bool hardConstrained, useKernel;

  ParticleAroundWalls():
    T(MT::getParameter<uint>("opt/ParticleAroundWalls/T",1000)),
    k(MT::getParameter<uint>("opt/ParticleAroundWalls/k",2)),
    hardConstrained(MT::getParameter<uint>("opt/ParticleAroundWalls/hardConstrained",true)),
    useKernel(false){}

  //implementations of the kOrderMarkov virtuals
  void phi_t(arr& phi, arr& J, TermTypeA& tt, uint t, const arr& x_bar);
  uint get_T(){ return T; }
  uint get_k(){ return k; }
  uint dim_x(){ return 3; }
  uint dim_phi(uint t);
  uint dim_g(uint t);

  bool isConstrained(){ return hardConstrained; }
  bool hasKernel(){ return useKernel; }
  double kernel(uint t0, uint t1){
    //if(t0==t1) return 1e3;
    return 1e0*::exp(-.001*MT::sqr((double)t0-t1));
  }
};

//===========================================================================

#endif
/// @}
