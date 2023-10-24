/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

//===========================================================================
//added Sep 2013

//-- return types
//struct     SqrPotential { arr A, a;          double c; }; ///< return type representing \f$x'A x - 2x'a + c\f$
//struct PairSqrPotential { arr A, B, C, a, b; double c; }; ///< return type representing \f$(x,y)' [A C, C' B] (x,y) - 2(x,y)'(a,b) + c\f$
//extern SqrPotential& NoPot; ///< reference to nullptr! used for optional arguments
//extern PairSqrPotential& NoPairPot; ///< reference to nullptr! used for optional arguments

/// a locally quadratic function TODO: replace by scalar with hessian!
//struct QuadraticFunction { virtual double fq(SqrPotential& S, const arr& x) = 0; };

///// functions \f$f_i:x_i \mapsto y\f$ and \f$f_{ij}: x_i,x_j \mapsto y\f$ over a chain \f$x_0,..,x_T\f$ of variables
struct VectorChainFunction {
  virtual uint get_T() = 0;
  virtual void fv_i(arr& y, arr& J, uint i, const arr& x_i) = 0;
  virtual void fv_ij(arr& y, arr& Ji, arr& Jj, uint i, uint j, const arr& x_i, const arr& x_j) = 0;
};

//struct QuadraticChainFunction {
//  virtual uint get_T() = 0;
//  virtual double fq_i(SqrPotential& S, uint i, const arr& x_i) = 0;
//  virtual double fq_ij(PairSqrPotential& S, uint i, uint j, const arr& x_i, const arr& x_j) = 0;
//};

/*
struct VectorGraphFunction {
  virtual uintA edges() = 0;
  virtual double fi (arr* grad, uint i, const arr& x_i) = 0;
  virtual double fij(arr* gradi, arr* gradj, uint i, uint j, const arr& x_i, const arr& x_j) = 0;
  double f_total(const arr& X);
};*/

struct VectorChainCost:VectorChainFunction {
  uint T, n;
  arr A, a;
  arr Wi, Wj, w;
  bool nonlinear;

  VectorChainCost(uint _T, uint _n);
  uint get_T() { return T; }
  void fv_i(arr& y, arr* J, uint i, const arr& x_i);
  void fv_ij(arr& y, arr* Ji, arr* Jj, uint i, uint j, const arr& x_i, const arr& x_j);
};

//===========================================================================

struct SlalomProblem:VectorChainFunction {
  uint T, K, n;
  double margin, w, power;

  SlalomProblem(uint _T, uint _K, double _margin, double _w, double _power);
  uint get_T() { return T; }
  void fv_i(arr& y, arr& J, uint i, const arr& x_i);
  void fv_ij(arr& y, arr& Ji, arr& Jj, uint i, uint j, const arr& x_i, const arr& x_j);
};

//===========================================================================

#include "optimization.h"

struct OptimizationProblem {
  bool isVectorValued;
  uint N;
  arr x;
  //virtual void model(arr& output, const arr& input, const arr& x, BinaryBPNet& bp){NIY;}
  virtual double loss(const arr& x, uint i, arr* grad, double* err) {NIY;} ///< loss and gradient for i-th datum and parameters x
  virtual double totalLoss(const arr& x, arr* grad, double* err) {NIY;} ///< loss and gradient for i-th datum and parameters x

  virtual double f(arr* grad, const arr& x, int i=-1) {NIY;}   ///< scalar valued function
  virtual void   F(arr& F, arr* grad, const arr& x, int i=-1) {NIY;} ///< vector valued function
  OptimizationProblem() { N=0; }
};

struct DecideSign {
  double sumX, sumXX;
  uint N;
  void init() { N=0; sumX=sumXX=0.; }
  bool step(double x);
  double mean() { return sumX/N; }
  double sdv() { double m=mean(); return sqrt((sumXX+2.*m*m)/N-m*m); }
  double sign() { return rai::sign(sumX); }
};

struct SGD {
  uint t, N;
  arr w1, w2;
  OptimizationProblem* m;
  double a1, a2, l1, l2, e1, e2;
  uintA perm;
  ofstream log;

#define BATCH 1000
#define UP 2.
#define DOWN 0.3

  void init(OptimizationProblem* _m, double initialRate, uint _N, const arr& w0) {
    t=0;
    m=_m;
    a1=a2=initialRate;
    a2 *= UP;
    N=_N;
    perm.setRandomPerm(N);
    w1=w2=w0;
    l1=l2=0.;
    e1=e2=0.;
    rai::open(log, "log.sgd");
  }

  void stepPlain() {
    arr grad;
    double err;
    l1 += m->loss(w1, perm(t%N), &grad, &err);   w1 -= a1 * grad;   e1+=err;
    log <<t
        <<" time= " <<rai::timerRead()
        <<" loss1= " <<l1/(t%BATCH+1)
        <<" err1= "  <<e1/(t%BATCH+1)
        <<" rate1= " <<a1
        <<endl;
    cout <<t
         <<" time= " <<rai::timerRead()
         <<" loss1= " <<l1/(t%BATCH+1)
         <<" err1= "  <<e1/(t%BATCH+1)
         <<" rate1= " <<a1
         <<endl;
    t++;
    if(!(t%N)) perm.setRandomPerm(N);
    if(!(t%BATCH)) {
      l1=l2=0.;
      e1=e2=0.;
    }
  }

  void stepTwin() {
    arr grad;
    double err;
    l1 += m->loss(w1, perm(t%N), &grad, &err);   w1 -= a1 * grad;   e1+=err;
    l2 += m->loss(w2, perm(t%N), &grad, &err);   w2 -= a2 * grad;   e2+=err;
    log <<t
        <<" time= " <<rai::timerRead()
        <<" loss1= " <<l1/(t%BATCH+1) <<" loss2= " <<l2/(t%BATCH+1)
        <<" err1= "  <<e1/(t%BATCH+1) <<" err2= "  <<e2/(t%BATCH+1)
        <<" rate1= " <<a1 <<" rate2= " <<a2
        <<endl;
    cout <<t
         <<" time= " <<rai::timerRead()
         <<" loss1= " <<l1/(t%BATCH+1) <<" loss2= " <<l2/(t%BATCH+1)
         <<" err1= "  <<e1/(t%BATCH+1) <<" err2= "  <<e2/(t%BATCH+1)
         <<" rate1= " <<a1 <<" rate2= " <<a2
         <<endl;
    t++;
    if(!(t%N)) perm.setRandomPerm(N);
    if(!(t%BATCH)) {
      if(l1<=l2) {  a2=a1;  w2=w1;  } else     {  a1=a2;  w1=w2;  }
      a1 *= DOWN; a2 *= UP;
      l1=l2=0.;
      e1=e2=0.;
    }
  }
};

inline double ModelStaticL(const arr& w, void* p) {    return ((OptimizationProblem*)p)->totalLoss(w, nullptr, nullptr); }
inline void   ModelStaticDL(arr& grad, const arr& w, void* p) {((OptimizationProblem*)p)->totalLoss(w, &grad, nullptr); }
//void   ModelStaticF (arr& out , const arr& w, void* p){ ((OptimizationProblem*)p)->f(out, w); }
// void   ModelStaticDF(arr& grad, const arr& w, void* p){ ((OptimizationProblem*)p)->df(grad, w); }

// void checkGrad_loss(OptimizationProblem& m, const arr& w, double tolerance){
//   checkGradient(ModelStaticL, ModelStaticDL, &m, w, tolerance);
// }

// void checkGrad_fvec(OptimizationProblem& m, const arr& w, double tolerance){
//   checkGradient(ModelStaticF, ModelStaticDF, &m, w, tolerance);
// }

//===========================================================================
//
// Online Rprop
//

struct OnlineRprop {
  Rprop rprop;
  uint t, N;
  arr w;
  double l, e;
  OptimizationProblem* m;
  uintA perm;
  ofstream log;
  rai::Array<DecideSign> signer;

  void init(OptimizationProblem* _m, double initialRate, uint _N, const arr& w0);
  void step();
};
#undef BATCH
#undef UP
#undef DOWN
