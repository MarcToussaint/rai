/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "convert.h"
#include "KOMO_Problem.h"

//the Convert is essentially only a ``garbage collector'', creating all the necessary conversion objects and then deleting them on destruction
Convert::Convert(const ScalarFunction& p) : cstyle_fs(nullptr), cstyle_fv(nullptr), data(nullptr), cpm(nullptr) { sf=p; }
Convert::Convert(const VectorFunction& p) : cstyle_fs(nullptr), cstyle_fv(nullptr), data(nullptr), cpm(nullptr) { vf=p; }
//Convert::Convert(KOrderMarkovFunction& p):kom(&p), cstyle_fs(nullptr), cstyle_fv(nullptr), data(nullptr) { }
Convert::Convert(double(*fs)(arr*, const arr&, void*), void* data) : cstyle_fs(fs), cstyle_fv(nullptr), data(data), cpm(nullptr) {  }
Convert::Convert(void (*fv)(arr&, arr*, const arr&, void*), void* data) : cstyle_fs(nullptr), cstyle_fv(fv), data(data), cpm(nullptr) {  }

#ifndef libRoboticsCourse
//Convert::Convert(ControlledSystem& p) { cs=&p; }
#endif

Convert::~Convert() {
  if(cpm) { delete cpm; cpm=nullptr; }
}

//void conv_KOrderMarkovFunction_MathematicalProgram(KOrderMarkovFunction& f, arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x);
double conv_VectorFunction_ScalarFunction(VectorFunction f, arr& g, arr& H, const arr& x) {
  arr y, J;
  f(y, (!!g?J:NoArr), x);
  //  if(J.special==arr::RowShiftedST) J = unpack(J);
  if(!!g) { g = comp_At_x(J, y); g *= 2.; }
  if(!!H) { H = comp_At_A(J); H *= 2.; }
  return sumOfSqr(y);
}

//===========================================================================
//
// casting methods
//

Convert::operator ScalarFunction() {
  if(!sf) {
    if(cstyle_fs) sf = conv_cstylefs2ScalarFunction(cstyle_fs, data);
    else {
      if(!vf) vf = this->operator VectorFunction();
      if(vf)  sf = conv_VectorFunction2ScalarFunction(vf);
    }
  }
  if(!sf) HALT("");
  return sf;
}

Convert::operator VectorFunction() {
  if(!vf) {
    if(cstyle_fv)
      vf = conv_cstylefv2VectorFunction(cstyle_fv, data);
//    else {
//      if(kom) vf = conv_KOrderMarkovFunction2VectorFunction(*kom);
//    }
  }
  if(!vf) HALT("");
  return vf;
}

//Convert::operator KOrderMarkovFunction&() {
//  if(!kom) {
//// #ifndef libRoboticsCourse
////     if(cs) kom = new sConvert::ControlledSystem_2OrderMarkovFunction(*cs);
//// #endif
//  }
//  if(!kom) HALT("");
//  return *kom;
//}

//===========================================================================
//
// actual convertion routines
//

ScalarFunction conv_cstylefs2ScalarFunction(double(*fs)(arr*, const arr&, void*), void* data) {
  return [&fs, data](arr& g, arr& H, const arr& x) -> double {
    if(!!H) NIY;
    return fs(&g, x, data);
  };
}

VectorFunction conv_cstylefv2VectorFunction(void (*fv)(arr&, arr*, const arr&, void*), void* data) {
  return [&fv, data](arr& y, arr& J, const arr& x) -> void {
    fv(y, &J, x, data);
  };
}

ScalarFunction conv_VectorFunction2ScalarFunction(const VectorFunction& f) {
  return [&f](arr& g, arr& H, const arr& x) -> double {
    arr y, J;
    f(y, (!!g?J:NoArr), x);
    //  if(J.special==arr::RowShiftedST) J = unpack(J);
    if(!!g) { g = comp_At_x(J, y); g *= 2.; }
    if(!!H) { H = comp_At_A(J); H *= 2.; }
    return sumOfSqr(y);
  };
}

void Conv_linearlyReparameterize_MathematicalProgram::evaluate(arr& phi, arr& J, const arr& z) {
  arr x = B*z;
  P.evaluate(phi, J, x);
  if(!!J) J = comp_A_x(J, B);
}

//===========================================================================

Convert::Convert(KOMO_Problem& p) : cstyle_fs(nullptr), cstyle_fv(nullptr), data(nullptr), cpm(nullptr) {
  cpm = new Conv_KOMOProblem_MathematicalProgram(p);
}

Convert::operator MathematicalProgram& () {
  if(!cpm) HALT("");
  return *cpm;
}

//===========================================================================

RUN_ON_INIT_BEGIN()
rai::Array<ObjectiveType>::memMove=true;
RUN_ON_INIT_END()
