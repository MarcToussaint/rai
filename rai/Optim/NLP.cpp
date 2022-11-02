/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "NLP.h"
#include "optimization.h"
#include "lagrangian.h"

#include "../Core/util.h"

#include <math.h>

//===========================================================================

template<> const char* rai::Enum<ObjectiveType>::names []= {
  "none", "f", "sos", "ineq", "eq", "ineqB", "ineqP", nullptr
};

//===========================================================================

arr summarizeErrors(const arr& phi, const ObjectiveTypeA& tt) {
  arr err = zeros(3);
  CHECK_EQ(phi.N, tt.N, "");
  for(uint i=0; i<phi.N; i++) {
    double phii = phi.p[i];
    ObjectiveType ot = tt.p[i];
    if(ot==OT_f) err(0) += phii;
    if(ot==OT_sos) err(0) += rai::sqr(phii);
    if((ot==OT_ineq || ot==OT_ineqB) && phii>0.) err(1) += phii;
    if(ot==OT_eq) err(2) += fabs(phii);
  }
  return err;
}

//===========================================================================

arr NLP::getInitializationSample(const arr& previousOptima) {
  arr blo, bup;
  uint n = getDimension();
  getBounds(blo, bup);
  if(!blo.N) {
    return 2.*rand(n)-1.;
  }

  CHECK_EQ(n, blo.N, "");
  CHECK_EQ(n, bup.N, "");
  return blo + rand(n) % (bup - blo);
}

void NLP::report(std::ostream& os, int verbose, const char* msg){
  os <<"NLP of type '" <<rai::niceTypeidName(typeid(*this)) <<"' -- no reporting implemented";
}

double NLP::eval_scalar(arr& g, arr& H, const arr& x){
  arr phi, J;
  evaluate(phi, J, x);

  CHECK_EQ(phi.N, featureTypes.N, "");
  CHECK_EQ(phi.N, J.d0, "");
  CHECK_EQ(x.N, J.d1, "");

  double f=0.;
  for(uint i=0; i<phi.N; i++) {
    if(featureTypes.p[i]==OT_sos) f += rai::sqr(phi.p[i]);
    else if(featureTypes.p[i]==OT_f) f += phi.p[i];
    else HALT("this must be an unconstrained problem!")
  }

  if(!!g) { //gradient
    arr coeff=zeros(phi.N);
    for(uint i=0; i<phi.N; i++) {
      if(featureTypes.p[i]==OT_sos) coeff.p[i] += 2.* phi.p[i];
      else if(featureTypes.p[i]==OT_f) coeff.p[i] += 1.;
    }
    g = comp_At_x(J, coeff);
    g.reshape(x.N);
  }

  if(!!H) { //hessian: Most terms are of the form   "J^T  diag(coeffs)  J"
    arr coeff=zeros(phi.N);
    double hasF=false;
    for(uint i=0; i<phi.N; i++) {
      if(featureTypes.p[i]==OT_sos) coeff.p[i] += 2.;
      else if(featureTypes.p[i]==OT_f) hasF=true;
    }
    arr tmp = J;
    if(!isSparseMatrix(tmp)) {
      for(uint i=0; i<phi.N; i++) tmp[i] *= sqrt(coeff.p[i]);
    } else {
      arr sqrtCoeff = sqrt(coeff);
      tmp.sparse().rowWiseMult(sqrtCoeff);
    }
    H = comp_At_A(tmp); //Gauss-Newton type!

    if(hasF) { //For f-terms, the Hessian must be given explicitly, and is not \propto J^T J
      arr fH;
      getFHessian(fH, x);
      if(fH.N) H += fH;
    }

    if(!H.special) H.reshape(x.N, x.N);
  }

  return f;
}

//===========================================================================

void NLP_Factored::evaluate(arr& phi, arr& J, const arr& x) {
  uintA varDimIntegral = integral(variableDimensions).prepend(0);

  //-- loop through variables and set them
  uint n=0;
  for(uint i=0; i<variableDimensions.N; i++) {
    uint d = variableDimensions(i);
    arr xi = x({n, n+d-1});
    setSingleVariable(i, xi);
    n += d;
  }
  CHECK_EQ(n, x.N, "");

  phi.resize(sum(featureDimensions)).setZero();
  bool resetJ=true;

  //-- loop through features and evaluate them
  n=0;
  arr phi_i, J_i;
  for(uint i=0; i<featureDimensions.N; i++) {
    uint d = featureDimensions(i);
    evaluateSingleFeature(i, phi_i, J_i, NoArr);
    CHECK_EQ(phi_i.N, d, "");
    CHECK_EQ(J_i.d0, d, "");
    phi({n, n+d-1}) = phi_i;
    if(!!J) {
      CHECK(!!J_i, "");
      if(resetJ){
        if(isSparse(J_i)) J.sparse().resize(phi.N, x.N, 0);
        else J.resize(phi.N, x.N).setZero();
        resetJ=false;
      }
      if(J_i.d1 < x.N){ //-- shift the row index!
        uint Jii=0;
        for(uint j=0; j<featureVariables(i).N; j++) {
          int varId = featureVariables(i)(j);
          if(varId>=0) {
            uint varDim = variableDimensions(varId);
            J.setMatrixBlock(J_i.sub(0, -1, Jii, Jii+varDim-1), n, varDimIntegral(varId));
            Jii += varDim;
          }
        }
        CHECK_EQ(Jii, J_i.d1, "");
      }else{
        if(isSparse(J)){
          J_i.sparse().reshape(J.d0, J.d1);
          J_i.sparse().colShift(n);
          J += J_i;
        }else{
          J.setMatrixBlock(J_i, n, 0);
        }
      }
    }
    n += d;
  }
  CHECK_EQ(n, phi.N, "");
}

rai::String NLP_Factored::getVariableName(uint var_id){ return STRING("-dummy-"); }

//===========================================================================

Conv_FactoredNLP_BandedNLP::Conv_FactoredNLP_BandedNLP(const shared_ptr<NLP_Factored>& P, uint _maxBandSize, bool _sparseNotBanded)
  : P(P), maxBandSize(_maxBandSize), sparseNotBanded(_sparseNotBanded) {
  varDimIntegral = integral(P->variableDimensions).prepend(0);
  featDimIntegral = integral(P->featureDimensions).prepend(0);
}

//===========================================================================

void Conv_FactoredNLP_BandedNLP::evaluate(arr& phi, arr& J, const arr& x) {
  CHECK_EQ(x.N, varDimIntegral.last(), "");

  //set all variables
#if 0
  uint n=0;
  for(uint i=0; i<variableDimensions.N; i++) {
    uint d = variableDimensions(i);
    CHECK_EQ(n, varDimIntegral(i), "");
    arr xi = x({n, n+d-1});
    P.setSingleVariable(i, xi);
    n += d;
  }
#else
  P->setAllVariables(x);
#endif

  //evaluate all features individually
  phi.resize(featDimIntegral.last()).setZero();
  arr phi_i;
  J_i.resize(P->featureDimensions.N);
  for(uint i=0; i<P->featureDimensions.N; i++) {
    uint d = P->featureDimensions(i);
    if(d) {
      P->evaluateSingleFeature(i, phi_i, J_i(i), NoArr);
      CHECK_EQ(phi_i.N, d, "");
      if(!!J) CHECK_EQ(J_i.elem(i).d0, d, "");
      phi.setVectorBlock(phi_i, featDimIntegral(i));
    }
  }

  if(!J) return;

  if(sparseNotBanded) {
    //count non-zeros!
    uint k=0;
    for(uint i=0; i<J_i.N; i++) {
      arr& Ji = J_i(i);
      if(!isSparseVector(Ji)) {
        for(uint j=0; j<Ji.N; j++) if(Ji.elem(j)) k++;
      } else {
        k += Ji.sparseVec().Z.N;
      }
    }

    //any sparse matrix is k-times-3
    J.sparse().resize(phi.N, x.N, k);
    J.setZero();

    k=0;
    for(uint i=0; i<J_i.N; i++) { //loop over features
      arr& Ji = J_i(i);
      uint f_dim = P->featureDimensions(i);
      uintA& vars = P->featureVariables(i);
      if(!isSparseVector(Ji)) {
        CHECK(!isSpecial(Ji), "");
        uint c=0;
        for(uint fi=0; fi<f_dim; fi++) { //loop over feature dimension
          for(uint& j:vars) if(j>=0) { //loop over variables of this features
              uint x_dim = P->variableDimensions.elem(j);
              for(uint xi=0; xi<x_dim; xi++) { //loop over variable dimension
                double J_value = Ji.elem(c);
                if(J_value) {
                  J.sparse().entry(featDimIntegral.elem(i)+fi, varDimIntegral.elem(j)+xi, k) = J_value;
                  k++;
                }
                c++;
              }
            }
        }
        CHECK_EQ(c, Ji.N, "you didn't count through all indexes");
      } else { //sparse vector
        for(uint l=0; l<Ji.N; l++) {
          double J_value = Ji.elem(l);
          uint   xi      = Ji.sparseVec().elems(l); //column index
          for(uint& j:P->featureVariables(i)) if(j>=0) {
              uint xj_dim = P->variableDimensions(j);
              if(xi<xj_dim) { //xj is the variable, and xi is the index within the variable
                uint jj = (j?varDimIntegral(j-1):0) + xi;
                J.sparse().entry(i, jj, k) = J_value;
                k++;
              }
              xi -= xj_dim; //xj is not yet the variable, skip over xj, and shift xi
            }
        }
      }
    }
    CHECK_EQ(k, J.N, ""); //one entry for each non-zero

  } else {

    if(!!J) {
      rai::RowShifted& Jaux = J.rowShifted();
      Jaux.resize(phi.N, x.N, maxBandSize); //(k+1)*dim_xmax
      J.setZero();

      for(uint i=0; i<P->featureDimensions.N; i++) {
        uint n = featDimIntegral(i);
        uint d = P->featureDimensions(i);
        if(d) {
          J.setMatrixBlock(J_i(i), n, 0);
          //      memmove(&J(i, 0), J_i.p, J_i.sizeT*J_i.N);
//          intA& vars = featureVariables(i);
//          int t=vars.first();
          //      uint xdim=variableDimensions(t);
          //      for(int s=1;s<vars.N;s++){ xdim+=variableDimensions(t+s); CHECK_EQ(vars(s), t+s, ""); }
          //      CHECK_EQ(xdim, J_i.d1, "");

          if(!isRowShifted(J_i(i))){
            HALT("perhaps the below needs to be enabled!");
            //          for(uint j=n; j<n+d; j++) {
            //            if(t<=0) Jaux.rowShift(j) += 0;
            //            else Jaux.rowShift(j) += varDimIntegral(t);
            //          }
          }
        }
      }
      Jaux.reshift();
      Jaux.computeColPatches(true);
    }
  }

  P->report(cout, 0);
}

//===========================================================================

void NLP_Traced::evaluate(arr& phi, arr& J, const arr& x) {
  evals++;
  P->evaluate(phi, J, x);
  if(trace_x){ xTrace.append(x); xTrace.reshape(-1, x.N); }
  if(trace_costs){ costTrace.append(summarizeErrors(phi, featureTypes)); costTrace.reshape(-1,3);  }
  if(trace_phi && !!phi) { phiTrace.append(phi);  phiTrace.reshape(-1, phi.N); }
  if(trace_J && !!J) { JTrace.append(J);  JTrace.reshape(-1, phi.N, x.N); }
}

void NLP_Traced::report(std::ostream& os, int verbose, const char* msg){
  os <<"TRACE: #evals: " <<evals;
  if(costTrace.N) os <<" costs: " <<costTrace[-1];
  if(xTrace.N && xTrace.d1<10) os <<" x: " <<xTrace[-1];
  os <<endl;
}

//===========================================================================

void NLP_Viewer::display(double mu){
  uint d = P->getDimension();
  CHECK_EQ(d, 2, "can only display 2D problems for now");

  //-- get bounds
  arr lo, up;
  P->getBounds(lo, up);
  if(!lo.N) lo = -ones(2);
  if(!up.N) up = ones(2);

  //-- make grid
  arr X, Y, phi;
  X.setGrid(2, 0., 1., 100);
  for(uint i=0;i<X.d0;i++){ X[i] = lo + (up-lo)%X[i]; }
  Y.resize(X.d0);

  //-- transform constrained problem to AugLag scalar function
  P->evaluate(phi, NoArr, X[0]);
  std::shared_ptr<LagrangianProblem> lag;
  std::shared_ptr<NLP> nlp_save;
  if(phi.N>1){
    lag = make_shared<LagrangianProblem>(P);
    lag->mu = lag->nu = mu;
    nlp_save = P;
    P.reset();
    P = make_shared<Conv_ScalarProblem_NLP>(*lag, d);
  }

  //-- evaluate over the grid
  for(uint i=0; i<X.d0; i++) {
    P->evaluate(phi, NoArr, X[i]);
    CHECK_EQ(phi.N, 1, "only 1 feature for now");
    double fx=phi.scalar();
    Y(i) = ((fx==fx && fx<10.)? fx : 10.);
  }
  Y.reshape(101, 101);

  //-- plot
  //  plot()->Gnuplot();  plot()->Surface(Y);  plot()->update(true);
  FILE("z.fct") <<Y.modRaw() <<endl;

  rai::String cmd;
  cmd <<"reset; set contour; set cntrparam linear; set cntrparam levels incremental 0,.1,10; set xlabel 'x'; set ylabel 'y'; ";
  rai::String splot;
  splot <<"splot [" <<lo(0) <<':' <<up(0) <<"][" <<lo(1) <<':' <<up(1) <<"] "
       <<"'z.fct' matrix us (" <<lo(0) <<"+(" <<up(0)-lo(0) <<")*$2/100):(" <<lo(1) <<"+(" <<up(1)-lo(1) <<")*$1/100):3 w l";
  if(!T){
    cmd <<splot <<";";
  }else{
    T->report(cout, 0);
    if(false && T->costTrace.N){
      FILE("z.trace") <<catCol(T->xTrace, T->costTrace.col(0)).modRaw();
      cmd <<splot <<", 'z.trace' us 1:2:3 w lp; ";
    }else{
      FILE("z.trace") <<T->xTrace.modRaw();
      cmd <<"unset surface; set table 'z.table'; ";
      cmd <<splot <<"; ";
      cmd <<"unset table; ";
      cmd <<"plot 'z.table' w l, 'z.trace' us 1:2 w lp lw 2; ";
    }
  }
  cout <<cmd <<endl;
  gnuplot(cmd);
}

void NLP_Viewer::plotCostTrace(){
  CHECK(T, "");
  FILE("z.trace") <<T->costTrace.modRaw();
  rai::String cmd;
  cmd <<"reset; set xlabel 'evals'; set ylabel 'objectives'; set style data lines;";
  cmd <<"plot 'z.trace' us ($0+1):1 t 'f+sos', '' us ($0+1):2 t 'ineq', '' us ($0+1):3 t 'eq';";
  gnuplot(cmd);
}
