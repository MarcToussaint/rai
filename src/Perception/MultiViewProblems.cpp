#include "MultiViewProblems.h"

void MultiViewPointData::setCamera(uint k, const arr& P_k){
  if(!P.N) P = zeros(K,3,4);
  if(P.d0==3) P[k] = P_k;
  else P[k] = P_k({0,3});
}

void MultiViewPointData::setCamera(uint k, const arr& fxycxy, const rai::Transformation& X){
  arr Tinv = X.getInverseMatrix();
  arr P = zeros(4,4);
  P(0, 0) = fxycxy(0);
  P(1, 1) = fxycxy(1);
  P(0, 2) = fxycxy(2);
  P(1, 2) = fxycxy(3);
  P(2, 2) = 1.;
  P(3, 3) = 1.; //homogeneous 3D is kept
  P = P * Tinv;
  setCamera(k, P);
}

void MultiViewPointData::addDataPoint(uint j, uint k, const arr& p, double d){
  data.append(Obs{j,k,p,d});
}

void MultiViewPointData::subSelectObservedPoints(){
  //count how often each point is observed
  uintA count(J);
  count.setZero();
  for(Obs& obs:data) count(obs.j)++;
  //select count>=2
  selectedJ=0;
  selectedJidx.resize(J) = -1;
  for(uint j=0,n=0;j<J;j++) if(count(j)>=2){ selectedJidx(j)=selectedJ; selectedJ++; }
  //select respective data
  selectedI;
  for(uint i=0;i<data.N;i++) if(selectedJidx(data(i).j)>=0) selectedI.append(i);
}

arr& MultiViewPointData::solveColinearityForPoints(){
  //-- build system
  arr A, a;
  for(uint i:selectedI){
    Obs& o = data(i);
    arr x = {o.p.elem(0), o.p.elem(1), 1.};
    arr skew_x = skew(x);
    arr B = zeros(3, 3*selectedJ);
    for(uint i=0;i<3;i++) B(i, 3*selectedJidx(o.j)+i) = 1.;
    arr P3 = P[o.k].sub({0,3}, {0,3});
    A.append(skew_x * P3 * B);
    arr P4 = P[o.k].sub({0,3}, {3,0}).reshape(3);
    a.append(skew_x * P4);
  }
  //-- solve
  arr At = ~A;
  arr At_A = ~A*A;
  double lambda = 1e-2;
  for(uint i=0;i<At_A.d0;i++) At_A(i,i) += lambda;
  arr Xsel = -inverse_SymPosDef(At_A)*At*a;
  Xsel.reshape(-1, 3);
  //-- copy to X buffer
  if(!X.N) X = zeros(J, 3);
  for(uint j=0;j<J;j++) if(selectedJidx(j)>=0){
      X[j] = Xsel[selectedJidx(j)];
    }
  return X;
}
