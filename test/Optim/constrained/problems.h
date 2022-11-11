#include <Optim/NLP.h>

#include <math.h>

struct CoveringSpheresProblem : NLP {
  const arr& x;
  double p, alpha;
  uint s;

  CoveringSpheresProblem(const arr& x, uint s):x(x), p(2.), alpha(-10.), s(s) {
  }

  arr initialization(const arr& x){
    arr c(s, 3);
    for(uint j=0;j<s;j++) c[j] = x[rnd(x.d0)];

    arr r=zeros(s);
    for(uint i=0;i<x.d0;i++) for(uint j=0;j<s;j++){
      double d=length(x[i] - c[j]);
      if(2.*d>r(j)) r(j) = 2.*d;
    }

//    r = .1;
    return (c,r);
  }

  virtual void getFeatureTypes(ObjectiveTypeA& ft){ NIY }
  virtual void evaluate(arr& phi, arr& J, const arr& x){ NIY }

  virtual double phi(arr& df, arr& Hf, arr& g, arr& Jg, const arr& c_r) {
    uint s=c_r.N/4;
    arr c,r;
    c.referToRange(c_r,0,3*s-1); c.reshape(s,3);
    r.referToRange(c_r,3*s,-1); r.reshape(s);

    //f
    double fx = 0.;
    for(uint i=0;i<s;i++) fx += pow(r(i), p);
    if(!!df){
      df=zeros(4*s);
      for(uint i=0;i<s;i++) df(3*s + i) = p * pow(r(i), p-1.);
    }
    if(!!Hf){
      Hf=zeros(4*s, 4*s);
      for(uint i=0;i<s;i++) Hf(3*s + i, 3*s + i) = p * (p-1.) * pow(r(i), p-2.);
    }

    //g
    arr d_ij(x.d0, s), ed_ij(x.d0, s), sed_i(x.d0);
    if(!!g || !!Jg){
      for(uint i=0;i<x.d0;i++) for(uint j=0;j<s;j++)
        d_ij(i,j) = length(x[i] - c[j]) - r(j);
      ed_ij = ::exp(alpha*d_ij);
      sed_i = sum(ed_ij,1);
    }
    if(!!g){
      g = sum((d_ij%ed_ij),1) / sed_i;
    }
    if(!!Jg){
      arr Jg_dij = (ed_ij%(1.+alpha*(d_ij - repmat(g,1,s))));
      for(uint i=0;i<x.d0;i++) Jg_dij[i] /= sed_i(i);

      arr Jg_cj(x.d0, s, 3);
      for(uint i=0;i<x.d0;i++) for(uint j=0;j<s;j++){
        Jg_cj[i][j] = Jg_dij(i,j) * (c[j]-x[i])/length(x[i]-c[j]);
      }

      arr Jg_rj = -Jg_dij;
      Jg_cj.reshape(x.d0,3*s);
      Jg = catCol(Jg_cj, Jg_rj);
      Jg.reshape(x.d0,c_r.d0);
    }
    return fx;
  }
  virtual uint dim_x(){ return 4*s;  }
  virtual uint dim_g(){ return x.d0; }
};
