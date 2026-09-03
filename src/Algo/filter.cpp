#include "filter.h"

#include <cmath>

namespace rai{

void KLinearFilter::update(double t_now, const arr& x_now){
  CHECK(x_now.nd==1, "");
  if(!x.N){ //first time
    x.resize(K, x_now.N);
    for(uint k=0;k<K;k++) x[k] = x_now;
    t.resize(K) = t_now;
  }else{
    x.shift(x.d1, false);
    x[0] = x_now;
    for(uint i=0;i<x.d1;i++){
      double d=x(0,i)-x(1,i);
      if(!std::isnan(d) && fabs(d)>threshold){
        LOG(0) <<"above threshold: " <<i <<' ' <<d;
        x(0,i)=std::nan("");
      }
    }
    t.shift(1, false);
    t(0) = t_now;
  }
}

arr KLinearFilter::get_x(double t_now){
  if(!x.N) return x;
#if 0 //mean constant
  return ::sum(x, 0)/K;
#elif 1 //linear in last two
  if(t_now>t(0)) return x[0];
  if(t_now<t(1)) return x[1];
  double a = (t_now-t(1))/(t(0)-t(1));
  return a*x[0] + (1.-a)*x[1]; //returns nan if one of them is nan (elem-wise)
#else
  BSpline S;
  S.setKnots(2, t);
  arr B = S.getBmatrix({t_now}, true, true);
  return (B*x).reshape(x.d1);
#endif

}

}//namespace
