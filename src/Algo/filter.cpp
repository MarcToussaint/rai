#include "filter.h"

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
  reg_is_good = false;
}

arr KLinearFilter::get_x(double t_now){
    if(!x.N) return x;
#if 0
    return sum(x, 0)/K;
#elif 1
    if(t_now>t(0)) return x[0];
    if(t_now<t(1)) return x[1];
    double a = (t_now-t(1))/(t(0)-t(1));
    return a*x[0] + (1.-a)*x[1];
#else
  if(!reg_is_good){
    //linear regression
    t_mean = sum(t)/K;
    x_mean = sum(x, 0)/K;
    double t_var  = sum(t%t)/K - t_mean*t_mean;
    arr xt_cov = sum(t%x, 0)/K - t_mean*x_mean;
    beta = xt_cov/(t_var + 1e-6);
    reg_is_good = true;
  }
  return x_mean + beta*(t_now-t_mean);
#endif

}
