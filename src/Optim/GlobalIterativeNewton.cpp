#include "GlobalIterativeNewton.h"

bool useNewton=true;

GlobalIterativeNewton::GlobalIterativeNewton(arr& x, const ScalarFunction& f, const arr& bounds_lo, const arr& bounds_hi, OptOptions opt)
  : newton(x, f, opt),
    grad(x, f, opt),
    bounds_lo(bounds_lo), bounds_hi(bounds_hi),
    best(NULL) {
}

GlobalIterativeNewton::~GlobalIterativeNewton(){
}

void addRun(GlobalIterativeNewton& gin, const arr& x, double fx, double tol){
  GlobalIterativeNewton::LocalMinimum *found=NULL;
  for(GlobalIterativeNewton::LocalMinimum& m:gin.localMinima){
    double d = euclideanDistance(x, m.x);
    if(euclideanDistance(x,m.x)<tol){
      if(!found) found = &m;
      else if(d<euclideanDistance(x,found->x)) found = &m;
    }
  }

  if(found){
    found->hits++;
    if(fx<found->fx){
      found->x = x;
      found->fx = fx;
    }
  }else{
    gin.localMinima.append( {x, fx, 1} );
    found = &gin.localMinima.last();
  }

  if(!gin.best || found->fx<gin.best->fx) gin.best=found;
  gin.newton.x = gin.best->x;
  gin.newton.fx = gin.best->fx;
  if(gin.newton.o.verbose>1) cout <<"***** optGlobalIterativeNewton: local minimum: " <<found->hits <<' ' <<found->fx <<' ' <<found->x <<endl;
}

void GlobalIterativeNewton::step(){

  arr x = bounds_lo + (bounds_hi-bounds_lo) % rand(bounds_lo.N);

  if(newton.o.verbose>1) cout <<"***** optGlobalIterativeNewton: new iteration from x=" <<x <<endl;

  if(useNewton){
    newton.reinit(x);
    newton.run();
    addRun(*this, newton.x, newton.fx, 3.*newton.o.stopTolerance);
  }else{
    grad.reinit(x);
    grad.run();
    addRun(*this, grad.x, grad.fx, 3.*grad.o.stopTolerance);
  }
}

void GlobalIterativeNewton::run(uint maxIt){
  for(uint i=0;i<maxIt;i++){
    step();
  }
}

void GlobalIterativeNewton::report(){
  cout <<"# local minima = " <<localMinima.N <<endl;
  uint i=0;
  for(LocalMinimum& m:localMinima){
    cout <<i++ <<' ' <<m.hits <<' ' <<m.fx <<" \t" <<m.x <<endl;
  }

}

