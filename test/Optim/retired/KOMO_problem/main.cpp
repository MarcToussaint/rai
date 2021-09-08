#include <Core/util.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <Optim/convert.h>
#include <Optim/constrained.h>

// the kernel stuff is preliminary -- please igore everything related to kernels so far
#if 0
arr buildKernelMatrix(KOrderMarkovFunction& P){
  CHECK(P.hasKernel(),"");
  uint T = P.get_T();
  uint n = P.dim_x();
  arr K((T+1)*n,(T+1)*n);
  K.setZero();
  for(uint t=0;t<=T;t++){
    for(uint s=t;s<=T;s++){
      double kts=P.kernel(t,s);
      for(uint i=0;i<n;i++){
        K(t*n+i,s*n+i) = kts;
        K(s*n+i,t*n+i) = kts;
      }
    }
  }
  for(uint i=0;i<K.d0;i++) K(i,i) += 1e-10;
  arr Kinv;
  inverse_SymPosDef(Kinv, K);
  return Kinv;
}

void TEST(KOrderMarkov) {
  //see the implemention of ParticleAroundWalls::phi_t for an example on how to specify constrained k-order-Markov optimization problems
  ParticleAroundWalls P;

  //-- print some info on the P
  uint T=P.get_T();
  uint k=P.get_k();
  uint n=P.dim_x(0);
  cout <<"P parameters:"
       <<"\n T=" <<T
       <<"\n k=" <<k
       <<"\n n=" <<n
       <<endl;

  //-- gradient check: this is slow!
  arr x(T+1,n);
  for(uint k=0;k<0;k++){
    rndUniform(x,-1.,1.);
    checkJacobianCP(Convert(P),  x, 1e-3);
  }

  //-- optimize
  rndUniform(x,-1.,1.);
  arr K;
  if(P.hasKernel()) K = buildKernelMatrix(P);
  optConstrained(x, NoArr, Convert(P) );

  write(LIST<arr>(x),"z.output");
  rai::String plt = "plot 'z.output' us 1";
  for(uint i=1;i<n;i++) plt <<", '' us " <<i+1;
  gnuplot(plt, true, true);
}
#endif

void TEST(KOrderMarkov2) {
  //see the implemention of ParticleAroundWalls::phi_t for an example on how to specify constrained k-order-Markov optimization problems
  ParticleAroundWalls2 P;

  //-- print some info on the P
  uint T=P.get_T();
  uint k=P.get_k();
  uintA d, times;
  P.getStructure(d, times, NoObjectiveTypeA);
  cout <<"P parameters:"
       <<"\n T=" <<T
       <<"\n k=" <<k
       <<"\n n=" <<d(0)
       <<endl;

  //-- gradient check: this is slow!
  arr x(sum(d));
  x.reshape(T,d(0));
  for(uint k=0;k<0;k++){
    rndUniform(x,-1.,1.);
    P.checkStructure(x);
    checkJacobianCP(Convert(P), x, 1e-3);
  }

  //-- optimize
  rndUniform(x,-1.,1.);
  arr K;
//  if(P.hasKernel()) K = buildKernelMatrix(P);
  OptConstrained(x, NoArr, Convert(P) )
          .run();

  write(LIST<arr>(x),"z.output");
  rai::String plt = "plot 'z.output' us 1";
  for(uint i=1;i<d(0);i++) plt <<", '' us " <<i+1;
  gnuplot(plt, true, true);
}

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  //  testKOrderMarkov();
  testKOrderMarkov2();

  return 0;
}

