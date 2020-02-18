#include <Core/util.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>

#include <Optim/Graph_Problem.h>
#include <Optim/constrained.h>

void testGraphProblem() {
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

  //-- convert to GraphProblem
  //  KOMO_GraphProblem G(P);

  //-- convert directly to ConstrainedProblem (efficient)
  Conv_KOMO_ConstrainedProblem C2(P); //(direct conversion - just for testing..)

  //-- convert to unstructued ConstrainedProblem (not efficient yet)
  //  Conv_Graph_ConstrainedProblem C(G);

  //-- gradient check: this is slow!
  arr x(sum(d));
  x.reshape(T,d(0));
  for(uint k=0;k<0;k++){
    rndUniform(x,-1.,1.);
    P.checkStructure(x);
    //    checkJacobianCP(C, x, 1e-3);
    checkJacobianCP(C2, x, 1e-3);
  }

  //-- optimize
  rndUniform(x,-1.,1.);
  arr K;
//  optConstrained(x, NoArr, C ); //slow, because structure not exploited
  optConstrained(x, NoArr, C2 ); //fast, using KOMO structure

  write(LIST<arr>(x),"z.output");
  rai::String plt = "plot 'z.output' us 1";
  for(uint i=1;i<d(0);i++) plt <<", '' us " <<i+1;
  gnuplot(plt, true, true);
}

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  testGraphProblem();

  return 0;
}

