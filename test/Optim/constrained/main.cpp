#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include "problems.h"
#include <Optim/constrained.h>

//lecture.cpp:
void lectureDemo(const shared_ptr<MathematicalProgram>& P, const arr& x_start=NoArr, uint iters=20);

//==============================================================================
//
// test standard constrained optimizers
//

void testConstraint2(MathematicalProgram& p, arr& x_start=NoArr){
  //-- initial x
  arr x = p.getInitializationSample();
  if(!!x_start) x=x_start;

  OptConstrained(x, NoArr, p.ptr())
      .run();

  if(!!x_start) x_start = x;
}

//==============================================================================
//
// test the phase one optimization
//

void testPhaseOne(const shared_ptr<MathematicalProgram>& f, uint dim_x){
  auto metaF = make_shared<PhaseOneProblem>(f);

  arr x;
  x = {1., 1., 10.};

  lectureDemo(metaF, x, 1);
  //one iteration of phase one should be enough
  //properly done: check in each step if constraints are fulfilled and exit phase one then
  //no need to really minimize

  x=x.sub(0,-2);
  lectureDemo(f, x);
}

//==============================================================================

void TEST(CoveringSphere){
  uint n=100, s=10;
  arr x(n,3); rndGauss(x);
  CoveringSpheresProblem F(x,s);

  //-- initial x
  arr cr = F.initialization(x);
  rndGauss(cr, .01);

  cout <<"point = " <<x <<endl;
  cout <<"cr_init=" <<cr <<endl;
  checkJacobianCP(F, cr, 1e-4);
  OptConstrained(cr, NoArr, F.ptr())
      .run();
  cout <<"cr_opt=" <<cr <<endl;
}

//===========================================================================

void TEST(MathematicalProgram){
  auto P = make_shared<MP_TrivialSquareFunction>(2, 1., 2.);

  arr x, phi;
  x = P->getInitializationSample();

  P->evaluate(phi, NoArr, x);
  cout <<x <<endl <<phi;

  //Conv_MathematicalProgram_ConstrainedProblem F(P);
  checkJacobianCP(*P, x, 1e-4);

  OptConstrained opt(x, NoArr, P, OptOptions().set_verbose(6));
  P->getBounds(opt.newton.bounds_lo, opt.newton.bounds_up);
  opt.run();

  cout <<"optimum: " <<x <<endl;

}

//==============================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  ChoiceConstraintFunction F;
//  RandomLPFunction F;
//  SimpleConstraintFunction F;
  lectureDemo(F.ptr(), {.2,.2});
//  testConstraint2(F);

//  testCoveringSphere();
//  testMathematicalProgram();

  return 0;
}
