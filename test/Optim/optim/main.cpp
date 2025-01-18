#include <Optim/benchmarks.h>
#include <functional>
#include <Optim/NLP_Solver.h>
#include <Optim/NLP_Sampler.h>
#include <Optim/lagrangian.h>
#include <Optim/constrained.h>
#include <Core/arrayDouble.h>
#include <Optim/opt-ipopt.h>

#include <Kin/kin.h>
#include <Kin/frame.h>

//===========================================================================

void TEST(Display) {
  std::shared_ptr<NLP> nlp = getBenchmarkFromCfg();

  NLP_Viewer(nlp).display();

  rai::wait();
}

//===========================================================================

void TEST(Solver) {
  std::shared_ptr<NLP> nlp = getBenchmarkFromCfg();

  rai::Enum<NLP_SolverID> sid (rai::getParameter<rai::String>("solver"));
  if(sid==NLPS_logBarrier){
    NLP_Viewer(nlp).display(1., 1.);
  }else{
    NLP_Viewer(nlp).display(1., -1.);
  }
  rai::wait(.5, true);

  //  arr x = nlp->getInitializationSample();
  //  checkJacobianCP(*nlp, x, 1e-4);

  arr x_init = rai::getParameter<arr>("x_init", {});
  NLP_Solver S;

  S.setSolver(sid);
  S.setProblem(nlp);
  if(x_init.N) S.setInitialization(x_init);
  if(sid==NLPS_augmentedLag || sid==NLPS_squaredPenalty || sid==NLPS_logBarrier){
    while(!S.step()){
      if(sid==NLPS_logBarrier){
        NLP_Viewer(nlp, S.P). display(S.optCon->L.mu, S.optCon->L.muLB);
      }else{
        NLP_Viewer(nlp, S.P). display(S.optCon->L.mu);
      }
      if(S.opt.verbose>2) rai::wait(.2, true);
    }
  }else{
    S.solve();
  }

  arr path = catCol(S.getTrace_x(), S.getTrace_costs());
  FILE("z.path") <<path.modRaw();

  cout <<"\nRESULT:\n" <<*S.ret <<"\nx: " <<S.ret->x <<"\ndual: " <<S.ret->dual <<endl;

  NLP_Viewer(nlp, S.P). display();
  // displayNLP(nlp, S.getTrace_x(), S.getTrace_costs());
  //  gnuplot("load 'plt'", false, false);
  rai::wait();
}

//===========================================================================

//a set of spheres, confined in a box, and no collision, minimizing their height..
struct SpherePacking : NLP{
  arr x; //position of spheres
  uint n;
  double rad;
  bool ineqAccum;

  rai::Configuration disp;

  SpherePacking(uint _n, double _rad, bool _ineqAccum=false) : n(_n), rad(_rad), ineqAccum(_ineqAccum) {
    dimension = 3*n;

    bounds.resize(2, n, 3);
    for(uint i=0;i<n;i++){
      bounds(0, i, 0) = -1.+rad;
      bounds(1, i, 0) = +1.-rad;
      bounds(0, i, 1) = -1.+rad;
      bounds(1, i, 1) = +1.-rad;
      bounds(0, i, 2) = 0.+rad;
      bounds(1, i, 2) = +16.-rad;
    }
    bounds.reshape(2, dimension);

    featureTypes.clear();
    if(!ineqAccum){
      featureTypes.append(rai::consts<ObjectiveType>(OT_ineq, n*(n-1)/2));
    }else{
      featureTypes.append(OT_eq);
    }
    featureTypes.append(rai::consts<ObjectiveType>(OT_f, n));
  }

  void ineqAccumulation(uint phiIdx, arr& phi, arr& J, arr& g, const arr& Jg){
    CHECK_EQ(g.N, Jg.d0, "");
    CHECK_EQ(J.d1, Jg.d1, "");
    CHECK_GE(phi.N, phiIdx, "");

    // g += .01;

    for(uint i=0; i<g.N; i++) if(g(i)>0.) {  //ReLu for g
#if 0
        phi.elem(phiIdx) += g(i);
        J.sparse().add(Jg.sparse().getSparseRow(i), phiIdx, 0);
#elif 1
	double delta=.1;
	if(g(i)>delta){
	  phi.elem(phiIdx) += g(i)-0.5*delta;
	  J.sparse().add(Jg.sparse().getSparseRow(i), phiIdx, 0);
	}else{
	  phi.elem(phiIdx) += 0.5*g(i)*g(i)/delta ;
	  J.sparse().add(Jg.sparse().getSparseRow(i), phiIdx, 0, g(i)/delta);
	}
#elif 1
	double delta=.1, x = g.elem(i);
	phi.elem(phiIdx) += x*x/delta ;
	J.sparse().add(Jg.sparse().getSparseRow(i), phiIdx, 0, 2.*x/delta);
#else
	double delta=.1;
	double x = g(i);
	phi.elem(phiIdx) += x * ::exp(-delta/x);
	J.sparse().add(Jg.sparse().getSparseRow(i), phiIdx, 0, ::exp(-delta/x)* (1.+delta/x));
#endif
      }

    // phi.elem(phiIdx) -= .1;
  }

  void evaluate(arr& phi, arr& J, const arr &_x){
    x.referTo(_x);
    x.reshape(n,3);

    uint dimphi = featureTypes.N;
    phi.resize(dimphi).setZero();
    // J.resize(dimphi, dimension).setZero();
    J.sparse().resize(dimphi, dimension, 0);

    uint m=0;

    //constraints: collisions
    arr collPhi, collJ;
    {
      collPhi.resize(n*(n-1)/2);
      collJ.sparse().resize(collPhi.N, dimension, 0);
      for(uint i=0;i<n;i++) for(uint j=i+1;j<n;j++){
          arr diff = x[i]-x[j];
          double dist = length(diff);
          if(dist<2.*rad+.2){
            collPhi(m) = (-dist + 2.*rad); //ineq
            if(!!J){
              for(uint k=0;k<3;k++){
                collJ.elem(m,3*i+k) = -1./dist*diff.elem(k);
                collJ.elem(m,3*j+k) = +1./dist*diff.elem(k);
              }
            }
          }else{
            collPhi(m) = (-.2);
          }
          m++;
        }
    }

    if(!ineqAccum){
      CHECK_EQ(collPhi.N, m, "");
      phi.setVectorBlock(collPhi, 0);
      J.sparse().add(collJ, 0, 0);
    }else{
      collJ.sparse().setupRowsCols();
      ineqAccumulation(0, phi, J, collPhi, collJ);
      m = 1;
    }

    //costs: z coordinates
    for(uint i=0;i<n;i++){
      phi(m) = x(i,2); //f
      if(!!J){
        J.elem(m,3*i+2) = 1.;
      }
      m++;
    }

    // report(cout, 10, "inner");
    CHECK_EQ(m, dimphi, "");
  }

  void report(ostream &os, int verbose, const char *msg=0){
    // NLP::report(os, verbose, msg);
    x.reshape(n, 3);
    os <<"SpherePacking problem" <<endl;
    if(!disp.frames.N){
      for(uint i=0;i<n;i++){
        rai::Frame *f = disp.addFrame(STRING("sphere"<<i));
        f->setShape(rai::ST_sphere, {rad});
      }
      disp.addFrame("box")->setShape(rai::ST_box, {2.,2.,16.}).setColor({1.,1.,0.,.2}).setPosition({0.,0.,8.});
    }
    for(uint i=0;i<n;i++){
      disp.frames(i)->setPosition(x[i]);
    }
    disp.view(verbose>5);
  }
};

void testSpherePacking(){
  auto P =   std::make_shared<SpherePacking>(50, .21, false);
  std::cout <<P->reportSignature() <<std::endl;

  //-- sample a feasible (=no collision) solution
  NLP_Sampler sam(P);
  sam.opt.set_downhillMaxSteps(100) .set_margin(1e-2) .set_tolerance(1e-4);
  auto retSam = sam.sample();
  cout <<*retSam <<endl;
  // P->report(std::cout, 10);

#if 0
  IpoptInterface ipo(P);
  ipo.solve(retSam->x);
  P->report(std::cout, 10);
#endif

  //-- optimize
  NLP_Solver S;
  S.setProblem(P);
  S.setInitialization(retSam->x);
  // S.getProblem()->checkJacobian(S.x, 1e-6);
  S.setSolver(NLPS_logBarrier);
  // S.setSolver(NLPS_augmentedLag);
  S.opt.set_muMax(1e6) .set_stopEvals(5000) .set_muLBInit(1e-3) .set_muInit(1e3);
  auto ret = S.solve(-1, 2);
  // S.getProblem()->checkJacobian(ret->x, 1e-5);
  std::cout <<*ret <<std::endl;
  P->report(std::cout, 10);

}
//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  // rnd.clockSeed();
  rnd.seed(0);

  // testDisplay();
  // testSolver();

  testSpherePacking();
  return 0;
}
