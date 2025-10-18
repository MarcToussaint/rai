#include <KOMO/testProblems_KOMO.h>
#include <KOMO/komo_NLP.h>
#include <Optim/NLP_Solver.h>
#include <KOMO/komo.h>
#include <Kin/viewer.h>

//===========================================================================

void TEST(KOMO_IK) {
  OptBench_InvKin_Endeff nlp("../../KOMO/switches/model2.g", false);

  rai::NLP_Solver S;

  rai::Enum<rai::OptMethod> sid (rai::getParameter<rai::String>("solver"));
  S.setSolver(sid);
  S.setProblem(nlp.get());

  auto ret = S.solve();
  nlp.get()->report(cout, 10);
  S.gnuplot_costs();

  rai::wait();
}

//===========================================================================

void TEST(Skeleton_Handover) {
  OptBench_Skeleton_Pick nlp(rai::_path);
  //  OptBench_Skeleton_Handover nlp(rai::_path);
  //  OptBench_Skeleton_StackAndBalance nlp(rai::_sequence);

  rai::NLP_Solver S;

  rai::Enum<rai::OptMethod> sid (rai::getParameter<rai::String>("solver"));
  S.setSolver(sid);
  S.setProblem(nlp.get());

  //  nlp.get()->report(cout, 0);
  auto ret = S.solve();
  nlp.get()->report(cout, 10);
  S.gnuplot_costs();

  rai::wait();
}

//===========================================================================

void test(str problemName={}, int initUniform=-1){
  if(initUniform<0) initUniform = rai::getParameter<int>("initUniform");
  if(!problemName.N) problemName = rai::getParameter<str>("problem");
  uint runs = rai::getParameter<uint>("runs", 20);

  // uint s=0;
  double _feasible=0., _cost=0., _constraints=0., _time=0., _count=0.;
  for(uint k=0;k<1;k++){
    auto nlp = make_NLP_Problem(problemName);

    for(uint i=0;i<runs;i++){
      _count++;
      rai::NLP_Solver S;
      S.setProblem(nlp);

      LOG(0) <<"problem: " <<problemName <<" method: " <<rai::Enum<rai::OptMethod>(S.opt.method) <<" initUniform: " <<initUniform;

      KOMO *komo = (KOMO*)(nlp->obj.get());

      if(komo){
        if(initUniform){
          S.setInitialization(nlp->getUniformSample());
          komo->pathConfig.setJointState(S.x);
        }else{
          komo->initRandom();
          S.setInitialization(nlp->getInitializationSample());
        }
      }

      if(S.opt.method==rai::M_LogBarrier){
        LOG(0) <<"PHASE ONE";
        S.setSolver(rai::M_slackGN);
        auto ret = S.solve();
        if(!ret->feasible){
          LOG(0) <<"FAILED!";
          continue;
        }
        LOG(0) <<"PHASE ONE: " <<*ret;
        S.setSolver(rai::M_LogBarrier);
      }

      // P.komo->view(true);
      auto ret = S.solve();
      cout <<*ret <<endl;
      if(ret->feasible){
        _feasible++;
        _cost += ret->sos + ret->f;
        _constraints += ret->eq + ret->ineq;
      }
      _time += ret->time;
      S.gnuplot_costs();
      // S.getProblem()->checkJacobian(ret->x, 1e-4, {});
      //cout <<P.komo->report() <<endl;
      // if(komo){
      //   komo->view(ret->feasible);
      //   if(ret->feasible){
      //     komo->get_viewer()->visualsOnly();
      //     komo->view_play(false, 0, .2, STRING("z."<<s++<<".vid/"));
      //   }
      // }else{
        nlp->report(cout, 3);
      // }
    }
  }

  cout <<"\n================================= (avgs over " <<_count <<" runs)\n";
  rai::OptOptions opt;
  cout <<"method.:\t" <<rai::Enum<rai::OptMethod>(opt.method) <<endl;
  cout <<"feas.:\t" <<_feasible/_count <<endl;
  cout <<"cost:\t" <<_cost/_feasible <<endl;
  cout <<"constraints:\t" <<_constraints/_feasible <<endl;
  cout <<"time:\t" <<_time/_count <<endl;
  cout <<"=================================\n";
}

//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  //  rnd.seed_random();
  rnd.seed(0);

  // testKOMO_IK();
  // testSkeleton_Handover();

  test();
  // test("IK", true);
  // test("IKobstacle", true);
  // test("IKtorus", true);
  // test("PushToReach");
  // test("StableSphere");

  return 0;
}
