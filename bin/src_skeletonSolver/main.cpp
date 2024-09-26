#include <KOMO/skeleton.h>
#include <KOMO/komo.h>
#include <Kin/viewer.h>
#include <Kin/dof_path.h>
#include <PathAlgos/ConfigurationProblem.h>
#include <PathAlgos/RRT_PathFinder.h>
#include <KOMO/pathTools.h>
#include <Gui/opengl.h>

//===========================================================================

const char *USAGE =
    "\nUSAGE:  skeletonSolver <problem> -mode [path|waypoints|final] -samples [n] -collisions [true|false]"
    "\n        (set parameters in rai.cfg alternatively, see z.log.global for a log of all used options)"
    "\n";

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

//  rnd.clockSeed();
  rnd.seed(0);

  cout <<USAGE <<endl;

  rai::String problem = rai::getParameter<rai::String>("problem", STRING("none"));
  if(rai::argc>=2 && rai::argv[1][0]!='-') problem=rai::argv[1];

  rai::String sktFile, confFile;
  if(problem=="none"){
    sktFile = rai::getParameter<rai::String>("sktFile", STRING("none"));
    confFile = rai::getParameter<rai::String>("confFile", STRING("none"));
  }else{
    confFile = problem+".g";
    sktFile = problem+".skt";
  }

  uint samples = rai::getParameter<uint>("samples", 10);
  bool collisions = rai::getParameter<bool>("collisions", false);
  bool final = rai::getParameter<bool>("final", true);
  bool ways = rai::getParameter<bool>("ways", true);
  bool rrt = rai::getParameter<bool>("rrt", false);
  bool path = rai::getParameter<bool>("path", false);
  double rrtStopEvals =  rai::getParameter<double>("rrtStopEvals", 10000);
  double rrtTolerance =  rai::getParameter<double>("rrtTolerance", .03);
  double rrtStepsize =  rai::getParameter<double>("rrtStepsize", .05);
  StringA floating = rai::getParameter<StringA>("floating", {});

  LOG(0) <<"used parameters: " <<rai::params();

  rai::Configuration C;
  C.addFile(confFile);

  if(floating.N){
    floating.reshape(-1, 3);
    for(uint i=0;i<floating.d0;i++){
      C[floating(i,1)]->makeManipJoint(rai::Enum<rai::JointType>(floating(i,2)), C[floating(i,0)], true);
    }
  }

  rai::Skeleton S;
  S.read(FILE(sktFile));
  S.collisions = collisions;
  cout <<S <<endl;

  std::shared_ptr<KOMO> komo_way = S.getKomo_waypoints(C);
  std::shared_ptr<KOMO> komo_path = S.getKomo_path(C);
  std::shared_ptr<KOMO> komo_final = S.getKomo_finalSlice(C);

  for(uint i=0;i<samples;i++){
    cout <<"=== SAMPLE " <<i <<" ===" <<endl;

    komo_way = S.getKomo_waypoints(C);
    komo_path = S.getKomo_path(C);
    komo_final = S.getKomo_finalSlice(C);

    //-- final
    if(final){
      komo_final->initRandom();
      komo_final->pathConfig.gl().setTitle("FINAL");
      komo_final->view(true, STRING("random init sample " <<i));
      {
        auto ret = NLP_Solver() .setProblem(komo_way->nlp()) .solve();
        cout <<komo_final->report(false, true, true) <<endl;
        cout <<*ret <<endl;

        rai::wait(.1);
        komo_final->pathConfig.get_viewer()->raiseWindow();
        komo_final->view(true, STRING("solved sample " <<i <<"\n" <<*ret));
        //if(!ret->feasible) continue;
      }
    }

    //-- waypoints
    komo_way->initRandom(0);
    komo_way->pathConfig.gl().setTitle("WAYPOINTS");
    komo_way->view(true, STRING("random init sample " <<i));
    if(!ways) continue;
    {
      komo_way->opt.animateOptimization=2;
      NLP_Solver sol;
      sol.setProblem(komo_way->nlp());
      auto ret = sol.solve();
      cout <<komo_way->report(false, true, true) <<endl;
      cout <<*ret <<endl;
      cout <<komo_way->getPath_qAll();

      rai::wait(.1);
      komo_way->pathConfig.get_viewer()->raiseWindow();
      komo_way->view(true, STRING("solved sample " <<i <<"\n" <<*ret));
      if(!ret->feasible) continue;
      while(komo_way->view_play(true));
    }

    //-- setup pathconfig
    komo_path->initWithWaypoints(komo_way->getPath_qAll());
    komo_path->pathConfig.gl().setTitle("PATH");
    komo_path->view(false, STRING("init path"));

    //-- rrt
    if(rrt){
      arrA paths;
      for(uint t=0;t<komo_way->T;t++){
        rai::Configuration C;
        arr q0, qT;
        rai::Skeleton::getTwoWaypointProblem(t, C, q0, qT, *komo_way);
        if(q0.N==0 && qT.N==0) continue;
        //cout <<C.getJointNames() <<endl;
        ConfigurationProblem cp(C, true, rrtTolerance);
        if(S.explicitCollisions.N) cp.setExplicitCollisionPairs(S.explicitCollisions);
        cp.computeAllCollisions = S.collisions;

        for(rai::Frame *f:C.frames) f->ensure_X();
        RRT_PathFinder rrt(cp, q0, qT, rrtStepsize);
        if(S.verbose>1) rrt.verbose=S.verbose-2;
        rrt.verbose=2;
        rrt.maxIters=rrtStopEvals;

        arr sol = rrt.planConnect();
        if(sol.N){
          sol = path_resampleLinear(sol, komo_path->stepsPerPhase);
          komo_path->initPhaseWithDofsPath(t, C.getDofIDs(), sol, false);
          komo_path->view(false, STRING("rrt phase " <<t));
        }
      }
    }
    komo_path->view(true, STRING("path after rrts"));

    //-- komo
    if(path){
      NLP_Solver sol;
      sol.setProblem(komo_path->nlp());
      auto ret = sol.solve();
      cout <<komo_path->report(false, true, true) <<endl;
      cout <<*ret <<endl;

      rai::wait(.1);
      komo_path->pathConfig.get_viewer()->raiseWindow();
      komo_path->view(true, STRING("solved sample " <<i <<"\n" <<*ret));
      while(komo_path->view_play(true));
    }

    //-- readout in case of float
    if(floating.N){
      arr X = komo_path->getPath_X();
      for(uint i=0;i<floating.d0;i++){
        rai::Frame *f = C[floating(i,1)];
        rai::String pathFile = STRING("guide_"<<floating(i,1)<<".dat");
        arr path = X.sub(0,-1,f->ID,f->ID,0,-1);
        FILE(pathFile) <<path;
        rai::Frame *g = C.addFrame(f->name+"_guide");
        if(!g->shape) g->shape = new rai::Shape(*g, f->shape); //copy shap
        if(!g->pathDof) g->pathDof = new rai::PathDof(*g);
        g->pathDof->path = path;
        if(!g->ats) g->ats = make_shared<rai::Graph>();
        g->ats->add<rai::String>("path", pathFile);
      }
      FILE("z.floating.g") <<C <<endl;
      while(C.animate()!='q');
    }
  }

  rai::wait();

  return 0;
}

