#include <LGP/LGP_Tool.h>

const char *USAGE =
    "\nUsage: lgpTool <lgp-filename>"
    "\n  -viewConfig           inspect the world configuration -- analogous to kinEdit of the .g-file"
    "\n  -buildTree 3          build the logic action tree down to depth 5 and display it (outputs z.pdf)"
    "\n  -play                 (interactively go through logic action tree - might be instable)"
    "\n  -folPlan 10           search (using Astar) and output the 5 first logic action plans"
    "\n  -compute \"[2 3 0 0]\"  compute along a branch down the Completion Tree [#plan #waypointSeed #rrt #rrt ... #pathSeed]"
    "\n  -solve                solve the lgp using Effort Level Search on the Completion Tree"
    "\n  -decisions            solve the Completion Tree for a specific action sequence fileBase.decisions"
    "\n  -skeleton             solve the Completion Tree for a specific skeleton defined in fileBase.skt (doesn't have to correspond to an action plan)"
    "\n  -LGP/verbose <0-4>    set verbosity level"
    "\n";

//===========================================================================

void lgpTool(){
  rai::String problem = rai::getParameter<rai::String>("problem", STRING("none"));

  rai::LGP_SkeletonTool lgp(problem);

//  rai::LGP_Tool lgp(L, C,
//                    lgpConfig.get<bool>("genericCollisions"),
//                    lgpConfig.get<StringA>("coll"),
//                    lgpConfig.get<StringA>("lifts"));

  LOG(0) <<"LGP info:";
  lgp.report(cout);

  if(rai::checkParameter<bool>("viewConfig")){
    lgp.viewConfig();
    return;
  }

  if(rai::checkParameter<int>("buildTree")){
    lgp.buildTree(rai::getParameter<int>("buildTree"));
    return;
  }

  if(rai::checkParameter<int>("folPlan")){
    int plans = rai::getParameter<int>("folPlan");
    LOG(0) <<"folPlan " <<plans;
    for(int i=0;i<plans;i++){
      lgp.step_folPlan();
    }
    return;
  }

  if(rai::checkParameter<arr>("compute")){
    intA branches = rai::convert<int>(rai::getParameter<arr>("compute"));
    LOG(0) <<"compute " <<branches <<endl;
    lgp.compute(branches);
    return;
  }

  if(rai::checkParameter<bool>("solveSkeleton")){
    rai::Skeleton S;
    S.read(FILE(lgp.fileBase+".skt"));
    lgp.solve_Skeleton(S);
  }

  if(rai::checkParameter<bool>("solveLGP")){
    lgp.solve_LGP();
    return;
  }

  if(rai::checkParameter<bool>("solveDecisions")){
    rai::String decisions;
    decisions.read(FILE(lgp.fileBase+".decisions"), "", "");
    lgp.solve_Decisions(decisions);
    return;
  }

  if(rai::checkParameter<bool>("play")){
    lgp.player();
    return;
  }
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc, argv, false);

  cout <<USAGE <<endl;

  rai::String problem = rai::params()->get<rai::String>("arg0", "none");
  if(problem!="none") rai::params()->add("problem", problem);

  if(!rai::checkParameter<rai::String>("problem")){
    cout <<"=== lgpFile or -problem parameter missing -- please specify ===" <<endl;
    return 0;
  }

  int s = rai::getParameter<double>("seed", -1.);
  if(s==-1) rnd.clockSeed(); else rnd.seed(s);

  lgpTool();

  return 0;
}

