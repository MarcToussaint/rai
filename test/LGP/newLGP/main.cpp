#include <LGP/LGP_Tool.h>

#include <KOMO/manipTools.h>
#include <LGP/LGP_SkeletonTool.h>

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  int verbose = rai::getParameter<int>("verbose", 1);

  rai::String problem = rai::getParameter<rai::String>("problem", STRING("none"));

  rai::Configuration C;
  C.addFile(problem+".g");
  auto trans = rai::default_Logic2KOMO_Translator();
  auto tamp = rai::default_TAMP_Provider(C, problem+".lgp");

  rai::LGP_Tool lgp(tamp->getConfig(), *tamp, *trans);

  bool fixWaypoints = rai::getParameter<bool>("fixWaypoints", true);

  for(uint k=0;k<10;k++){
    lgp.solve(verbose-1);
    cout <<"** solution " <<k <<" (" <<lgp.step_count <<"steps): " <<lgp.getSolvedPlan() <<' ' <<lgp.getSolvedKOMO().get() <<endl;
    lgp.view_solved(false);

    //solve the path piece-by-piece
    StringAA plan = lgp.getSolvedPlan();
    for(uint k=0;k<plan.N;k++){
      cout <<"--- plan " <<k <<' ' <<plan(k) <<endl;
      PTR<KOMO> piece = lgp.get_piecewiseMotionProblem(k, fixWaypoints);
      auto ret = NLP_Solver(piece->nlp(), 0).solve();
      piece->set_viewer(C.get_viewer()); // to prevent too many windows popping up
      if(verbose>1)
        piece->view(true, STRING(*ret <<"\n PIECE " <<k <<' ' <<plan(k)));
      else
        piece->view_play(false, STRING(*ret <<"\n PIECE " <<k <<' ' <<plan(k)));
    }

    //alternative: solve full path jointly
    PTR<KOMO> path = lgp.get_fullMotionProblem(fixWaypoints);
    auto ret = NLP_Solver(path->nlp(), 0). solve();
    path->set_viewer(C.get_viewer()); // to prevent too many windows popping up
    path->view_play(verbose>1, STRING(*ret <<"\n FULL PATH " <<plan));
  }

  return 0;
}

