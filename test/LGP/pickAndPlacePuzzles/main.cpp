#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>
#include <Kin/proxy.h>
#include <Kin/viewer.h>

#include <LGP/LGP_tree.h>
#include <KOMO/komo.h>

void loadProblem(rai::Configuration& C, const std::string &filename){
  C.addFile(filename.c_str());
  //C.selectJointsByAtt({"ego"});
  //C.pruneInactiveJoints();
  //C.optimizeTree();
  //check for collisions
 // C.stepSwift();
  //arr y, J;
  //C.kinematicsPenetration(y, J);
 // LOG(0) <<"collision costs of config: " << y.scalar() <<endl;
//    K.reportProxies(cout, .1, true);
//    K.watch();
 // if(y.scalar()==0.) { 
 //   LOG(0) << "Problem without collisions"; 
 // }
 // else {
 //   LOG(1) << "Collisions found in the config";
 // }
  
 // C.proxies.clear();
}


void generateProblem(rai::Configuration& C){
  uint numObj = 4;
  for(;;){
    C.clear();    
    C.addFile("0k-2d-room-ego-obj-limits.g");

    C.selectJointsByAtt({"ego"});
    C["ego"]->ats->newNode<rai::Graph>({"logical"}, {}, {{"gripper", true}});
    
    for(uint i=0;i<numObj;i++){
      rai::Frame *f = C.addFrame(STRING("obj"<<i), "floor", "type:ssBox size:[.1 .1 .2 .02] color:[1. 0. 0.],  logical={ object }, joint:rigid" );
      f->setRelativePosition({rnd.uni(-.3, .3), rnd.uni(-1.,1.), .15});
      f->setRelativeQuaternion(rai::Quaternion(0).addZ(rnd.uni(-RAI_PI,RAI_PI)).getArr4d());
    }
    C.stepSwift(); 
    arr y, J;
    C.kinematicsPenetration(y, J); 
//    cout <<"collision costs of config: " <<y.scalar() <<endl;
//    K.reportProxies(cout, .1, true);
//    K.watch();
    if(y.scalar()==0.) break; 
  }

  C.proxies.clear();

  rai::Frame *f = C.addFrame("tray", "floor", "type:ssBox size:[.15 .15 .04 .02] color:[0. 1. 0.], logical={ table }" );
  f->setRelativePosition({0.,0.,.07});

  // C.addFrame("", "tray", "type:ssBox size:[.27 .27 .04 .02] color:[0. 1. 0.]" );

}

  
void solve(const std::string &configName) {
  rai::Configuration C;
  loadProblem(C,  configName);

  rai::LGP_Tree lgp(C, "fol-pnp-switch.g");
  //rai::LGP_Tree lgp(C, "fol-pnp.g");

  //lgp.fol.addTerminalRule("(on goal obj)"); 
  //lgp.fol.addTerminalRule("(on goal obj) (on goal ego)"); 
  lgp.fol.addTerminalRule("(on goal obj)"); 
  ///lgp.fol.addTerminalRule("(stable ego obj)"); 

  lgp.displayBound = rai::BD_seqPath;
  //lgp.verbose=2;

  lgp.fol.writePDDLfiles("z");

  lgp.run();
  if(lgp.solutions.set().data->d0 == 0) {
    LOG(0) << "No solutions found";
  }
  for(auto* s:lgp.solutions.set()()){
    cout <<"SOLUTION:\n";
    s->write(cout);
    cout <<endl;
  }

  rai::wait();
  // if(lgp.verbose>1){
  //   rai::wait();
  //   lgp.renderToVideo();
  // }
}

void playIt(const std::string &configName){
  rai::Configuration C;
   //generateProblem(C);
  loadProblem(C, configName);
  rai::LGP_Tree lgp(C, "fol-pnp-switch.g");

  lgp.player();
}


void testBounds(const std::string &configName){
  rai::Configuration C;
  //generateProblem(C);
  loadProblem(C, configName);
  FrameL collisions = C.getCollisionAllPairs();
  for(auto coll : collisions) {
    std::cout << coll->name << std::endl;
  }
  rai::LGP_Tree lgp(C, "fol-pnp-switch.g");

  lgp.inspectSequence("(pick ego obj) (place ego obj goal) ");
  //lgp.inspectSequence("(pick ego cube2) (place ego cube2 floor_right) ");
  //lgp.inspectSequence("(pick ego obj) (place ego obj floor_left) (pick ego cube2) (place ego cube2 goal) (pick ego obj) (place ego obj goal)  ");

  return;

  ////rai::LGP_Node* node = lgp.walkToNode("(pick pr2R obj0) (pick pr2L obj3) (place pr2R obj0 tray) (place pr2L obj3 tray)");
  ////rai::BoundType bound = rai::BD_path;
  ////node->optBound(bound, true, 2);
  ////rai::ConfigurationViewer V;
  ////node->displayBound(V, bound);
//  V.setPath(node->problem(bound).komo->getPath_X(), "", true);
//  for(uint i=0;i<2;i++) V.playVideo(true, 3.);
}

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);
//  rnd.clockSeed();
  std::string configName = "minimal.g";

  if(argc > 1) {
    configName = argv[1];
  }
  //solve(configName);

  //playIt(configName);

  testBounds(configName);

  return 0;
}
