#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>
#include <Kin/proxy.h>
#include <Kin/viewer.h>

#include <LGP/LGP_tree.h>
#include <KOMO/komo.h>

void generateProblem(rai::Configuration& C){
  uint numObj = 4;
  for(;;){
    C.clear();
    C.addFile("../../../../rai-robotModels/pr2/pr2.g");
    C.selectJointsByAtt({"base","armL","armR"});
    C.pruneInactiveJoints();
    C.optimizeTree();
    C["pr2L"]->ats->add<rai::Graph>({"logical"}, {{"gripper", true}});
    C["pr2R"]->ats->add<rai::Graph>({"logical"}, {{"gripper", true}});
    C["worldTranslationRotation"]->joint->H = 1e-0;
    C.addFile("../../../../rai-robotModels/objects/tables.g");
    for(uint i=0;i<numObj;i++){
      rai::Frame *f = C.addFrame(STRING("obj"<<i), "table1", "type:ssBox size:[.1 .1 .2 .02] color:[1. 0. 0.], contact, logical={ object }, joint:rigid" );
      f->setRelativePosition({rnd.uni(-.3, .3), rnd.uni(-1.,1.), .15});
      f->setRelativeQuaternion(rai::Quaternion(0).addZ(rnd.uni(-RAI_PI,RAI_PI)).getArr4d());
    }
    C.stepFcl();
    arr y, J;
    C.kinematicsPenetration(y, J);
//    cout <<"collision costs of config: " <<y.scalar() <<endl;
//    K.reportProxies(cout, .1, true);
//    K.view();
    if(y.scalar()==0.) break;
  }

  C.proxies.clear();

  rai::Frame *f = C.addFrame("tray", "table2", "type:ssBox size:[.15 .15 .04 .02] color:[0. 1. 0.], logical={ table }" );
  f->setRelativePosition({0.,0.,.07});
//  f->Q.pos = {rnd.uni(-.3, .3), rnd.uni(-1.,1.), .07};
//  f->Q.rot.addZ(rnd.uni(-RAI_PI,RAI_PI));

  // add a bigger geometry around the center placement area of the tray
  C.addFrame("", "tray", "type:ssBox size:[.27 .27 .04 .02] color:[0. 0.95 0.]" );
//  K.addFrame("", "tray", "type:ssBox size:[.04 .3 .1 .02] Q:<t(+.13 0 .03)> color:[0. 1. 0.], contact" );
//  K.addFrame("", "tray", "type:ssBox size:[.04 .3 .1 .02] Q:<t(-.13 0 .03)> color:[0. 1. 0.], contact" );
//  K.addFrame("", "tray", "type:ssBox size:[.04 .3 .1 .02] Q:<d(90 0 0 1) t(+.13 0 .03)> color:[0. 1. 0.], contact" );
//  K.addFrame("", "tray", "type:ssBox size:[.04 .3 .1 .02] Q:<d(90 0 0 1) t(-.13 0 .03)> color:[0. 1. 0.], contact" );

}

void solve(){
  rai::Configuration C;
  generateProblem(C);
  //  K.addFile("model2.g");
  C.selectJointsByAtt({"base","armL","armR"});
  C.optimizeTree();
  //  C.sortFrames(); FILE("z.g") <<C;

  rai::LGP_Tree lgp(C, "fol-pnp-switch.g");
  lgp.fol.addTerminalRule("(on tray obj0) (on tray obj1) (on tray obj2)");
  lgp.displayBound = rai::BD_seqPath;
  //lgp.verbose=2;

  lgp.fol.writePDDLfiles("z");

  lgp.run(-1);

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

void playIt(){
  rai::Configuration C;
  generateProblem(C);
  rai::LGP_Tree lgp(C, "fol-pnp-switch.g");
  lgp.player();
}

void testBounds(){
  rai::Configuration C;
  generateProblem(C);
//  K.addFile("model2.g");

  rai::LGP_Tree lgp(C, "fol-pnp-switch.g");

//  lgp.inspectSequence("(pick pr2R obj0) (place pr2R obj0 tray)");
//  lgp.inspectSequence("(pick pr2R obj0) (pick pr2L obj1) (place pr2R obj0 tray) (place pr2L obj1 tray) (pick pr2L obj2) (place pr2L obj2 tray)");
  lgp.inspectSequence("(pick pr2R obj0) (pick pr2L obj3) (place pr2R obj0 tray) (place pr2L obj3 tray)");
  return;

  rai::LGP_Node* node = lgp.walkToNode("(pick pr2R obj0) (pick pr2L obj3) (place pr2R obj0 tray) (place pr2L obj3 tray)");
  rai::BoundType bound = rai::BD_path;
  node->optBound(bound, true, 2);
  rai::ConfigurationViewer V;
  node->displayBound(V, bound);
}

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);
//  rnd.clockSeed();

  solve();

//  testBounds();

//  if(rai::getInteractivity())
//    playIt();

  return 0;
}
