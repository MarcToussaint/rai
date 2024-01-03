#include <Kin/kin.h>
#include <KOMO/komo-ext.h>
#include <Kin/taskMaps.h>
#include <Core/graph.h>
#include <Gui/opengl.h>
#include <Kin/proxy.h>
#include <Kin/frame.h>
#include <Kin/kin_swift.h>
#include <Kin/forceExchange.h>
#include <Kin/TM_ContactConstraints.h>
#include <Kin/TM_FlagConstraints.h>
#include <Kin/kin_physx.h>
#include <Kin/switch.h>
#include <Kin/kinViewer.h>

void plan(){
  rai::Configuration K("model.g");

  K["ball2"]->joint->setType(rai::JT_rigid);
  K.reset_q();

  KOMO_ext komo;
  komo.setConfig(K, false);
  komo.setPathOpt(3., 20, 1.);
  komo.setSquaredQAccelerations();
  
  //permanent tasks: no collision, gravity
  komo.addObjective({}, make_shared<F_PairCollision>(K, "ball1", "ball2", F_PairCollision::_negScalar, false), OT_ineq, {1e2});

  //-- action 1
//  komo.setImpact(1., "ball1", "ball2");
//  komo.setPlace(1., nullptr, "ball1", "table1");
  komo.addSwitch_dynamicOn(1., -1., "world", "ball2");
//  komo.setKinematicSwitch(1., true, new rai::KinematicSwitch(rai::SW_actJoint, rai::JT_transXY, "world", "ball2", K));
//  komo.setFlag(1., new rai::Flag(FL_zeroAcc, K["ball2"]->ID, 0, true));

  // ball2 over the edge
//  komo.setOverTheEdge(2., "ball2", "table1");

  // gravity for ball2
  komo.addSwitch_dynamic(2., -1., "world", "ball2");
//  komo.setKinematicSwitch(2., true, new rai::KinematicSwitch(rai::SW_actJoint, rai::JT_trans3, "world", "ball2", K));
//  komo.setFlag(2., new rai::Flag(FL_clear, K["ball2"]->ID, 0, true));
//  komo.setFlag(2., new rai::Flag(FL_gravityAcc, K["ball2"]->ID, 0, true));

  // final target for ball2
  komo.addObjective({3., 3.}, make_shared<TM_Default>(TMT_posDiff, K, "ball2"), OT_sos, {+2.,-0.,.3}, 1e1);

  cout <<komo.report(true, false) <<endl;
  komo.verbose=2;
  komo.optimize();
  komo.report(false, false, true);
  komo.checkGradients();

  for(uint i=0;i<2;i++) komo.displayTrajectory(-.01, true, "vid/z.");
  //renderConfigurations(komo.configurations, filePrefix, -komo.k_order, 600, 600, &komo.gl->camera);
}

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  plan();

  return 0;
}
