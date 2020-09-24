/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "pathProblem.h"
#include "../Kin/taskMaps.h"
#include "../Kin/kin_swift.h"
#include "../Optim/constrained.h"
#include "../Kin/F_collisions.h"

//===========================================================================

PathProblem::PathProblem(const rai::Configuration& world_initial,
                         const rai::Configuration& world_final,
                         const Graph& symbolicState,
                         uint microSteps,
                         int verbose)
  : world(world_initial), symbolicState(symbolicState), microSteps(microSteps), verbose(verbose), MP(world) {
  //  ConstrainedProblem::operator=( conv_KOrderMarkovFunction2ConstrainedProblem(MP.komo_problem) );

  double posPrec = rai::getParameter<double>("LGP/precision", 1e3);
//  double colPrec = rai::getParameter<double>("LGP/collisionPrecision", -1e0);
  double margin = rai::getParameter<double>("LGP/collisionMargin", .05);

  //get the actions!
  Node* actionSequence=symbolicState["actionSequence"];
  Graph& actions = actionSequence->graph();
  uint endeff_index = world.getShapeByName("graspRef")->index;
  uint hand_index = world.getShapeByName("eff")->index;

  //-- set up the KOMO
  MP.T=2*actions.N*microSteps;
  world.swift().initActivations(world);
//  MP.world.watch(false);

  //-- decide on pickAndPlace times
  uintA tPick(actions.N), tPlace(actions.N), idObject(actions.N);
  for(uint i=0; i<actions.N; i++) {
    tPick(i) = (2*i+1)*microSteps;
    tPlace(i) = (2*i+2)*microSteps;
    idObject(i) = world.getShapeByName(actions(i)->parents(1)->keys.last())->index;
  }

  //-- transitions
  {
    Objective* t;
    t = MP.addTask("transitions", new TM_Transition(world), OT_sos);
    if(microSteps>3) t->feat->order=2;
    else t->feat->order=1;
    t->setCostSpecs(0, MP.T, {0.}, 1e-1);
  }

  //-- pose damping
  {
    Objective* t;
    t = MP.addTask("pose", new F_qItself(), OT_sos);
    t->feat->order=0;
    t->setCostSpecs(0, MP.T, {0.}, 1e-5);
  }

  //-- tasks
  {
    Objective* t;
    TM_Default* m;
    //pick & place position
    t = MP.addTask("pap_pos", m=new TM_Default(TMT_posDiff), OT_sos);
    m->referenceIds.resize(MP.T+1, 2) = -1;
    t->prec.resize(MP.T+1).setZero();
    t->target.resize(MP.T+1, 3).setZero();
    for(uint i=0; i<actions.N; i++) {
      //pick
      m->referenceIds(tPick(i), 0) = endeff_index;
      m->referenceIds(tPick(i), 1) = idObject(i);
      t->prec(tPick(i))=posPrec;
      //      t->target[tPick(i)]=conv_vec2arr( world_initial.shapes(idObject(i))->X.pos );

      //place
      m->referenceIds(tPlace(i), 0) = idObject(i);
      t->prec(tPlace(i))=posPrec;
      t->target[tPlace(i)]=conv_vec2arr(world_final.shapes(idObject(i))->X.pos);
    }

    //pick & place quaternion
    t = MP.addTask("psp_quat", m=new TM_Default(TMT_quatDiff), OT_sos);
    m->referenceIds.resize(MP.T+1, 2) = -1;
    t->prec.resize(MP.T+1).setZero();
    t->target.resize(MP.T+1, 4).setZero();
    for(uint i=0; i<actions.N; i++) {
      //pick
      m->referenceIds(tPick(i), 0) = endeff_index;
      m->referenceIds(tPick(i), 1) = idObject(i);
      t->prec(tPick(i))=posPrec;
      //      t->target[tPlace(i)]=conv_quat2arr( world_initial.shapes(idObject(i))->X.rot );

      //place
      m->referenceIds(tPlace(i), 0) = idObject(i);
      t->prec(tPlace(i))=posPrec;
      t->target[tPlace(i)]=conv_quat2arr(world_final.shapes(idObject(i))->X.rot);
    }

    // zero position velocity
    if(microSteps>3) {
      t = MP.addTask("psp_zeroPosVel", m=new TM_Default(TMT_pos, endeff_index), OT_sos);
      t->feat->order=1;
      t->prec.resize(MP.T+1).setZero();
      for(uint i=0; i<actions.N; i++) {
        t->prec(tPick(i))=posPrec;
        t->prec(tPlace(i))=posPrec;
      }

      // zero quaternion velocity
      t = MP.addTask("pap_zeroQuatVel", new TM_Default(TMT_quat, endeff_index), OT_sos);
      t->feat->order=1;
      t->prec.resize(MP.T+1).setZero();
      for(uint i=0; i<actions.N; i++) {
        t->prec(tPick(i))=posPrec;
        t->prec(tPlace(i))=posPrec;
      }
    }

    // zero grasp joint motion during holding
//    rai::Joint *j_grasp = world.getJointByName("graspJoint");
//    arr M(j_grasp->qDim(),world.getJointStateDimension());
//    M.setZero();
//    for(uint i=0;i<j_grasp->qDim();i++) M(i,j_grasp->qIndex+i)=1.;
//    cout <<M <<endl;
    t = MP.addTask("graspJoint", new F_qItself(QIP_byJointNames, {"graspJoint"}, world), OT_sos);
    t->feat->order=1;
    t->prec.resize(MP.T+1).setZero();
    for(uint i=0; i<actions.N; i++) {
      for(uint time=tPick(i)+1; time<tPlace(i); time++) t->prec(time)=posPrec;
    }

    // up/down velocities after/before pick/place
    if(microSteps>3) {
      t = MP.addTask("pap_upDownPosVel", new TM_Default(TMT_pos, endeff_index), OT_sos);
      t->feat->order=1;
      t->prec.resize(MP.T+1).setZero();
      t->target.resize(MP.T+1, 3).setZero();
      for(uint i=0; i<actions.N; i++) {
        t->prec(tPick(i)+2)=posPrec;
        t->target[tPick(i)+2] = {0., 0., +.1};

        t->prec(tPlace(i)-2)=posPrec;
        t->target[tPlace(i)-2] = {0., 0., -.1};
      }
    }
  }

  //-- collisions
  {
    Objective* t;
    TM_ProxyConstraint* m;

    //of the object itself
    if(microSteps>3) {
      t = MP.addTask("object_collisions", m=new TM_ProxyConstraint(TMT_allVsListedP, uintA(), margin, true), OT_ineq);
      m->proxyCosts.shapes.resize(MP.T+1, 1) = -1;
      t->prec.resize(MP.T+1).setZero();
      for(uint i=0; i<actions.N; i++) {
        for(uint time=tPick(i)+3; time<tPlace(i)-3; time++) {
          m->proxyCosts.shapes(time, 0)=idObject(i);
          t->prec(time)=1.;
        }
      }
    }

    //of the hand
    t = MP.addTask("hand_collisions", m=new TM_ProxyConstraint(TMT_allVsListedP, uintA(), margin, true), OT_ineq);
    m->proxyCosts.shapes.resize(MP.T+1, 1) = -1;
    t->prec.resize(MP.T+1).setZero();
    for(uint time=0; time<=MP.T; time++) {
      m->proxyCosts.shapes(time, 0)=hand_index;
      t->prec(time)=1.;
    }
  }

  //-- graph switches
  for(uint i=0; i<actions.N; i++) {
    //pick at time 2*i+1
    rai::KinematicSwitch* op_pick = new rai::KinematicSwitch();
    op_pick->symbol = rai::SW_effJoint;
    op_pick->jointType = rai::JT_rigid;
    op_pick->timeOfApplication = tPick(i)+1;
    op_pick->fromId = world.shapes(endeff_index)->index;
    op_pick->toId = world.shapes(idObject(i))->index;
    MP.switches.append(op_pick);

    //place at time 2*i+2
    rai::KinematicSwitch* op_place = new rai::KinematicSwitch();
    op_place->symbol = rai::deleteJoint;
    op_place->timeOfApplication = tPlace(i)+1;
    op_place->fromId = world.shapes(endeff_index)->index;
    op_place->toId = world.shapes(idObject(i))->index;
    MP.switches.append(op_place);
  }

  /*
    if(colPrec<0){ //interpreted as hard constraint (default)
      t = MP.addTask("collisionConstraints", new CollisionConstraint(margin));
      t->setCostSpecs(0, MP.T, {0.}, 1.);
    }else{ //cost term
      t = MP.addTask("collision", new TM_Proxy(TMT_allP, {0}, margin));
      t->setCostSpecs(0, MP.T, {0.}, colPrec);
    }
  */

}

//===========================================================================

double PathProblem::optimize(arr& x) {
  x = MP.getInitialization();
//  rndGauss(x,.01,true); //don't initialize at a singular config

  Conv_KOMO_ConstrainedProblem CP(MP.komo_problem);
  OptConstrained opt(x, NoArr, CP, OPT(verbose=2, damping = 1e-1, stopTolerance=1e-2, maxStep=.5));
  opt.run();
  cout <<MP.getReport();
//  for(;;)
  displayTrajectory(x, 1, MP.world, MP.switches, "planned configs", .02);
  return opt.L.get_costs();
}

//===========================================================================

