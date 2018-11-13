#include "komo-ext.h"

#include <Kin/TM_ContactConstraints.h>
//#include <KOMOcsail/komo-CSAIL.h>
#include <Kin/TM_default.h>
#include <Kin/TM_linTrans.h>
#include <Kin/TM_qItself.h>

double shapeSize(const rai::KinematicWorld& K, const char* name, uint i=2);

void addBoxGrasp(KOMO& komo, const char* object, const char* endeff, int axis){
  //  komo.addObjective(0., 0., OT_eq, FS_accumulatedCollisions, {}, 1e0);
  komo.addObjective(0., 0., new TM_LinTrans(new TM_Default(TMT_posDiff, komo.world, "endeffWorkspace", NoVector, object), {2,3,{1.,0.,0., 0.,1.,0.}}, {}), OT_sos, {}, 1e2);

  //height to grasp
  double h = .5*shapeSize(komo.world, object);
#if 0
  setTask(0.,0.,
          new TM_LinTrans(new TM_Default(TMT_pos, K, endeff, {0.,0.,.05}, object), {2,3,{0.,1.,0., 0.,0.,1.}}, {}),
          OT_eq, {0.,h}, 1e2);

  //insideBox
  //    setTask(1.,1., new TM_InsideBox(K, endeff, {0.,0.,.06}, object, .02), OT_ineq, NoArr, 1e2);
  core_setTouch(0., 0., endeff, object);
#else
  //perfect central
  komo.addObjective({}, OT_eq, FS_positionDiff, {endeff, object}, {1e2}, {0,0,h-.05});
#endif

  //anti-podal
  switch(axis){
    case 1:
      komo.addObjective({}, OT_eq, FS_scalarProductXY, {endeff, object}, {1e1});
      komo.addObjective({}, OT_eq, FS_scalarProductXZ, {endeff, object}, {1e1});
      break;
    case 2:
      komo.addObjective({}, OT_eq, FS_scalarProductXX, {endeff, object}, {1e1});
      komo.addObjective({}, OT_eq, FS_scalarProductXZ, {endeff, object}, {1e1});
      break;
    case 3:
      komo.addObjective({}, OT_eq, FS_scalarProductXX, {endeff, object}, {1e1});
      komo.addObjective({}, OT_eq, FS_scalarProductXY, {endeff, object}, {1e1});
      break;
    default: HALT("axis " <<axis <<" needs to be in {1,2,3}");
  }

  //vertical
  komo.addObjective({}, OT_sos, FS_vectorZ, {endeff}, {3e0}, {0.,0.,1.} );

}

void addMotionTo(KOMO& komo, const arr& target_q, const StringA& target_joints, const char* endeff, double up, double down){

  if(endeff){
    arr profile(komo.T, 3);
    profile.setZero();

    if(up>0.){
      uint t0=up*komo.T+1;
      for(uint t=0;t<t0;t++) profile[t] = ARR(0.,0., .05*((double(t)/t0)));
      komo.addObjective(0., up, new TM_Default(TMT_posDiff, komo.world, endeff), OT_sos, profile, 1e2, 1);
    }

    if(down>0.){
      uint t0=down*komo.T-1;
      for(uint t=t0;t<komo.T;t++) profile[t] = ARR(0.,0., -.05*(1.-double(t-t0)/(komo.T-1-t0)));
      komo.addObjective(down, 1., new TM_Default(TMT_posDiff, komo.world, endeff), OT_sos, profile, 1e2, 1);
    }
  }

  if(!target_joints.N){
    komo.addObjective(1.,1., new TM_qItself(), OT_eq, target_q, 1e1);
  }else{
    komo.addObjective(1.,1., new TM_qItself(QIP_byJointNames, target_joints, komo.world), OT_eq, target_q, 1e1);
  }

  komo.setSlow(0.,0., 1e2, true);
  komo.setSlow(1.,1., 1e2, true);
}

void chooseBoxGrasp(rai::KinematicWorld& K, const char* endeff, const char* object){
  KOMO komo;
  komo.setModel(K, true, true);
  komo.setIKOpt();

  //  komo.addObjective(0., 0., OT_eq, FS_accumulatedCollisions, {}, 1e0);
  //open gripper
  //  komo.addObjective(0.,0., OT_sos, FS_qItself, {"r_gripper_joint"}, 1e1, {.08} );
  //  komo.addObjective(0.,0., OT_sos, FS_qItself, {"r_gripper_l_finger_joint"}, 1e1, {.8} );

  addBoxGrasp(komo, object, endeff, 1);
  komo.optimize();
  //  komo.getConfiguration(0);
  auto q1 = komo.x;
  double score1 = 10.*komo.getConstraintViolations() + komo.getCosts();
  //    D.update(true);

  komo.clearObjectives();
  komo.setIKOpt();
  addBoxGrasp(komo, object, endeff, 2);
  komo.optimize();
  //  komo.getConfiguration(0);
  auto q2 = komo.x;//K.getJointState();
  double score2 = 10.*komo.getConstraintViolations() + komo.getCosts();
  //    D.update(true);

  if(score1<score2){
    cout <<"using axis 1" <<endl;
    K.setJointState(q1);
  }else{
    cout <<"using axis 2" <<endl;
    K.setJointState(q2);
  }
}
