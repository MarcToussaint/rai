#include "path.h"

#include <KOMO/komo-ext.h>

rai::String validatePath(const rai::KinematicWorld& _K, const arr& q_now, const StringA& joints, const arr& q, const arr& tau){
  rai::KinematicWorld K;
  K.copy(_K, true);

//  arr q0 = K.getJointState();

//  syncModelJointStateWithRealOrSimulation();

//  arr q_now = K.getJointState(joints);

  CHECK_EQ(q_now.N, q.d1, "");

  rai::String txt;
  txt <<"VALIDATE ";

  if(q.d0>1){
    double startVel, endVel, maxVel=0.;
    startVel = length(q[0]-q_now)/(tau(0));
    endVel = length(q[-1]-q[-2])/(tau(-1)-tau(-2));
    for(uint t=1;t<q.d0;t++){
      double v = length(q[t]-q[t-1])/(tau(t)-tau(t-1));
      if(v>maxVel) maxVel = v;
    }
    txt <<"\nv0=" <<startVel <<" vT=" <<endVel <<" vMax=" <<maxVel;
  }
  if(joints.N<=3){
    txt <<"\n" <<joints;
  }
  txt <<"\n";
  return txt;
}


//  PlanDrawer planDrawer(K, q_now, joints, q, tau);
//  K.gl().remove(K);
//  K.gl().add(planDrawer);
//  for(;;){
//    int key = K.gl().watch(txt);

//    if(key==13){ //validated
//      K.gl().remove(planDrawer);
//      K.gl().add(K);
//      K.setJointState(q0);
//      K.gl().update("validated");
//      return;
//    }

//    if(key==27){
//      LOG(0) <<"NO VALIDATION - exiting";
//      K.gl().closeWindow();
//      exit(0);
//    }
//  }
//}

std::pair<arr, arr> computePath(const rai::KinematicWorld& K, arr target_q, StringA target_joints, const char* endeff, double up, double down){
    KOMO komo;
    komo.setModel(K, true);
    komo.setPathOpt(1., 20, 3.);
    komo.setSquaredQAccelerations();

    addMotionTo(komo, target_q, target_joints, endeff, up, down);
    komo.verbose=1;
    komo.optimize();

    arr path = komo.getPath(K.getJointNames());
    path[path.d0-1] = target_q; //overwrite last config
    arr tau = komo.getPath_times();
    cout <<validatePath(K, K.getJointState(), target_joints, path, tau) <<endl;
    bool go = komo.displayPath(false);//;/komo.display();
    if(!go){
        cout <<"ABORT!" <<endl;
        return {arr(), arr()};
    }
    return {path, tau};
}

void mirrorDuplicate(std::pair<arr, arr>& path){
    arr& q = path.first;
    arr& t = path.second;

    if(!q.N) return;

    uint T=q.d0-1;
    double D=2.*t.last();

    q.resizeCopy(2*T+1, q.d1);
    t.resizeCopy(2*T+1);
    for(uint i=1;i<=T;i++){
        q[T+i] = q[T-i];
        t(T+i) = D - t(T-i);
    }
}
