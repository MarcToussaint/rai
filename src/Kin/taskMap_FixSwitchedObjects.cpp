/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "taskMap_FixSwitchedObjects.h"
#include "taskMap_qItself.h"
#include "taskMap_default.h"

uint TaskMap_FixSwichedObjects::dim_phi(const WorldL& G, int t){
#if 0
  mlr::Array<mlr::Joint*> switchedJoints = getSwitchedJoints(*G.elem(-2), *G.elem(-1));
  return switchedJoints.d0*7;
#else
  mlr::Array<mlr::Body*> switchedBodies = getSwitchedBodies(*G.elem(-2), *G.elem(-1));
  return switchedBodies.d0*7;
#endif
}


#if 0
void TaskMap_FixSwichedObjects::phi(arr& y, arr& J, const WorldL& G, double tau, int t){
  //TODO: so far this only fixes switched objects to zero pose vel
  //better: constrain to zero relative velocity with BOTH, pre-attached and post-attached
  CHECK(order==1,"");

  arr yi, Ji;
  uint M=7;
  mlr::Array<mlr::Joint*> switchedJoints = getSwitchedJoints(*G.elem(-2), *G.elem(-1));
  y.resize(M*switchedJoints.d0);
  if(&J){
    uint xbarDim=0;
    for(auto& W:G) xbarDim+=W->q.N;
    J.resize(M*switchedJoints.d0, xbarDim).setZero();
  }
  for(uint i=0;i<switchedJoints.d0;i++){
    mlr::Joint *j0 = switchedJoints(i,0);    CHECK(&j0->world==G.elem(-2),"");
    mlr::Joint *j1 = switchedJoints(i,1);    CHECK(&j1->world==G.elem(-1),"");
    CHECK(j0->to->index == j1->to->index,"");

//    if(j1->to->name.startsWith("slider")) continue;

#if 1 //absolute velocities
    TaskMap_Default pos(posDiffTMT, j0->to->shapes.first()->index);
    pos.order=1;
    pos.TaskMap::phi(y({M*i,M*i+2})(), (&J?J({M*i,M*i+2})():NoArr), G, tau, t);

    TaskMap_Default quat(quatDiffTMT, j0->to->shapes.first()->index);
    // flipp the quaternion sign if necessary
    quat.flipTargetSignOnNegScalarProduct = true;
    quat.order=1;
    quat.TaskMap::phi(y({M*i+3,M*i+6})(), (&J?J({M*i+3,M*i+6})():NoArr), G, tau, t);
#else //relative velocities
    TaskMap_Default pos(posDiffTMT, j0->to->shapes.first()->index, NoVector, j0->from->shapes.first()->index);
    pos.order=1;
    pos.TaskMap::phi(y({M*i,M*i+2})(), (&J?J({M*i,M*i+2})():NoArr), G, tau, t);

    TaskMap_Default quat(quatDiffTMT, j0->to->shapes.first()->index/*, NoVector, j0->from->shapes.first()->index*/);
    quat.flipTargetSignOnNegScalarProduct = true;
    quat.order=1;
    quat.TaskMap::phi(y({M*i+3,M*i+6})(), (&J?J({M*i+3,M*i+6})():NoArr), G, tau, t);
#endif
  }
}
#endif

void TaskMap_FixSwichedObjects::phi(arr& y, arr& J, const WorldL& G, double tau, int t){
  //TODO: so far this only fixes switched objects to zero pose vel
  //better: constrain to zero relative velocity with BOTH, pre-attached and post-attached
  CHECK(order==1,"");

  arr yi, Ji;
  uint M=7;
  mlr::Array<mlr::Body*> switchedBodies = getSwitchedBodies(*G.elem(-2), *G.elem(-1));
  y.resize(M*switchedBodies.d0).setZero();
  if(&J){
    uint xbarDim=0;
    for(auto& W:G) xbarDim+=W->q.N;
    J.resize(M*switchedBodies.d0, xbarDim).setZero();
  }
  for(uint i=0;i<switchedBodies .d0;i++){
    mlr::Body *b0 = switchedBodies(i,0);    CHECK(&b0->world==G.elem(-2),"");
    mlr::Body *b1 = switchedBodies(i,1);    CHECK(&b1->world==G.elem(-1),"");
    CHECK(b0->index == b1->index, "");
    CHECK(b0->shapes.first()->index == b1->shapes.first()->index, "");

    if(b0->name.startsWith("slider")) continue; //warning: this introduces zeros in y and J -- but should be ok

#if 1 //absolute velocities
    TaskMap_Default pos(posDiffTMT, b0->shapes.first()->index);
    pos.order=1;
    pos.TaskMap::phi(y({M*i,M*i+2})(), (&J?J({M*i,M*i+2})():NoArr), G, tau, t);

    TaskMap_Default quat(quatDiffTMT, b0->shapes.first()->index);
    // flipp the quaternion sign if necessary
    quat.flipTargetSignOnNegScalarProduct = true;  ///WARNING: somehow that doesn't work! The effect on grasping is strange (test when grasping rotated stick)
    quat.order=1;
    quat.TaskMap::phi(y({M*i+3,M*i+6})(), (&J?J({M*i+3,M*i+6})():NoArr), G, tau, t);

#if 0
    if(&J && t==3){
        cout <<"switched bodies:" <<b0->name <<' ' <<b1->name <<endl;
//        analyzeJointStateDimensions();
        //-- clean up the graph
//        G.elem(-1)->analyzeJointStateDimensions();
//        G.elem(-1)->checkConsistency();
//        G.elem(-1)->topSort();
//        G.elem(-1)->jointSort();
//        G.elem(-1)->calc_missingAB_from_BodyAndJointFrames();
//        G.elem(-1)->analyzeJointStateDimensions();
//        G.elem(-1)->calc_q_from_Q();
//        G.elem(-1)->calc_fwdPropagateFrames();

        arr yt,Jt;
        pos.phi(yt, Jt, *G.last(), t);

        G.elem(-1)->checkConsistency();
        G.elem(-2)->checkConsistency();
        FILE("z.last") <<*G.elem(-1);
        cout <<"\n*** A ***\n" <<J*sqrt(1000.) <<endl;
        cout <<"\n*** C ***\n" <<Jt*sqrt(1000.) <<endl;

        if(true){
//          const char* filename="z.last";
          const char* shape="obj1";
          mlr::KinematicWorld K(*G.elem(-1));
          FILE("z.last2") <<K;
//          mlr::KinematicWorld K(filename);
//          K.setJointState(G.elem(-1)->q);
          mlr::Shape *sh=K.getShapeByName(shape);
          TaskMap_Default pos(posDiffTMT, sh->index);
          arr y,J;
          pos.phi(y, J, K);
          cout <<"\n*** B ***\n" <<J*sqrt(1000.) <<endl;


          VectorFunction f = ( [&pos, &K](arr& y, arr& J, const arr& x) -> void
          {
              K.setJointState(x);
                  pos.phi(y,J,K);
          } );

          checkJacobian(f, K.q, 1e-4);

//          exit(0);
//          mlr::wait();
        }

    }
#endif

//    if(sumOfSqr(y)>1e-3) cout <<"body " <<b0->name <<" causes switch costs " <<sumOfSqr(y) <<" at t=" <<t <<" y=" <<y <<endl;
#else //relative velocities
    TaskMap_Default pos(posDiffTMT, j0->to->shapes.first()->index, NoVector, j0->from->shapes.first()->index);
    pos.order=1;
    pos.TaskMap::phi(y({M*i,M*i+2})(), (&J?J({M*i,M*i+2})():NoArr), G, tau, t);

    TaskMap_Default quat(quatDiffTMT, j0->to->shapes.first()->index/*, NoVector, j0->from->shapes.first()->index*/);
    // flipp the quaternion sign if necessary
    quat.flipTargetSignOnNegScalarProduct = true;
    quat.order=1;
    quat.TaskMap::phi(y({M*i+3,M*i+6})(), (&J?J({M*i+3,M*i+6})():NoArr), G, tau, t);
#endif
  }
}



