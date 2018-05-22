/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_FixSwitchedObjects.h"
#include "TM_qItself.h"
#include "TM_default.h"
#include "frame.h"
#include "flag.h"

uint TM_FixSwichedObjects::dim_phi(const WorldL& G) {
  uintA switchedBodies = getSwitchedBodies(*G.elem(-2), *G.elem(-1));
//  if(order==2) switchedBodies.setAppend( getSwitchedBodies(*G.elem(-3), *G.elem(-2)) );
  return switchedBodies.N*7;
}

void TM_FixSwichedObjects::phi(arr& y, arr& J, const WorldL& G) {
  //TODO: so far this only fixes switched objects to zero pose vel
  //better: constrain to zero relative velocity with BOTH, pre-attached and post-attached
  
  uint M=7;
  uintA switchedBodies = getSwitchedBodies(*G.elem(-2), *G.elem(-1));
//  if(order==2) switchedBodies.setAppend( getSwitchedBodies(*G.elem(-3), *G.elem(-2)) );
  y.resize(M*switchedBodies.N).setZero();
  if(&J) {
    uint xbarDim=0;
    for(auto& W:G) xbarDim+=W->q.N;
    J.resize(M*switchedBodies.N, xbarDim).setZero();
  }
  for(uint i=0; i<switchedBodies.N; i++) {
    uint id = switchedBodies(i);
    rai::Frame *b0 = G.elem(-2)->frames(id);    CHECK(&b0->K==G.elem(-2),"");
    rai::Frame *b1 = G.elem(-1)->frames(id);    CHECK(&b1->K==G.elem(-1),"");
    CHECK_EQ(b0->ID, b1->ID, "");
    CHECK_EQ(b0->name, b1->name, "");
    
    if(b0->name.startsWith("slider")) continue; //warning: this introduces zeros in y and J -- but should be ok
    
    if(b1->flags && (b1->flags & (1<<FL_impulseExchange))) continue;
    
//    if(order==2){
//      rai::Frame *b2 = G.elem(-1)->frames(id);
//      if(b2->flags & (1<<FL_impulseExchange)) continue;
//    }

    if(order==1) { //absolute velocities
      TM_Default pos(TMT_pos, id);
      pos.order=1;
      pos.TaskMap::phi(y({M*i,M*i+2})(), (&J?J({M*i,M*i+2})():NoArr), G);
      
      TM_Default quat(TMT_quat, id); //mt: NOT TMT_quatDiff!! (this would compute the diff to world, which zeros the w=1...)
      // flip the quaternion sign if necessary
      quat.flipTargetSignOnNegScalarProduct = true;
      quat.order=1;
      quat.TaskMap::phi(y({M*i+3,M*i+6})(), (&J?J({M*i+3,M*i+6})():NoArr), G);
    } else if(order==2) { //absolute accelerations
      TM_Default pos(TMT_pos, id);
      pos.order=2;
      pos.TaskMap::phi(y({M*i,M*i+2})(), (&J?J({M*i,M*i+2})():NoArr), G);
      
      TM_Default quat(TMT_quat, id); //mt: NOT TMT_quatDiff!! (this would compute the diff to world, which zeros the w=1...)
      // flip the quaternion sign if necessary
      quat.flipTargetSignOnNegScalarProduct = true;
      quat.order=2;
      quat.TaskMap::phi(y({M*i+3,M*i+6})(), (&J?J({M*i+3,M*i+6})():NoArr), G);
    } else NIY;
    
    //    if(sumOfSqr(y)>1e-3) cout <<"body " <<b0->name <<" causes switch costs " <<sumOfSqr(y) <<" at t=" <<t <<" y=" <<y <<endl;
#if 0 //OBSOLETE: relative velocities
    TM_Default pos(TMT_posDiff, b0->shape->index, NoVector, b1->shape->index);
    pos.order=1;
    pos.TaskMap::phi(y({M*i,M*i+2})(), (&J?J({M*i,M*i+2})():NoArr), G);
    
    TM_Default quat(TMT_quatDiff, j0->to->shape->index/*, NoVector, j0->from->shape->index*/);
    // flipp the quaternion sign if necessary
    quat.flipTargetSignOnNegScalarProduct = true;
    quat.order=1;
    quat.TaskMap::phi(y({M*i+3,M*i+6})(), (&J?J({M*i+3,M*i+6})():NoArr), G);
#endif
  }
}

#if 0

(mt:) These are remains for debugging: testing the Jacobians(I used this to find the top sort error...)

  if(&J && t==3) {
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
    pos.phi(yt, Jt, *G.last());
    
    G.elem(-1)->checkConsistency();
    G.elem(-2)->checkConsistency();
    FILE("z.last") <<*G.elem(-1);
    cout <<"\n*** A ***\n" <<J*sqrt(1000.) <<endl;
    cout <<"\n*** C ***\n" <<Jt*sqrt(1000.) <<endl;
    
    if(true) {
//          const char* filename="z.last";
      const char* shape="obj1";
      rai::KinematicWorld K(*G.elem(-1));
      FILE("z.last2") <<K;
//          rai::KinematicWorld K(filename);
//          K.setJointState(G.elem(-1)->q);
      rai::Shape *sh=K.getShapeByName(shape);
      TM_Default pos(TMT_posDiff, sh->index);
      arr y,J;
      pos.phi(y, J, K);
      cout <<"\n*** B ***\n" <<J*sqrt(1000.) <<endl;
      
      VectorFunction f = ([&pos, &K](arr& y, arr& J, const arr& x) -> void {
        K.setJointState(x);
        pos.phi(y,J,K);
      });
      
      checkJacobian(f, K.q, 1e-4);
      
//          exit(0);
//          rai::wait();
    }
    
  }
#endif
