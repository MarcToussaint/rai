/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_FixSwitchedObjects.h"
#include "F_qFeatures.h"
#include "TM_default.h"
#include "frame.h"
#include "flag.h"
#include "TM_angVel.h"

uint TM_FixSwichedObjects::dim_phi(const ConfigurationL& G) {
  uintA switchedBodies = getSwitchedBodies(*G.elem(-2), *G.elem(-1));
//  if(order==2) switchedBodies.setAppend( getSwitchedBodies(*G.elem(-3), *G.elem(-2)) );
  return switchedBodies.N*6;
}

void TM_FixSwichedObjects::phi(arr& y, arr& J, const ConfigurationL& Ktuple) {
  //TODO: so far this only fixes switched objects to zero pose vel
  //better: constrain to zero relative velocity with BOTH, pre-attached and post-attached

  uint M=6;
  uintA switchedBodies = getSwitchedBodies(*Ktuple.elem(-2), *Ktuple.elem(-1));
//  if(order==2) switchedBodies.setAppend( getSwitchedBodies(*G.elem(-3), *G.elem(-2)) );
  y.resize(M*switchedBodies.N).setZero();
  if(!!J) {
    uint xbarDim=0;
    for(auto& W:Ktuple) xbarDim+=W->q.N;
    J.resize(M*switchedBodies.N, xbarDim).setZero();
  }
  for(uint i=0; i<switchedBodies.N; i++) {
    uint id = switchedBodies(i);
    rai::Frame* b0 = Ktuple.elem(-2)->frames(id);    CHECK(&b0->C==Ktuple.elem(-2), "");
    rai::Frame* b1 = Ktuple.elem(-1)->frames(id);    CHECK(&b1->C==Ktuple.elem(-1), "");
    CHECK_EQ(b0->ID, b1->ID, "");
    CHECK_EQ(b0->name, b1->name, "");
//    cout <<"SWITCH " <<b0->parent->name <<'-' <<b0->name <<" => " <<b1->parent->name <<'-' <<b1->name <<endl;

    if(b0->name.startsWith("slider")) continue; //warning: this introduces zeros in y and J -- but should be ok

    if(b1->flags && (b1->flags & (1<<FL_impulseExchange))) continue;

    rai::Frame* b0Link = b0->getUpwardLink();
    if(b0Link->joint && b0Link->joint->type!=rai::JT_rigid) {
      continue;
    }

    b0Link = b0Link->parent;
    CHECK(b0Link, "");

//    if(order==2){
//      rai::Frame *b2 = G.elem(-1)->frames(id);
//      if(b2->flags & (1<<FL_impulseExchange)) continue;
//    }

    if(order==1) { //absolute velocities
#if 0
      TM_Default pos(TMT_pos, id, NoVector, b0Parent->ID);
      pos.order=1;
      pos.Feature::__phi(y({M*i, M*i+2})(), (!!J?J({M*i, M*i+2})():NoArr), Ktuple);

      TM_Default quat(TMT_quat, id, NoVector, b0Parent->ID); //mt: NOT TMT_quatDiff!! (this would compute the diff to world, which zeros the w=1...)
      // flip the quaternion sign if necessary
      quat.flipTargetSignOnNegScalarProduct = true;
      quat.order=1;
      quat.Feature::__phi(y({M*i+3, M*i+6})(), (!!J?J({M*i+3, M*i+6})():NoArr), Ktuple);
#else
      TM_Default pos(TMT_pos, id);
      pos.order=1;
      pos.Feature::__phi(y({M*i, M*i+2})(), (!!J?J({M*i, M*i+2})():NoArr), Ktuple);

      TM_AngVel rot(id);
      rot.order=1;
      rot.phi(y({M*i+3, M*i+5})(), (!!J?J({M*i+3, M*i+5})():NoArr), Ktuple);
#endif
    } else if(order==2) { //absolute accelerations
#if 0
      TM_Default pos(TMT_pos, id);
      pos.order=2;
      pos.Feature::__phi(y({M*i, M*i+2})(), (!!J?J({M*i, M*i+2})():NoArr), Ktuple);

      TM_Default quat(TMT_quat, id); //mt: NOT TMT_quatDiff!! (this would compute the diff to world, which zeros the w=1...)
      // flip the quaternion sign if necessary
      quat.flipTargetSignOnNegScalarProduct = true;
      quat.order=2;
      quat.Feature::__phi(y({M*i+3, M*i+6})(), (!!J?J({M*i+3, M*i+6})():NoArr), Ktuple);
#elif 0
      TM_Default pose(TMT_pose, id, NoVector, b0Parent->ID);
      pose.order=2;
      pose.Feature::__phi(y({M*i, M*i+6})(), (!!J?J({M*i, M*i+6})():NoArr), Ktuple);
#else
      TM_Default pos(TMT_pos, id);
      pos.order=2;
      pos.Feature::__phi(y({M*i, M*i+2})(), (!!J?J({M*i, M*i+2})():NoArr), Ktuple);

      TM_AngVel rot(id);
      rot.order=2;
      rot.phi(y({M*i+3, M*i+5})(), (!!J?J({M*i+3, M*i+5})():NoArr), Ktuple);
#endif
    } else NIY;
  }
}
