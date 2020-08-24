/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_FlagConstraints.h"
#include "F_qFeatures.h"
#include "TM_default.h"
#include "frame.h"
#include "flag.h"

//===========================================================================

bool JointDidNotSwitch(const rai::Frame* a1, const ConfigurationL& Ktuple, int order) {
  CHECK_EQ(&a1->C, Ktuple.last(), "");
  if(order<1) return true;
  for(int i=0; i<order; i++) {
    if(a1->ID >= Ktuple(-2-i)->frames.N) return false;
    rai::Frame* a0 = Ktuple(-2-i)->frames(a1->ID);
    rai::Joint* j0 = a0->joint, *j1 = a1->joint;
    if(!j0 || !j1
        || j0->type!=j1->type
        || j0->from()->ID!=j1->from()->ID) return false;
  }
  return true;
}

//===========================================================================

uint TM_FlagConstraints::dim_phi(const ConfigurationL& Ktuple) {
  uint d=0;
  for(rai::Frame* a : Ktuple.last()->frames) {
    if(a->flags & (1<<FL_zeroVel)) d += 7;
    if(order>=2 && a->flags & (1<<FL_zeroAcc) && !(a->flags & (1<<FL_impulseExchange))) d += 7;
    if(order>=2 && a->flags & (1<<FL_gravityAcc) && !(a->flags & (1<<FL_impulseExchange))) d += 7;
    if(a->flags & (1<<FL_zeroQVel)) if(JointDidNotSwitch(a, Ktuple, 1)) d += a->joint->dim;
  }
  return d;
}

void TM_FlagConstraints::phi(arr& y, arr& J, const ConfigurationL& Ktuple) {
  CHECK_GE(order, 1, "FlagConstraints needs k-order 1");

  rai::Configuration& K = *Ktuple.last();

  y.resize(dim_phi(Ktuple)).setZero();
  if(!!J) {
    uintA xbarDim=getKtupleDim(Ktuple);
    J.resize(y.N, xbarDim.last()).setZero();
  }

  uint d=0;
  for(rai::Frame* a : K.frames) if(a->flags) {
      if(a->flags & (1<<FL_zeroVel)) {
        TM_Default pos(TMT_pos, a->ID);
        pos.order=1;
        pos.Feature::__phi(y({d, d+2})(), (!!J?J({d, d+2})():NoArr), Ktuple);

        TM_Default quat(TMT_quat, a->ID); //mt: NOT TMT_quatDiff!! (this would compute the diff to world, which zeros the w=1...)
        // flip the quaternion sign if necessary
        quat.flipTargetSignOnNegScalarProduct = true;
        quat.order=1;
        quat.Feature::__phi(y({d+3, d+6})(), (!!J?J({d+3, d+6})():NoArr), Ktuple);

        d += 7;
      }

      if(order>=2 && a->flags & (1<<FL_zeroAcc) && !(a->flags & (1<<FL_impulseExchange))) {
        CHECK_GE(order, 2, "FT_zeroAcc needs k-order 2");
        TM_Default pos(TMT_pos, a->ID);
        pos.order=2;
        pos.Feature::__phi(y({d, d+2})(), (!!J?J({d, d+2})():NoArr), Ktuple);

        TM_Default quat(TMT_quat, a->ID); //mt: NOT TMT_quatDiff!! (this would compute the diff to world, which zeros the w=1...)
        // flip the quaternion sign if necessary
        quat.flipTargetSignOnNegScalarProduct = true;
        quat.order=2;
        quat.Feature::__phi(y({d+3, d+6})(), (!!J?J({d+3, d+6})():NoArr), Ktuple);

        d += 7;
      }

      if(order>=2 && a->flags & (1<<FL_gravityAcc) && !(a->flags & (1<<FL_impulseExchange))) {
        CHECK_GE(order, 2, "FT_zeroAcc needs k-order 2");
        TM_Default pos(TMT_pos, a->ID);
        pos.order=2;
        pos.Feature::__phi(y({d, d+2})(), (!!J?J({d, d+2})():NoArr), Ktuple);
        y(d+2) += g;

        TM_Default quat(TMT_quat, a->ID); //mt: NOT TMT_quatDiff!! (this would compute the diff to world, which zeros the w=1...)
        // flip the quaternion sign if necessary
        quat.flipTargetSignOnNegScalarProduct = true;
        quat.order=1;
        quat.Feature::__phi(y({d+3, d+6})(), (!!J?J({d+3, d+6})():NoArr), Ktuple);
        if(false) { //rotational friction
          double eps = 1e-2;
          arr w, Jw;
          quat.order=1;
          quat.Feature::__phi(w, (!!J?Jw:NoArr), Ktuple);
          y({d+3, d+6}) += eps*w;
          if(!!J) J({d+3, d+6}) += eps*Jw;
        }

        d += 7;
      }

      if(a->flags & (1<<FL_zeroQVel)) if(JointDidNotSwitch(a, Ktuple, 1)) {
          uint jdim = a->joint->dim;

          F_qItself q({a->ID}, false);
          q.order=1;
          q.Feature::__phi(y({d, d+jdim-1})(), (!!J?J({d, d+jdim-1})():NoArr), Ktuple);

          d += jdim;
        }

    }

  CHECK_EQ(d, y.N, "");
}

//===========================================================================

uint TM_FlagCosts::dim_phi(const ConfigurationL& Ktuple) {
  uint d=0;
  for(rai::Frame* a : Ktuple.last()->frames) {
    if(order>=2 && a->flags & (1<<FL_xPosAccCosts)) d+=3;
    if(a->flags & (1<<FL_xPosVelCosts)) d+=3;
    if(order>=2 && a->flags & (1<<FL_qCtrlCostAcc)) if(JointDidNotSwitch(a, Ktuple, 2)) d += a->joint->dim;
    if(order>=1 && a->flags & (1<<FL_qCtrlCostVel)) if(JointDidNotSwitch(a, Ktuple, 1)) d += a->joint->dim;
  }
  return d;
}

void TM_FlagCosts::phi(arr& y, arr& J, const ConfigurationL& Ktuple) {
  CHECK_GE(order, 1, "FlagConstraints needs k-order 1");

  rai::Configuration& K = *Ktuple.last();

  y.resize(dim_phi(Ktuple)).setZero();
  if(!!J) {
    uintA xbarDim=getKtupleDim(Ktuple);
    J.resize(y.N, xbarDim.last()).setZero();
  }

  uint d=0;
  for(rai::Frame* a : K.frames) if(a->flags) {

      if(order>=2 && a->flags & (1<<FL_xPosAccCosts)) {
        CHECK_GE(order, 2, "FT_zeroAcc needs k-order 2");
        TM_Default pos(TMT_pos, a->ID);
        pos.order=2;
        pos.Feature::__phi(y({d, d+2})(), (!!J?J({d, d+2})():NoArr), Ktuple);

        d += 3;
      }

      if(a->flags & (1<<FL_xPosVelCosts)) {
        CHECK_GE(order, 1, "FT_velCost needs k-order 2");
        TM_Default pos(TMT_pos, a->ID);
        pos.order=1;
        pos.Feature::__phi(y({d, d+2})(), (!!J?J({d, d+2})():NoArr), Ktuple);

        d += 3;
      }

      if(order>=2 && a->flags & (1<<FL_qCtrlCostAcc)) if(JointDidNotSwitch(a, Ktuple, 2)) {
          uint jdim = a->joint->dim;

          F_qItself q({a->ID}, false);
          q.order=2;
          q.Feature::__phi(y({d, d+jdim-1})(), (!!J?J({d, d+jdim-1})():NoArr), Ktuple);

          d += jdim;
        }

      if(order>=1 && a->flags & (1<<FL_qCtrlCostVel)) if(JointDidNotSwitch(a, Ktuple, 1)) {
          uint jdim = a->joint->dim;

          F_qItself q({a->ID}, false);
          q.order=1;
          q.Feature::__phi(y({d, d+jdim-1})(), (!!J?J({d, d+jdim-1})():NoArr), Ktuple);

          d += jdim;
        }

    }

  CHECK_EQ(d, y.N, "");
}
