/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_gravity.h"
#include "flag.h"
#include "frame.h"
#include "forceExchange.h"
#include "TM_default.h"
#include "F_PairCollision.h"
#include "TM_angVel.h"

bool JointDidNotSwitch(const rai::Frame* a1, const ConfigurationL& Ktuple, int order);

void shapeFunction(double& x, double& dx) {
  if(x>0.) { x=0.; dx=0.; return; }
  if(x>1. || x<-1.) { x=1.; dx=0.; return; }
  double x2=x*x;
  dx = 1.5*(1.-x2);
  x = 0.5*(3.*x-x2*x);
}

TM_Gravity::TM_Gravity() {
  gravity = rai::getParameter<double>("TM_Gravity/gravity", 9.81);
}

void TM_Gravity::phi(arr& y, arr& J, const ConfigurationL& Ktuple) {

  y.clear();
  if(!!J) J.clear();

  if(order==0) HALT("that doesn't make sense");

  if(order==1) {
    rai::Configuration& K = *Ktuple(-1);

    arr p0, J0, p1, J1, pc, Jc;
    //check equal # of frames in each world
    for(rai::Frame* a:K.frames) {
      //      if(a->inertia && a->inertia->type==rai::BT_dynamic){
      if(a->flags & (1<<FL_gravityAcc)) {
//    uint nf = K.frames.N;
//    for(uint i=0;i<nf;i++){
//      rai::Frame *a = K.frames(i);
//      if(a->inertia && a->inertia->type==rai::BT_dynamic){
        TM_Default pos(TMT_pos, a->ID);
        pos.order=1;
        pos.Feature::__phi(p0, (!!J?J0:NoArr), Ktuple);

        arr v_ref = {0., 0., -gravity};
        arr Jv_ref = zeros(3, K.q.N);
#if 0
        if(false && a->forces.N) {
          for(rai::ForceExchange* c:a->forces) {
            if(&c->a == a) {
              K.kinematicsVec(pc, (!!J?Jc:NoArr), a, c->a_rel);
            } else {
              CHECK_EQ(&c->b, a, "");
              K.kinematicsVec(pc, (!!J?Jc:NoArr), a, c->b_rel);
            }
            arr rel = pc;
            double rel2=sumOfSqr(rel);
            if(!!J) {
              arr Jrel = Jc;
              arr Jrel2 = ((-2./(rel2*rel2)) * ~rel) * Jrel;
              Jv_ref -= ((rel * ~rel)/rel2) * Jv_ref; //derive w.r.t. v_ref
              Jv_ref -= (scalarProduct(rel, v_ref)/rel2) * Jrel; //derivative w.r.t. first rel
              Jv_ref -= ((rel * ~v_ref)/rel2) * Jrel; //derivative w.r.t. second rel
              Jv_ref -= (scalarProduct(rel, v_ref) * rel) * Jrel2; //derivative w.r.t. rel2
            }
            v_ref -= rel * (scalarProduct(rel, v_ref)/rel2);
          }
        }
#endif

        //z-velocity only, compared to default .1 drop velocity
#if 1
        y.append(p0 - v_ref);
        if(!!J) {
//          uint Ktuple_dim = 0;
//          for(auto *K:Ks) Ktuple_dim += K->q.N;
//          arr tmp = zeros(3, Ktuple_dim);
//          tmp.setMatrixBlock(- J0, 0, Ktuple_dim-J0.d1-J1.d1);
//          tmp.setMatrixBlock(  J1 /*- Jv_ref*/, 0, Ktuple_dim-J1.d1);
          arr tmp = zeros(3, J0.d1);
          tmp.setMatrixBlock(-Jv_ref, 0, tmp.d1-Jv_ref.d1);
          J.append(J0 + tmp);
          J.reshape(y.N, J0.d1);
        }
#else
        y.append(scalarProduct(v_ref, p0) - .1);
        if(!!J) {
//          uint Ktuple_dim = 0;
//          for(auto *K:Ks) Ktuple_dim += K->q.N;
//          arr tmp = zeros(1, Ktuple_dim);
//          tmp.setMatrixBlock(- (~v_ref*J0) /tau, 0, Ktuple_dim-J0.d1-J1.d1);
//          tmp.setMatrixBlock(  (~v_ref*J1 + ~(p1-p0)*Jv_ref) /tau, 0, Ktuple_dim-J1.d1);
          arr tmp = zeros(3, J0.d1);
          tmp.setMatrixBlock(Jv_ref, 0, tmp.d1-Jv_ref.d1);
          J.append(~v_ref*J0 + ~p0*tmp);
          J.reshape(y.N, J0.d1);
        }
#endif
      }
    }
  }

  if(order==2) {
    rai::Configuration& K = *Ktuple(-2);

    arr acc, Jacc;
    arr acc_ref = {0., 0., -gravity};
    arr Jacc_ref = zeros(3, K.q.N);
    for(rai::Frame* a:K.frames) {
      if(a->flags & (1<<FL_gravityAcc)) {
        TM_Default pos(TMT_posDiff, a->ID);
        pos.order=2;
        pos.Feature::__phi(acc, (!!J?Jacc:NoArr), Ktuple);

        arr err = acc - acc_ref;
        arr Jerr = Jacc;

        y.append(err);

        if(!!J) {
          expandJacobian(Jacc_ref, Ktuple);
          Jerr -= Jacc_ref;
          J = Jerr;
        }

        if(a->forces.N) {
          CHECK_EQ(a->forces.N, 1, "");
          for(rai::ForceExchange* con:a->forces) {

            arr d, Jd;
            F_PairCollision dist(con->a.ID, con->b.ID, F_PairCollision::_negScalar, false);
            dist.phi(d, (!!J?Jd:NoArr), *Ktuple(-2));
            if(!!J) expandJacobian(Jd, Ktuple, -2);
            d *= 1.;
            if(!!J) Jd *= 1.;

//            arr d2, Jd2;
//            F_PairCollision dist2(con->a.ID, con->b.ID, true, false);
//            dist2.phi(d2, (!!J?Jd2:NoArr), *Ktuple(-1));
//            if(!!J) expandJacobian(Jd2, Ktuple, -1);
//            d += d2;
//            if(!!J) Jd += Jd2;

//            d.scalar() = tanh(d.scalar());
//            if(!!J) Jd *= (1.-d.scalar()*d.scalar());
//            double dd;
//            shapeFunction(d.scalar(), dd);
//            if(!!J) Jd *= dd;

            arr c, Jc;
            F_PairCollision coll(con->a.ID, con->b.ID, F_PairCollision::_vector, true);
            coll.phi(c, (!!J?Jc:NoArr), K);
            if(length(c)<1e-6) continue;
            normalizeWithJac(c, Jc);
            if(!!J) expandJacobian(Jc, Ktuple, -2);

            double sign = scalarProduct(c, err);

//            cout <<"time " <<t <<" frame " <<a->name <<" norm=" <<c <<" dist=" <<d <<endl;
#if 0
            if(!!J) J -= (c*~c*J + c*~y*Jc + scalarProduct(c, y)*Jc);
            y -= c*scalarProduct(c, y);
#elif 1
            if(sign<0.) {
              if(!!J) J -= (1.-d.scalar())*(c*~c*J + c*~y*Jc + scalarProduct(c, y)*Jc) - c*scalarProduct(c, y)*Jd;
              y -= (1.-d.scalar())*c*scalarProduct(c, y);
            }
#else
            if(sign<0.) {
              double dfactor=exp(-0.5*d.scalar()*d.scalar()/.01);
              double ddfactor = dfactor * (-d.scalar()/.01);
              if(!!J) J -= dfactor*(c*~c*J + c*~y*Jc + scalarProduct(c, y)*Jc) + ddfactor*c*scalarProduct(c, y)*Jd;
              y -= dfactor*c*scalarProduct(c, y);
            }
#endif

#if 0
            if(!!J) {
              arr tmp = scalarProduct(c, err)*Jd + d*~c*Jerr + d*~err*Jc;
              J.append(tmp);
            }
            y.append(d*scalarProduct(c, err));
#else
//            if(!!J){
//              arr tmp = scalarProduct(c,err)*Jd + d*~c*Jerr + d*~err*Jc;
//              J += c*tmp + d.scalar()*scalarProduct(c,err)*Jc;
//            }
//            y += c*d.scalar()*scalarProduct(c,err);

//            y.append(0.);
//            if(!!J) J.append(zeros(1, J.d1));
#endif
          }
        } else {
//            y.append(0.);
//            if(!!J) J.append(zeros(1, J.d1));
        }
      }
    }
  }

  uintA KD = getKtupleDim(Ktuple);
  if(!!J) J.reshape(y.N, KD.last());
}

uint TM_Gravity::dim_phi(const ConfigurationL& Ktuple) {
  rai::Configuration& K = *Ktuple(-1);
  uint d = 0;
  for(rai::Frame* a: K.frames) if(a->flags & (1<<FL_gravityAcc)) {
      d+=3;
    }
  return d;
}

TM_Gravity2::TM_Gravity2(int iShape) : i(iShape) {
  order=2;
  gravity = rai::getParameter<double>("FlagConstraints/gravity", 1.);
}

void TM_Gravity2::phi(arr& y, arr& J, const ConfigurationL& Ktuple) {
  CHECK_GE(order, 2, "needs k-order 2");

  rai::Frame* a = Ktuple(-2)->frames(i);
  if((a->flags & (1<<FL_impulseExchange))) {
    y.resize(3).setZero();
    if(!!J) J.resize(3, getKtupleDim(Ktuple).last()).setZero();
    return;
  }
  TM_LinVel pos(i);
  pos.order=2;
  pos.Feature::__phi(y, J, Ktuple);
  y(2) += gravity;

//  TM_Default quat(TMT_quat, a->ID); //mt: NOT TMT_quatDiff!! (this would compute the diff to world, which zeros the w=1...)
//  // flip the quaternion sign if necessary
//  quat.flipTargetSignOnNegScalarProduct = true;
//  quat.order=1;
//  quat.Feature::__phi(y({d+3,d+6})(), (!!J?J({d+3,d+6})():NoArr), Ktuple);
//  if(false) { //rotational friction
//    double eps = 1e-2;
//    arr w,Jw;
//    quat.order=1;
//    quat.Feature::__phi(w, (!!J?Jw:NoArr), Ktuple);
//    y({d+3,d+6}) += eps*w;
//    if(!!J) J({d+3,d+6}) += eps*Jw;
//  }

}

TM_ZeroAcc::TM_ZeroAcc(int iShape) : i(iShape) {
  order=2;
}

void TM_ZeroAcc::phi(arr& y, arr& J, const ConfigurationL& Ktuple) {
  CHECK_GE(order, 2, "needs k-order 2");

  rai::Frame* a = Ktuple(-2)->frames(i);
  if((a->flags & (1<<FL_impulseExchange))) {
    y.resize(3).setZero();
    if(!!J) J.resize(3, getKtupleDim(Ktuple).last()).setZero();
    return;
  }
  TM_Default pos(TMT_pos, i);
  pos.order=2;
  pos.Feature::__phi(y, J, Ktuple);
}

