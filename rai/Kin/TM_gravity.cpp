#include "TM_gravity.h"
#include <Kin/flag.h>
#include <Kin/frame.h>
#include <Kin/contact.h>
#include <Kin/TM_default.h>
#include <Kin/TM_PairCollision.h>

void TM_Gravity::phi(arr &y, arr &J, const WorldL &Ktuple, double tau, int t){

  y.clear();
  if(&J) J.clear();

  mlr::KinematicWorld& K = *Ktuple(-1);

  if(order==0) HALT("that doesn't make sense");

  if(order==1){
    arr p0, J0, p1, J1, pc, Jc;
    //check equal # of frames in each world
    for(mlr::Frame *a:K.frames){
   //      if(a->inertia && a->inertia->type==mlr::BT_dynamic){
         if(a->flags & (1<<FL_gravityAcc)){
//    uint nf = K.frames.N;
//    for(uint i=0;i<nf;i++){
//      mlr::Frame *a = K.frames(i);
//      if(a->inertia && a->inertia->type==mlr::BT_dynamic){
        TM_Default pos(TMT_pos, a->ID);
        pos.order=1;
        pos.TaskMap::phi(p0, (&J?J0:NoArr), Ktuple, tau, t);

        arr v_ref = {0.,0.,-.1};
        arr Jv_ref = zeros(3, K.q.N);
#if 0
        if(false && a->contacts.N){
          for(mlr::Contact *c:a->contacts){
            if(&c->a == a){
              K.kinematicsVec(pc, (&J?Jc:NoArr), a, c->a_rel);
            }else{
              CHECK_EQ(&c->b, a, "");
              K.kinematicsVec(pc, (&J?Jc:NoArr), a, c->b_rel);
            }
            arr rel = pc;
            double rel2=sumOfSqr(rel);
            if(&J){
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
        if(&J){
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
        if(&J){
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

  if(order==2){
    arr acc, Jacc;
    arr acc_ref = {0.,0.,-9.81};
    arr Jacc_ref = zeros(3, K.q.N);
    for(mlr::Frame *a:K.frames){
      if(a->flags & (1<<FL_gravityAcc)){
        TM_Default pos(TMT_posDiff, a->ID);
        pos.order=2;
        pos.TaskMap::phi(acc, (&J?Jacc:NoArr), Ktuple, tau, t);

        arr err = acc - acc_ref;
        arr Jerr = Jacc;

        y.append(err);

        if(&J){
          expandJacobian(Jacc_ref, Ktuple);
          Jerr -= Jacc_ref;
          J = Jerr;
        }

        if(a->contacts.N){
          CHECK_EQ(a->contacts.N, 1, "");
          for(mlr::Contact *con:a->contacts){

            arr d, Jd;
            TM_PairCollision dist(con->a.ID, con->b.ID, true, false);
            dist.phi(d, (&J?Jd:NoArr), K);
            if(&J) expandJacobian(Jd, Ktuple);
            d*=1.;
            if(&J) Jd *= 1.;

            arr c, Jc;
            TM_PairCollision coll(con->a.ID, con->b.ID, false, true);
            coll.phi(c, (&J?Jc:NoArr), K);
            normalizeWithJac(c, Jc);
            if(&J) expandJacobian(Jc, Ktuple);

//            cout <<"time " <<t <<" frame " <<a->name <<" norm=" <<c <<" dist=" <<d <<endl;
#if 0
            if(&J) J -= ( c*~c*J + c*~y*Jc + scalarProduct(c,y)*Jc );
            y -= c*scalarProduct(c,y);
#else
            if(&J) J -= (1.-d.scalar())*( c*~c*J + c*~y*Jc + scalarProduct(c,y)*Jc ) - c*scalarProduct(c,y)*Jd;
            y -= c*(1.-d.scalar())*scalarProduct(c,y);
#endif

#if 0
            if(&J){
              arr tmp = scalarProduct(c,err)*Jd + d*~c*Jerr + d*~err*Jc;
              J.append(tmp);
            }
            y.append(d*scalarProduct(c,err));
#else
//            if(&J){
//              arr tmp = scalarProduct(c,err)*Jd + d*~c*Jerr + d*~err*Jc;
//              J += c*tmp + d.scalar()*scalarProduct(c,err)*Jc;
//            }
//            y += c*d.scalar()*scalarProduct(c,err);

            y.append(0.);
            if(&J) J.append(zeros(1, J.d1));
#endif
          }
        }else{
            y.append(0.);
            if(&J) J.append(zeros(1, J.d1));
        }
      }
    }
  }

  uintA KD = getKtupleDim(Ktuple);
  if(&J) J.reshape(y.N, KD.last());
}

uint TM_Gravity::dim_phi(const WorldL &Ktuple, int t){
  mlr::KinematicWorld& K = *Ktuple(-1);
  uint d = 0;
  for(mlr::Frame *a: K.frames) if(a->flags & (1<<FL_gravityAcc)){
    d+=4;
  }
  return d;
}
