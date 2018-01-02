#include "TM_gravity.h"
#include <Kin/flag.h>
#include <Kin/frame.h>
#include <Kin/contact.h>
#include <Kin/taskMap_default.h>

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
        TaskMap_Default pos(posTMT, a->ID);
        pos.order=1;
        pos.TaskMap::phi(p0, (&J?J0:NoArr), Ktuple, tau, t);

        arr v_ref = {0.,0.,-.1};
        arr Jv_ref = zeros(3, K.q.N);
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
    arr acc_ref = {0.,0.,-1.}; //-9.81};
    arr Jacc_ref = zeros(3, K.q.N);
    for(mlr::Frame *a:K.frames){
//      if(a->inertia && a->inertia->type==mlr::BT_dynamic){
      if(a->flags & (1<<FL_gravityAcc)){
        TaskMap_Default pos(posTMT, a->ID);
        pos.order=2;
        pos.TaskMap::phi(acc, (&J?Jacc:NoArr), Ktuple, tau, t);

        y.append(acc - acc_ref);
        if(&J){
          arr tmp = zeros(3, Jacc.d1);
          tmp.setMatrixBlock(-Jacc_ref, 0, tmp.d1-Jacc_ref.d1);
          J.append(Jacc + tmp);
          J.reshape(y.N, Jacc.d1);
        }
      }
    }
  }

  uintA KD = getKtupleDim(Ktuple);
  J.reshape(y.N, KD.last());
}

uint TM_Gravity::dim_phi(const WorldL &Ktuple, int t){
  mlr::KinematicWorld& K = *Ktuple(-1);
  uint d = 0;
  for(mlr::Frame *a: K.frames) if(a->flags & (1<<FL_gravityAcc)){
    d+=3;
  }
  return d;
}
