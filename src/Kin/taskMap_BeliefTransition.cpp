#include "taskMap_BeliefTransition.h"
#include "frame.h"

uint TaskMap_BeliefTransition::dim_phi(const mlr::KinematicWorld& G){
  uint n=0;
  for(mlr::Joint *j : G.fwdActiveJoints) if(j->uncertainty){
    n += j->dim;
  }
  return n;
}

void TaskMap_BeliefTransition::phi(arr &y, arr &J, const WorldL &G, double tau, int t){
  //-- transition costs
//  double xi = mlr::sigmoid(-x(t+1,0)/margin+1);
  double xi = 0.;
  double b0 = .01;
  double alpha = 10.;

  uint i=0;
  y.resize(dim_phi(*G.last())).setZero();
  if(&J) J.resize(y.N, G.N, G.elem(-1)->q.N).setZero();

  for(mlr::Joint *j1 : G.elem(-1)->fwdActiveJoints) if(j1->uncertainty){
    mlr::Joint *j0 = G.elem(-2)->frames(j1->to()->ID)->joint();
    CHECK(j0, "");
    CHECK(j0->uncertainty, "");
    CHECK_EQ(j0->dim, j1->dim, "");
    for(uint d=j0->dim;d<2*j0->dim;d++){
      y(i) = G.elem(-1)->q(j1->qIndex+d) - (1.-tau*alpha*xi)*G.elem(-2)->q(j0->qIndex+d) - tau*b0;
      if(&J){
        J(i, G.N-1, j1->qIndex+d) = 1.;
        J(i, G.N-2, j0->qIndex+d) = -(1.-tau*alpha*xi);
//        J(1,1,0) = beliefDynPrec * (tau*x(t+1,1)) * alpha*xi*(1.-xi)/margin*(-1.); //dependence of xi on q...
      }
      i++;
    }
  }

  if(&J) J.reshape(y.N, G.N*G.elem(-1)->q.N);
  CHECK_EQ(i, y.N, "");
}
