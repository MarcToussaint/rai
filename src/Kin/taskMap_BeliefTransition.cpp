#include "taskMap_BeliefTransition.h"
#include "frame.h"

uint TaskMap_BeliefTransition::dim_phi(const mlr::KinematicWorld& G){
  uint n=0;
  for(mlr::Joint *j : G.fwdActiveJoints) if(j->uncertainty){
    n += j->dim;
  }
  return n;
}

void forsyth(double& f, double& df_dx, double x, double a){
  double x2=x*x;
  double a2=a*a;
  f = x2/(x2+a2);
  df_dx = 2.*x/(x2+a2) - 2.*x*x2/((x2+a2)*(x2+a2));
}

double Forsyth(arr& J, const arr& x, double a){
  double x2 = sumOfSqr(x);
  double x2_a2 = x2 + a*a;
  double f = x2/x2_a2;
  if(&J){
    J = (2.*(1.-f)/x2_a2) * x;
    J.reshape(1, x.N);
  }
  return f;
};

void TaskMap_BeliefTransition::phi(arr &y, arr &J, const WorldL &G, double tau, int t){
  uint i=0;
  y.resize(dim_phi(*G.last())).setZero();
  if(&J) J.resize(y.N, G.N, G.elem(-1)->q.N).setZero();

  //parameters of the belief transition
  double xi = 0.;
  double b0 = .01;
  arr J_xi = zeros(G.N, G.elem(-1)->q.N);
  if(viewError){
    arr y_view, J_view;
    viewError->phi(y_view, J_view, G, tau, t);
    y_view *= 2.;
    J_view *= 2.;
    xi = 1. - Forsyth(J_xi, y_view, 1.);
    J_xi = - J_xi * J_view;
    J_xi.reshape(G.N, G.elem(-1)->q.N);
    xi *= 2.;
    J_xi *= 2.;
  }

  for(mlr::Joint *j1 : G.elem(-1)->fwdActiveJoints) if(j1->uncertainty){
    mlr::Joint *j0 = G.elem(-2)->frames(j1->to()->ID)->joint();
    CHECK(j0, "");
    CHECK(j0->uncertainty, "");
    CHECK_EQ(j0->dim, j1->dim, "");
    for(uint d=j0->dim;d<2*j0->dim;d++){
      y(i) = G.elem(-1)->q(j1->qIndex+d) - (1.-tau*xi)*G.elem(-2)->q(j0->qIndex+d) - tau*b0;
//      if(y(i)<0.) y(i)=0.; //hack: the finite integration may lead to negative values
      if(&J){
        J(i, G.N-1, j1->qIndex+d) = 1.;
        J(i, G.N-2, j0->qIndex+d) = -(1.-tau*xi);

        J[i] += (tau*G.elem(-2)->q(j0->qIndex+d)) * J_xi;
      }
      i++;
    }
  }

  if(&J) J.reshape(y.N, G.N*G.elem(-1)->q.N);
  CHECK_EQ(i, y.N, "");
}
