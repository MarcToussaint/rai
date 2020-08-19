/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_BeliefTransition.h"
#include "frame.h"

uint TM_BeliefTransition::dim_phi(const rai::Configuration& G) {
  uint n=0;
  for(rai::Joint* j : G.activeJoints) if(j->uncertainty) {
      n += j->dim;
    }
  return n;
}

void forsyth(double& f, double& df_dx, double x, double a) {
  double x2=x*x;
  double a2=a*a;
  f = x2/(x2+a2);
  df_dx = 2.*x/(x2+a2) - 2.*x*x2/((x2+a2)*(x2+a2));
}

double Forsyth(arr& J, const arr& x, double a) {
  double x2 = sumOfSqr(x);
  double x2_a2 = x2 + a*a;
  double f = x2/x2_a2;
  if(!!J) {
    J = (2.*(1.-f)/x2_a2) * x;
    J.reshape(1, x.N);
  }
  return f;
};

void TM_BeliefTransition::phi(arr& y, arr& J, const ConfigurationL& Ktuple) {
  uint i=0;
  y.resize(dim_phi(*Ktuple.last())).setZero();
  if(!!J) J.resize(y.N, Ktuple.N, Ktuple.elem(-1)->q.N).setZero();

  double tau = Ktuple(-1)->frames(0)->tau; // - Ktuple(-2)->frames(0)->time;

  //parameters of the belief transition
  double xi = 0.;
  double b0 = .01;
  arr J_xi = zeros(Ktuple.N, Ktuple.elem(-1)->q.N);
  if(viewError) {
    arr y_view, J_view;
    viewError->__phi(y_view, J_view, Ktuple);
    y_view *= 2.;
    J_view *= 2.;
    xi = 1. - Forsyth(J_xi, y_view, 1.);
    J_xi = - J_xi * J_view;
    J_xi.reshape(Ktuple.N, Ktuple.elem(-1)->q.N);
    xi *= 2.;
    J_xi *= 2.;
  }

  for(rai::Joint* j1 : Ktuple.elem(-1)->activeJoints) if(j1->uncertainty) {
      rai::Joint* j0 = Ktuple.elem(-2)->frames(j1->frame->ID)->joint;
      CHECK(j0, "");
      CHECK(j0->uncertainty, "");
      CHECK_EQ(j0->dim, j1->dim, "");
      for(uint d=j0->dim; d<2*j0->dim; d++) {
        y(i) = Ktuple.elem(-1)->q(j1->qIndex+d) - (1.-tau*xi)*Ktuple.elem(-2)->q(j0->qIndex+d) - tau*b0;
//      if(y(i)<0.) y(i)=0.; //hack: the finite integration may lead to negative values
        if(!!J) {
          J(i, Ktuple.N-1, j1->qIndex+d) = 1.;
          J(i, Ktuple.N-2, j0->qIndex+d) = -(1.-tau*xi);

          J[i] += (tau*Ktuple.elem(-2)->q(j0->qIndex+d)) * J_xi;
        }
        i++;
      }
    }

  if(!!J) J.reshape(y.N, Ktuple.N*Ktuple.elem(-1)->q.N);
  CHECK_EQ(i, y.N, "");
}
