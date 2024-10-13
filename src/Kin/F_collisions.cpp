/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "F_collisions.h"
#include "proxy.h"
#include "forceExchange.h"

#include "../Geo/pairCollision.h"
#include "../Optim/newton.h"
#include "../Gui/opengl.h"

//===========================================================================

uint F_PairCollision::dim_phi2(const FrameL& F) {
  if(type==_negScalar) {
    if(F.nd==3) { CHECK_EQ(F.d0, 1, ""); return F.d1; }
    if(F.nd==2) return F.d0;
    return 1;
  }
  return 3;
}

void F_PairCollision::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0) {  Feature::phi2(y, J, F);  return;  }
  if(F.nd>=2) {
    FrameL _F = F.ref();
    if(F.nd==3) _F.reshape(F.d1, F.d2);
    F.last()->C.kinematicsZero(y, J, dim_phi2(_F));
    arr ysub, Jsub;
    for(uint i=0; i<_F.d0; i++) {
      F_PairCollision(type).phi2(ysub, Jsub, _F[i]);
      y.setVectorBlock(ysub, i);
      if(!!J) J.setMatrixBlock(Jsub, i, 0);
    }
    return;
  }

  CHECK_EQ(F.N, 2, "");
  rai::Frame* f1 = F.elem(0);
  rai::Frame* f2 = F.elem(1);
  double r1=0., r2=0.;

  //which meshes should we collide? -> m1, m2
  rai::Mesh dot;
  dot.setDot();
  rai::Mesh* m1=&dot, *m2=&dot;
  if(f1->shape && f1->shape->type()!=rai::ST_marker) {
    r1 = f1->shape->radius();
    m1 = &f1->shape->sscCore();  if(!m1->V.N) { m1 = &f1->shape->mesh(); r1=0.; }
    if(!m1->V.N) m1 = &dot;
  }
  if(f2->shape && f2->shape->type()!=rai::ST_marker) {
    r2 = f2->shape->radius();
    m2 = &f2->shape->sscCore();  if(!m2->V.N) { m2 = &f2->shape->mesh(); r2=0.; }
    if(!m2->V.N) m2 = &dot;
  }

  //is this a point cloud collision? -> different method
  if((type==_negScalar || type==_vector) && m1->V.d0==1 && m2->V.d0>2 && !m2->T.N) {
    arr Jp1, Jp2, Jx1, Jx2;
    if(!!J) {
      f1->C.jacobian_pos(Jp1, f1, f1->ensure_X().pos);
      f2->C.jacobian_pos(Jp2, f2, f2->ensure_X().pos);
      f1->C.jacobian_angular(Jx1, f1);
      f2->C.jacobian_angular(Jx2, f2);
    }
    rai::PclCollision coll(m1->V, m2->ensure_ann(),
                           f1->ensure_X(), Jp1, Jx1,
                           f2->ensure_X(), Jp2, Jx2,
                           r1, r2, type==_vector);
    if(type==_negScalar) {
      y = -coll.y;
      if(!!J) J = -coll.J;
    } else if(type==_vector) {
      y = coll.y;
      if(!!J) J = coll.J;
    } else NIY;
    if(!!J) checkNan(J);
    return;
  }

  //compute the collision
  coll.reset();
#if 0 //use functionals!
  auto func1=f1->shape->functional();
  auto func2=f2->shape->functional();
  if(func1 && func2) {
    coll=make_shared<PairCollision>(*func1, *func2, .5*(f1->getPosition()+f2->getPosition()));
  } else {
    coll=make_shared<PairCollision>(*m1, *m2, f1->ensure_X(), f2->ensure_X(), r1, r2);
  }
#else
  coll = make_shared<rai::PairCollision>(*m1, *m2, f1->ensure_X(), f2->ensure_X(), r1, r2);
#endif

  if(neglectRadii) coll->rad1=coll->rad2=0.;

  //extract the specific feature with Jacobian
  if(type==_negScalar) {
    arr Jp1, Jp2;
    if(!!J) {
      f1->C.jacobian_pos(Jp1, f1, coll->p1);
      f2->C.jacobian_pos(Jp2, f2, coll->p2);
    }
    coll->kinDistance(y, J, Jp1, Jp2);
    y *= -1.;
    if(!!J) J *= -1.;
    if(!!J) checkNan(J);
  } else {
    arr Jp1, Jp2, Jx1, Jx2;
    if(!!J) {
      f1->C.jacobian_pos(Jp1, f1, coll->p1);
      f2->C.jacobian_pos(Jp2, f2, coll->p2);
      f1->C.jacobian_angular(Jx1, f1);
      f2->C.jacobian_angular(Jx2, f2);
    }
    if(type==_vector) coll->kinVector(y, J, Jp1, Jp2, Jx1, Jx2);
    if(type==_normal) coll->kinNormal(y, J, Jp1, Jp2, Jx1, Jx2);
    if(type==_center) coll->kinCenter(y, J, Jp1, Jp2, Jx1, Jx2);
    if(type==_p1) coll->kinPointP1(y, J, Jp1, Jp2, Jx1, Jx2);
    if(type==_p2) coll->kinPointP2(y, J, Jp1, Jp2, Jx1, Jx2);
  }
}

//===========================================================================

void F_AccumulatedCollisions::phi2(arr& y, arr& J, const FrameL& F) {
  rai::Configuration& C = F.first()->C;
  C.kinematicsZero(y, J, 1);
  for(rai::Proxy& p: C.proxies) {
    bool isSelected=false;
    if(selectAll) {
      //select based on indices, e.g. being in a single time slice of a path config
      isSelected = (p.a->ID>=F.first()->ID && p.a->ID<=F.last()->ID)
                   || (p.b->ID>=F.first()->ID && p.b->ID<=F.last()->ID);
    } else {
      //select by explicitly looking up in F
      isSelected = (!selectXor && (F.contains(p.a) || F.contains(p.b)))
                   || (selectXor && (F.contains(p.a) ^ F.contains(p.b)));
    }
    if(isSelected) {
      CHECK(p.a->shape, "");
      CHECK(p.b->shape, "");

      //early check: if swift is way out of collision, don't bother computing it precisely
      if(p.d > p.a->shape->radius() + p.b->shape->radius() + .01 + margin) continue;

      if(!p.collision) p.calc_coll();

      if(p.collision->getDistance()>margin) continue;

      arr Jp1, Jp2;
      p.a->C.jacobian_pos(Jp1, p.a, p.collision->p1);
      p.b->C.jacobian_pos(Jp2, p.b, p.collision->p2);

      arr y_dist, J_dist;
      p.collision->kinDistance(y_dist, J_dist, Jp1, Jp2);

      if(y_dist.scalar()>margin) continue; //this is the hinge: proxies contribute only when below margin

      y += margin-y_dist.scalar();
      J -= J_dist;
    }
  }
}

//===========================================================================

namespace rai {
template<class T>
arr block(const Array<T>& A, const Array<T>& B, const Array<T>& C, const Array<T>& D) {
  arr X(A.d0+C.d0, A.d1+B.d1);
  X.setMatrixBlock(A, 0, 0);
  X.setMatrixBlock(B, 0, A.d1);
  X.setMatrixBlock(C, A.d0, 0);
  X.setMatrixBlock(D, A.d0, A.d1);
  return X;
}
}

struct SweepingSDFPenetration : ScalarFunction {
  //inputs
  shared_ptr<ScalarFunction> sdf1;
  shared_ptr<ScalarFunction> sdf2;
  arr vel1, vel2;
  //query outputs
  arr x, g1, g2, z1, z2, y1, y2;
  double d1, d2, s;
  arr w1, w2;

  SweepingSDFPenetration(const FrameL& F) {
    CHECK_EQ(F.d0, 2, "");
    CHECK_EQ(F.d1, 2, "");
    sdf1 = F(0, 0)->shape->functional();
    sdf2 = F(0, 1)->shape->functional();
    vel1 = F(0, 0)->getPosition() - F(1, 0)->getPosition();
    vel2 = F(0, 1)->getPosition() - F(1, 1)->getPosition();

    ScalarFunction::operator=([this](arr& g, arr& H, const arr& x) -> double {
      return this->scalarFunction(g, H, x);
    });

  }

  double scalarFunction(arr& g, arr& H, const arr& x_s) {
    s = x_s.last();
    x = x_s({0, -2});
    CHECK_EQ(x.N, 3, "");
//    CHECK_GE(s, 0., "");
//    CHECK_LE(s, 1., "");

    w1 = x+vel1;
    w2 = x+vel2;

    z1 = x + s*vel1;
    z2 = x + s*vel2;
    arr dz1 = catCol(eye(3), vel1) ;
    arr dz2 = catCol(eye(3), vel2) ;

    y1 = x + (s-1.)*vel1;
    y2 = x + (s-1.)*vel2;

    arr H1, H2;
    double b = 10.;
    d1 = (*sdf1)(g1, H1, z1);
    d2 = (*sdf2)(g2, H2, z2);
    double dd = d1 - d2;

    arr tmp1 = -~g1;
    arr tmp2 = -~g2;
    H1 = ~dz1 * H1 * dz1 + rai::block(zeros(3, 3), ~tmp1, tmp1, zeros(1, 1));
    H2 = ~dz2 * H2 * dz2 + rai::block(zeros(3, 3), ~tmp2, tmp2, zeros(1, 1));
    g1 = ((~g1) * dz1).reshape(-1);
    g2 = ((~g2) * dz2).reshape(-1);

    if(!!H) H = H1 + H2 + (2.*b*dd)*(H1-H2) + (2.*b)*((g1-g2)^(g1-g2));
    if(!!g) g = g1 + g2 + (2.*b*dd)*(g1-g2);

    return d1+d2+b*dd*dd;

  }
};

//===========================================================================

void F_PairFunctional::phi2(arr& y, arr& J, const FrameL& F) {
  if(order==1) {
    P.reset();
    P = make_shared<SweepingSDFPenetration>(F);
    ScalarFunction f = *P;

    arr seed = .25*(F(0, 0)->getPosition() + F(0, 1)->getPosition() +
                    F(1, 0)->getPosition() + F(1, 1)->getPosition());

    seed.append(0.5);

//    checkGradient(f, seed, 1e-5);
//    checkHessian(f, seed, 1e-5);

    x = seed;
    OptNewton newton(x, f, rai::OptOptions()
                     .set_verbose(0)
                     .set_stopTolerance(1e-5)
                     .set_maxStep(1.)
                     .set_damping(1e-10));
    newton.setBounds(arr{{2,4}, {0., 0., 0., 0., -1., -1., -1., 1}});
    newton.run();

    d1 = P->d1;
    d2 = P->d2;
    x = P->x;
    g1 = P->g1.sub(0, 2);
    g2 = P->g2.sub(0, 2);
    double s = P->s;
//    LOG(0) <<"f(x):" <<newton.fx <<" d1:" <<d1 <<" d2:" <<d2 <<" s:" <<s <<" iters:" <<newton.its;
//    LOG(0) <<"x:" <<x ;

    //  if(!!J) LOG(0) <<"f(x):" <<newton.fx <<" d1:" <<d1 <<" d2:" <<d2 <<" iters:" <<newton.its;

    y.resize(1).scalar() = -d1 -d2;
    if(!!J) {
      arr Jp1a, Jp1b, Jp2a, Jp2b, Jp1c, Jp2c;
      F(0, 0)->C.jacobian_pos(Jp1a, F(0, 0), P->z1);
      F(0, 1)->C.jacobian_pos(Jp2a, F(0, 1), P->z2);
      F(0, 0)->C.jacobian_pos(Jp1c, F(0, 0), F(0, 0)->ensure_X().pos);
      F(0, 1)->C.jacobian_pos(Jp2c, F(0, 1), F(0, 1)->ensure_X().pos);
      F(1, 0)->C.jacobian_pos(Jp1b, F(1, 0), F(1, 0)->ensure_X().pos);
      F(1, 1)->C.jacobian_pos(Jp2b, F(1, 1), F(1, 1)->ensure_X().pos);
      J = ~g1*(Jp1a + s*(Jp1b-Jp1c)) + ~g2*(Jp2a + s*(Jp2b-Jp2c));
      checkNan(J);
    }

  } else {
    if(order>0) {  Feature::phi2(y, J, F);  return;  }
    CHECK_EQ(F.N, 2, "");
    rai::Frame* f1 = F.elem(0);
    rai::Frame* f2 = F.elem(1);
    CHECK(f1->shape && f2->shape, "");
    auto func1=f1->shape->functional();
    auto func2=f2->shape->functional();
    CHECK(func1 && func2, "");

    auto f = [&func1, &func2](arr& g, arr& H, const arr& x) {
      arr g1, g2, H1, H2;
      double b = 1e1;
      //double c = 1e2;
      double d1 = (*func1)(g1, H1, x);
      double d2 = (*func2)(g2, H2, x);
      double dd = d1 - d2;
      if(!!H) H = H1 + H2 + (2.*b*dd)*(H1-H2) + (2.*b)*((g1-g2)^(g1-g2));
      if(!!g) g = g1 + g2 + (2.*b*dd)*(g1-g2); // + (2*c)*(d1*H1+(g1^g1)+ d2*H2+(g2^g2))*(d1*g1+d2*g2);
      return d1 + d2 + b*dd*dd; // + c*sumOfSqr(d1*g1+d2*g2);
    };

    arr seed = .5*(f1->getPosition()+f2->getPosition());
    rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1), false);
    if(ex) seed = ex->poa;

//    checkGradient(f, seed, 1e-5);
//    checkHessian(f, seed, 1e-5);

    x = seed;
    OptNewton newton(x, f, rai::OptOptions()
                     .set_verbose(0)
                     .set_stopTolerance(1e-5)
                     .set_maxStep(1.)
                     .set_damping(1e-10));
    newton.run();

    d1 = (*func1)(g1, NoArr, x);
    d2 = (*func2)(g2, NoArr, x);

    //  if(!!J) LOG(0) <<"f(x):" <<newton.fx <<" d1:" <<d1 <<" d2:" <<d2 <<" (g1+g2):" <<sumOfSqr(g1+g2) <<" iters:" <<newton.its;

    rai::Proxy prox;
    prox.a = F.elem(0); prox.b = F.elem(1); prox.posA=x-d1*g1; prox.posB=x-d2*g2; prox.normal=g1-g2; prox.d = d1+d2;
    F.elem(0)->C.proxies.append(prox);

    y.resize(1).scalar() = -d1 -d2;
    if(!!J) {
      arr Jp1, Jp2;
      F.elem(0)->C.jacobian_pos(Jp1, F.elem(0), x);
      F.elem(1)->C.jacobian_pos(Jp2, F.elem(1), x);
      J = ~g1*Jp1 + ~g2*Jp2;
      checkNan(J);
    }
  }
}

