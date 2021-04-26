/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
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

void F_PairCollision::phi2(arr& y, arr& J, const FrameL& F) {
  if(order>0){  Feature::phi2(y, J, F);  return;  }
  CHECK_EQ(F.N, 2, "");
  rai::Frame* f1 = F.elem(0);
  rai::Frame* f2 = F.elem(1);
  CHECK(f1->shape && f2->shape, "");
  double r1=f1->shape->radius();
  double r2=f2->shape->radius();
  rai::Mesh* m1 = &f1->shape->sscCore();  if(!m1->V.N) { m1 = &f1->shape->mesh(); r1=0.; }
  rai::Mesh* m2 = &f2->shape->sscCore();  if(!m2->V.N) { m2 = &f2->shape->mesh(); r2=0.; }
  if(!m1->V.N) m1->V = zeros(1, 3);
  if(!m2->V.N) m2->V = zeros(1, 3);

  coll.reset();
#if 0 //use functionals!
  auto func1=f1->shape->functional();
  auto func2=f2->shape->functional();
  if(func1 && func2){
    coll=make_shared<PairCollision>(*func1, *func2, .5*(f1->getPosition()+f2->getPosition()));
  }else{
    coll=make_shared<PairCollision>(*m1, *m2, f1->ensure_X(), f2->ensure_X(), r1, r2);
  }
#else
  coll=make_shared<PairCollision>(*m1, *m2, f1->ensure_X(), f2->ensure_X(), r1, r2);
#endif

  if(neglectRadii) coll->rad1=coll->rad2=0.;

  if(type==_negScalar) {
    arr Jp1, Jp2;
    f1->C.jacobian_pos(Jp1, f1, coll->p1);
    f2->C.jacobian_pos(Jp2, f2, coll->p2);
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
    if(p.a->ID>=F.first()->ID && p.a->ID<=F.last()->ID) { //F.contains(p.a) && F.contains(p.b)) {
      CHECK(p.a->shape, "");
      CHECK(p.b->shape, "");

      //early check: if swift is way out of collision, don't bother computing it precise
      if(p.d > p.a->shape->radius() + p.b->shape->radius() + .01 + margin) continue;

      if(!p.collision) p.calc_coll();

      if(p.collision->getDistance()>margin) continue;

      arr Jp1, Jp2;
      p.a->C.jacobian_pos(Jp1, p.a, p.collision->p1);
      p.b->C.jacobian_pos(Jp2, p.b, p.collision->p2);

      arr y_dist, J_dist;
      p.collision->kinDistance(y_dist, J_dist, Jp1, Jp2);

      if(y_dist.scalar()>margin) continue;
      y += margin-y_dist.scalar();
      J -= J_dist;
    }
  }
}

//===========================================================================

namespace rai{
  template<class T>
arr block(const Array<T>& A, const Array<T>& B, const Array<T>& C, const Array<T>& D){
  arr X;
  X.setBlockMatrix(A, B, C, D);
  return X;
}
}

struct SweepingSDFPenetration : ScalarFunction {
  //inputs
  shared_ptr<ScalarFunction> sdf1;
  shared_ptr<ScalarFunction> sdf2;
  arr R1, t1, R2, t2;
  //query outputs
  arr x, g1, g2;
  double d1, d2, s;
  SweepingSDFPenetration(const FrameL& F){
    CHECK_EQ(F.d0, 2, "");
    CHECK_EQ(F.d1, 2, "");
    sdf1 = F(0,0)->shape->functional();
    sdf2 = F(0,1)->shape->functional();
    rai::Transformation rel1 = F(0,0)->ensure_X() * -F(1,0)->ensure_X();
    //rel1=-rel1;
    R1 = rel1.rot.getArr();
    t1 = rel1.pos.getArr();
    rai::Transformation rel2 = F(0,1)->ensure_X() * -F(1,1)->ensure_X();
    //rel2=-rel2;
    R2 = rel2.rot.getArr();
    t2 = rel2.pos.getArr();

    ScalarFunction::operator=([this](arr& g, arr& H, const arr& x) -> double {
      return this->scalarFunction(g, H, x);
    });

  }

  double scalarFunction(arr& g, arr& H, const arr& x_s){
    s = x_s.last();
    x = x_s({0,-2});
    CHECK_EQ(x.N, 3, "");
//    CHECK_GE(s, 0., "");
//    CHECK_LE(s, 1., "");

    arr z1 = (1.-s)*x + s*(R1*x+t1);
    arr z2 = (1.-s)*x + s*(R2*x+t2);
    arr dz1 = catCol((1.-s)*eye(3) + s*R1, -x + (R1*x+t1)) ;
    arr dz2 = catCol((1.-s)*eye(3) + s*R2, -x + (R2*x+t2)) ;

    arr H1, H2;
    double b = 10.;
    d1 = (*sdf1)(g1, H1, z1);
    d2 = (*sdf2)(g2, H2, z2);
    double dd = d1 - d2;

    arr tmp1 = (~g1)*(-eye(3) + R1);
    arr tmp2 = (~g2)*(-eye(3) + R2);
    H1 = ~dz1 * H1 * dz1 + rai::block(zeros(3,3), ~tmp1, tmp1, zeros(1,1));
    H2 = ~dz2 * H2 * dz2 + rai::block(zeros(3,3), ~tmp2, tmp2, zeros(1,1));
    g1 = ((~g1) * dz1).reshape(-1);
    g2 = ((~g2) * dz2).reshape(-1);

    if(!!H) H = H1 + H2 + (2.*b*dd)*(H1-H2) + (2.*b)*((g1-g2)^(g1-g2));
    if(!!g) g = g1 + g2 + (2.*b*dd)*(g1-g2);

    return d1+d2+b*dd*dd;

  }

};

//===========================================================================

void F_PairFunctional::phi2(arr& y, arr& J, const FrameL& F){
  ScalarFunction f;
  arr seed;
  shared_ptr<ScalarFunction>  func1, func2;
  shared_ptr<SweepingSDFPenetration> P;

  if(order==1){
    P = make_shared<SweepingSDFPenetration>(F);
    f = *P;

    seed = .25*(F(0,0)->getPosition() + F(0,1)->getPosition() +
                F(1,0)->getPosition() + F(1,1)->getPosition() );

    seed.append(0.5);

  }else{
    if(order>0){  Feature::phi2(y, J, F);  return;  }
    CHECK_EQ(F.N, 2, "");
    rai::Frame* f1 = F.elem(0);
    rai::Frame* f2 = F.elem(1);
    CHECK(f1->shape && f2->shape, "");
    auto func1=f1->shape->functional();
    auto func2=f2->shape->functional();
    CHECK(func1 && func2, "");

    f = [&func1, &func2](arr& g, arr& H, const arr& x){
      arr g1, g2, H1, H2;
      double b = 10.;
      double d1 = (*func1)(g1, H1, x);
      double d2 = (*func2)(g2, H2, x);
      double dd = d1 - d2;
      H = H1 + H2 + (2.*b*dd)*(H1-H2) + (2.*b)*((g1-g2)^(g1-g2));
      g = g1 + g2 + (2.*b*dd)*(g1-g2);
      return d1+d2+b*dd*dd;
    };

    seed = .5*(f1->getPosition()+f2->getPosition());
    rai::ForceExchange* ex = getContact(F.elem(0), F.elem(1), false);
    if(ex) seed = ex->poa;
  }

  checkGradient(f, seed, 1e-5);
  checkHessian(f, seed, 1e-5);

  x = seed;
  OptNewton newton(x, f, OptOptions()
                   .set_verbose(0)
                   .set_stopTolerance(1e-5)
                   .set_maxStep(1.)
                   .set_damping(1e-10) );
  if(order==1) newton.setBounds({0.,0.,0.,0.},{0.,0.,0.,1});
  newton.run();

  if(order==1){
    d1 = P->d1;
    d2 = P->d2;
    x = P->x;
    g1 = P->g1.sub(0,2);
    g2 = P->g2.sub(0,2);
    LOG(0) <<"f(x):" <<newton.fx <<" d1:" <<d1 <<" d2:" <<d2 <<" iters:" <<newton.its;
    LOG(0) <<"x:" <<x <<" s:" <<P->s;
  }else{
    d1 = (*func1)(g1, NoArr, x);
    d2 = (*func2)(g2, NoArr, x);
  }

//  if(!!J) LOG(0) <<"f(x):" <<newton.fx <<" d1:" <<d1 <<" d2:" <<d2 <<" iters:" <<newton.its;

  y.resize(1).scalar() = -d1 -d2;
  if(!!J) {
    arr Jp1, Jp2;
    F(0,0)->C.jacobian_pos(Jp1, F(0,0), x);
    F(0,1)->C.jacobian_pos(Jp2, F(0,1), x);
    J = ~g1*Jp1 + ~g2*Jp2;
    checkNan(J);
  }

}


void F_PairFunctional::glDraw(OpenGL&) {
#ifdef RAI_GL
  glColor(0., 1., 0., 1.);
  glDrawDiamond(x(0), x(1), x(2), .05, .05, .05);

  glColor(1., 0., 0., 1.);
  glLineWidth(2.f);
  glDrawProxy(x-d1*g1, x, .02);
  glDrawProxy(x, x-d2*g2, .02);
  glLineWidth(1.f);
  glLoadIdentity();
#endif
}

