/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TM_GJK.h"
#include "frame.h"

#define RAI_CCD
#ifdef RAI_CCD

#include "../Geo/ccd/ccd.h"
#include "../Geo/ccd/quat.h" // for work with quaternions

void support_mesh(const void* _obj, const ccd_vec3_t* _dir, ccd_vec3_t* v);

double GJK_libccd_penetration(arr& dir, arr& pos, const rai::Mesh& m1, const rai::Mesh& m2) {
  ccd_t ccd;
  CCD_INIT(&ccd); // initialize ccd_t struct

  // set up ccd_t struct
  ccd.support1       = support_mesh; // support function for first object
  ccd.support2       = support_mesh; // support function for second object
  ccd.max_iterations = 100;     // maximal number of iterations
  ccd.epa_tolerance  = 0.0001;  // maximal tolerance fro EPA part

  ccd_real_t depth;
  ccd_vec3_t _dir, _pos;

  //    S.m1.translate(randn(3));
  //    int intersect = ccdGJKIntersect(&S.m1, &S.m2, &ccd, &v1, &v2);
  int non_intersect = ccdGJKPenetration(&m1, &m2, &ccd, &depth, &_dir, &_pos);
  //    int intersect = ccdMPRIntersect(&S.m1, &S.m2, &ccd);
//  int non_intersect = ccdMPRPenetration(&m1, &m2, &ccd, &depth, &_dir, &_pos);
  if(non_intersect) { dir.clear(); pos.clear(); return 0.; }

  dir.setCarray(_dir.v, 3);
  pos.setCarray(_pos.v, 3);
  return depth;
}

#endif

TM_GJK::TM_GJK(const rai::Frame* s1, const rai::Frame* s2, bool exact, bool negative) : exact(exact), negScalar(negative) {
  CHECK(s1 && s2, "");
  CHECK(s1->shape && s2->shape, "");
  i = s1->ID;
  j = s2->ID;
}

TM_GJK::TM_GJK(const rai::Configuration& W, const char* s1, const char* s2, bool exact, bool negative) : exact(exact), negScalar(negative) {
  CHECK(s1 && s2, "");
  rai::Frame* s;
  s=W.getFrameByName(s1); CHECK(s, "shape name '" <<s1 <<"' does not exist"); i=s->ID;
  s=W.getFrameByName(s2); CHECK(s, "shape name '" <<s2 <<"' does not exist"); j=s->ID;
}

TM_GJK::TM_GJK(const rai::Configuration& W, const Graph& specs, bool exact) : exact(exact) {
  Node* it;
  if((it=specs["sym2"])) { auto name=it->get<rai::String>(); auto* s=W.getFrameByName(name); CHECK(s, "shape name '" <<name <<"' does not exist"); i=s->ID; }
  if((it=specs["sym3"])) { auto name=it->get<rai::String>(); auto* s=W.getFrameByName(name); CHECK(s, "shape name '" <<name <<"' does not exist"); j=s->ID; }
//  if((it=specs["vec1"])) vec1 = rai::Vector(it->get<arr>());  else vec1.setZero();
//  if((it=specs["vec2"])) vec2 = rai::Vector(it->get<arr>());  else vec2.setZero();
}

void TM_GJK::phi(arr& v, arr& J, const rai::Configuration& W) {
  rai::Shape* s1 = i<0?nullptr: W.frames(i)->shape;
  rai::Shape* s2 = j<0?nullptr: W.frames(j)->shape;
  CHECK(s1 && s2, "");
  CHECK(s1->type()==rai::ST_mesh || s1->type()==rai::ST_ssCvx || s1->type()==rai::ST_ssBox, "");
  CHECK(s2->type()==rai::ST_mesh || s2->type()==rai::ST_ssCvx || s2->type()==rai::ST_ssBox, "");
  const rai::Mesh* m1, *m2;
  if(s1->type()==rai::ST_mesh) m1=&s1->mesh(); else m1=&s1->sscCore();
  if(s2->type()==rai::ST_mesh) m2=&s2->mesh(); else m2=&s2->sscCore();
  CHECK(m1->V.N, "");
  CHECK(m1->V.N, "");
  rai::Vector p1, p2, e1, e2;
  GJK_point_type pt1, pt2;

  double d2 = GJK_sqrDistance(*m1, *m2, s1->frame.ensure_X(), s2->frame.ensure_X(), p1, p2, e1, e2, pt1, pt2);
  //  if(d2<1e-10) LOG(-1) <<"zero distance";
  arr y1, J1, y2, J2;

  double penetrating = false;
  if(d2<1e-10) {
    penetrating = true;
    arr dir, pos;
    //THIS IS PRETTY SLOW... (make the support function depend on transform?)
    rai::Mesh M1(*m1); s1->frame.ensure_X().applyOnPointArray(M1.V);
    rai::Mesh M2(*m2); s2->frame.ensure_X().applyOnPointArray(M2.V);
    double penetration = GJK_libccd_penetration(dir, pos, M1, M2);
//    cout <<"penetration=" <<penetration <<endl;
    rai::Vector cen = pos; //.5*(p1+p2);
    p1 = cen + .5*penetration*rai::Vector(dir);
    p2 = cen - .5*penetration*rai::Vector(dir);
    pt1=GJK_vertex; pt2=GJK_face;
  }

  W.kinematicsPos(y1, (!!J?J1:NoArr), &s1->frame, s1->frame.ensure_X().rot/(p1-s1->frame.ensure_X().pos));
  W.kinematicsPos(y2, (!!J?J2:NoArr), &s2->frame, s2->frame.ensure_X().rot/(p2-s2->frame.ensure_X().pos));
  v = y1 - y2;
  if(!!J) {
    J = J1 - J2;
    if(exact) {
      if((pt1==GJK_vertex && pt2==GJK_face) || (pt1==GJK_face && pt2==GJK_vertex)) {
        arr vec, Jv, n = v/length(v);
        J = n*(~n*J);
        if(pt1==GJK_vertex) W.kinematicsVec(vec, Jv, &s2->frame, s2->frame.ensure_X().rot/(p1-p2));
        if(pt2==GJK_vertex) W.kinematicsVec(vec, Jv, &s1->frame, s1->frame.ensure_X().rot/(p1-p2));
        J += Jv;
      }
      if(pt1==GJK_edge && pt2==GJK_edge) {
        arr vec, Jv, n, a, b;
        n = v/length(v);
        J = n*(~n*J);

        W.kinematicsVec(vec, Jv, &s1->frame, s1->frame.ensure_X().rot/e1);
        a=conv_vec2arr(e1);
        b=conv_vec2arr(e2);
        double ab=scalarProduct(a, b);
        J += (a-b*ab) * (1./(1.-ab*ab)) * (~v*(b*~b -eye(3, 3))) * Jv;

        W.kinematicsVec(vec, Jv, &s2->frame, s2->frame.ensure_X().rot/e2);
        a=conv_vec2arr(e2);
        b=conv_vec2arr(e1);
        J += (a-b*ab) * (1./(1.-ab*ab)) * (~v*(b*~b -eye(3, 3))) * Jv;
      }
      if((pt1==GJK_vertex && pt2==GJK_edge) || (pt1==GJK_edge && pt2==GJK_vertex)) {
        arr vec, Jv, n;
        if(pt1==GJK_vertex) n=conv_vec2arr(e2); else n=conv_vec2arr(e1);
        J = J - n*(~n*J);
        if(pt1==GJK_vertex) W.kinematicsVec(vec, Jv, &s2->frame, s2->frame.ensure_X().rot/(p1-p2));
        if(pt2==GJK_vertex) W.kinematicsVec(vec, Jv, &s1->frame, s1->frame.ensure_X().rot/(p1-p2));
        J += n*(~n*Jv);
      }
    }
  }

  //reduce by radii
  double rad=0.;
  if(s1->type()==rai::ST_ssCvx) rad += s1->size(-1);
  if(s2->type()==rai::ST_ssCvx) rad += s2->size(-1);
  double l2=sumOfSqr(v), l=sqrt(l2);
  double fac = (l-rad)/l;
  if(!!J) {
    arr d_fac = (1.-(l-rad)/l)/l2 *(~v)*J;
    J = J*fac + v*d_fac;
  }
  v *= fac;

  if(negScalar) {
    if(penetrating) {
      if(!!J) J = ~(v/l)*J;
      v = ARR(l);
    } else {
      if(!!J) J = ~(v/(-l))*J;
      v = ARR(-l);
    }
  }
//  CHECK_ZERO(l2-d2, 1e-6,"");
}

rai::String TM_GJK::shortTag(const rai::Configuration& G) {
  return STRING("TM_GJK"<<(i<0?"WORLD":G.frames(i)->name) <<':' <<(j<0?"WORLD":G.frames(j)->name));
}

