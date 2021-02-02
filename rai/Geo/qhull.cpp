/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_QHULL

#include "qhull.h"
#include "mesh.h"

extern "C" {
#ifdef RAI_MSVC
#  include <libqhull/qhull_a.h>
#else
#  include <qhull/qhull_a.h>
#endif
}
#undef dX
#undef dY
#undef dZ
#undef dW

int QHULL_DEBUG_LEVEL=0;

static Mutex qhullMutex;

//===========================================================================

const char* qhullVersion() {
  return qh_version;
}

//===========================================================================

void getQhullState(uint D, arr& points, arr& vertices, arr& lines) {
  uint i;
  double* point, *pointtemp;
  vertexT* vertex, **vertexp;
  facetT* facet;

//  plot()->Opengl();
//  plot()->Clear();

  cout <<"\n** points:";
  FORALLpoints {
    points.setCarray(point, D);
    cout <<"\n  " <<points;
//    plot()->Points(x);
  }

  cout <<"\n** vertices:";
  FORALLvertices {
    vertices.setCarray(vertex->point, D);
    i = (vertex->point - (qh first_point))/D;
    cout <<"\n  " <<vertex->id <<"(" <<i <<")" <<":" <<points;
  }

  cout <<"\n** facets:";
  arr x;
  FORALLfacets {
    cout <<"\n  " <<facet->id <<":";
    lines.clear();
    FOREACHvertex_(facet->vertices) {
      cout <<' ' <<vertex->id;
      x.setCarray(vertex->point, D);
      lines.append(x);
    }
    x.setCarray(((vertexT*)(facet->vertices->e[0].p))->point, D);
    lines.append(x);
    lines.reshape(lines.N/D, D);
//    plot()->Line(line);
  }
  cout <<endl;
}

//===========================================================================

void qhull_free() {
  qh_freeqhull(!qh_ALL);
  int curlong, totlong;
  qh_memfreeshort(&curlong, &totlong);
  if(curlong || totlong)
    RAI_MSG("qhull internal warning (main): did not free " <<totlong <<" bytes of long memory (" <<curlong <<" pieces)\n");
}

//===========================================================================

double distanceToConvexHull(const arr& X, const arr& y, arr& distances, arr& projectedPoints, uintA* faceVertices, bool freeqhull) {
  auto lock = qhullMutex(RAI_HERE);

  int exitcode;
  //static const char* cmd = "qhull Tv i p";
  static char* cmd = (char*) "qhull ";
  exitcode = qh_new_qhull(X.d1, X.d0, X.p, false, cmd, nullptr, stderr);
  if(exitcode) HALT("qh_new_qhull error - exitcode " <<exitcode);

  facetT* bestfacet;
  double bestdist;
  boolT isoutside;
  int totpart;

  arr Y;
  Y.referTo(y);
  if(y.nd==1) Y.reshape(1, Y.N);
  if(!!distances) distances.clear();
  if(!!projectedPoints) projectedPoints.clear();

  for(uint i=0; i<Y.d0; i++) {
    bestfacet = qh_findbest(Y[i].p, qh facet_list,
                            !qh_ALL, !qh_ISnewfacets, !qh_ALL,
                            &bestdist, &isoutside, &totpart);

    /*alternatives??
    //qh_findbestfacet(origin0, qh_ALL, &bestdist, &isoutside);

    //bestfacet= qh_findbest (origin0, qh facet_list,
    //  qh_ALL, !qh_ISnewfacets, qh_ALL , // qh_NOupper
    //        &bestdist, &isoutside, &totpart);
    */

    CHECK(length(y)>1e-10 || fabs(bestdist-bestfacet->offset)<1e-10, "inconsistent!");
    CHECK((isoutside && bestdist>-1e-10) || (!isoutside && bestdist<1e-10), "");

    if(!!distances) {
      distances.append(bestdist);
    }

    if(!!projectedPoints) {
      arr p = Y[i];
      arr n(bestfacet->normal, p.N, true);
      projectedPoints.append(p - bestdist*n);
      if(y.nd==2) projectedPoints.reshape(i+1, X.d1);
    }

    if(faceVertices) {
      faceVertices->clear();
      vertexT* vertex, **vertexp;
      FOREACHvertex_(bestfacet->vertices) {
        i = (vertex->point - (qh first_point))/X.d1;
        faceVertices->append(i);
      }
    }

//  if(QHULL_DEBUG_LEVEL>1) {
//    arr line;
//    NIY;
////    plotQhullState(X.d1);
////    plot()->Points(y);
//    if(projectedPoint) {
//      line.clear();
//      line.append(y);
//      line.append(*projectedPoint);
////      plot()->Points(*projectedPoint);
//      line.reshape(2, X.d1);
////      plot()->Line(line);
//    }
//    plot()->update();

//    //cout <<"**best facet: " <<bestfacet->id <<endl;
//    //FOREACHvertex_(facet->vertices) cout <<vertex->id <<' ';
//  }
  }

  if(freeqhull) qhull_free();

  return bestdist;
}

//===========================================================================

void makeNormal(arr& a, const arr& b) { a -= b * scalarProduct(a, b)/sumOfSqr(b); }

double distanceToConvexHullGradient(arr& dDdX, const arr& X, const arr& y, bool freeqhull) {
  arr p;
  uintA vertices;
  double d;

  d=distanceToConvexHull(X, y, NoArr, p, &vertices, freeqhull);

  dDdX.resizeAs(X);
  dDdX.setZero();

  uint i, j, k, l;
  arr v, f, w, v_f, y_f, dv, subn, wk, W;
  for(i=0; i<vertices.N; i++) {
    v.referToDim(X, vertices(i)); //v is the vertex in question

    // subn: normal of the sub-facet opposit to v
    if(i) j=0; else j=1;
    w.referToDim(X, vertices(j)); //take w as origin of local frame
    CHECK_GE(vertices.N, X.d1, ""); //won't work otherwise..
    W.resize(vertices.N, X.d1);      //compose matrix of basis vectors
    for(k=0, l=0; k<vertices.N; k++) if(k!=i && k!=j) {
        wk.referToDim(X, vertices(k));
        W[l]() = wk-w;
        l++;
      }
    CHECK_EQ(l, vertices.N-2, "");
    W[l]() = v-w;
    W[l+1]() = p-y; //not important (is already orthogonal to the full facet)
    rai::Array<double*> tmp;
    qh_gram_schmidt(X.d1, W.getCarray(tmp)); //orthogonalize local basis vectors
    subn = W[l]; //this entry should now be orthogonal to the sub-facet

    //f: axis point: projection of v along p onto the sub-facet (``Dreisatz'')
    double alpha = scalarProduct(w-v, subn)/scalarProduct(p-v, subn);
    f = v + alpha*(p-v);

    v_f = v-f;
    y_f = y-f;
    double yf_vf=scalarProduct(y_f, v_f);
    double yf_vf_norm=yf_vf/sumOfSqr(v_f);
    // check pythagoras
#ifndef RAI_NOCHECK
    double dd = sumOfSqr(y_f) - yf_vf * yf_vf_norm;
    CHECK(fabs(dd - d*d)<1e-8, "");
#endif

    //compute gradient
    dv.referToDim(dDdX, vertices(i));
    dv = f - y + yf_vf_norm*v_f;
    dv *= 2.*yf_vf_norm;
    dv *= .5/d;
  }

  return d;
}

//===========================================================================

double forceClosure(const arr& C, const arr& Cn, const rai::Vector& center,
                    double mu, double torqueWeights, arr* dFdC) { //, arr *dFdCn
  CHECK_EQ(C.d0, Cn.d0, "different number of points and normals");
  CHECK_EQ(C.d1, 3, "");

  uint i, j, S=7;
  rai::Vector c, n;

  arr X;
  if(torqueWeights>0.)  X.resize(C.d0*S, 6);  //store 6d points for convex wrench hull
  else X.resize(C.d0*S, 3);                //store 3d points for convex force hull

  arr dXdC;
  if(dFdC) {
    dXdC.resize(X.d0, X.d1, 3);
    dXdC.setZero();
  }
  /*if(dFdCn){
    dXdCn.resize(C.d0*S, 6, 3);
    dXdCn.setZero();
  }*/

  for(i=0; i<C.d0; i++) {  //each contact point contributes a friction cone
    c.set(&C(i, 0));                    //contact point
    n.set(&Cn(i, 0));                   //contact normal
    c -= center;

    rai::Quaternion r;
    r.setDiff(Vector_z, n);//rotate cone's z-axis into contact normal n

    for(j=0; j<S; j++) {   //each sample, equidistant on a circle
      double angle = j*RAI_2PI/S;
      rai::Vector f(cos(angle)*mu, sin(angle)*mu, 1.);  //force point sampled from cone

      f = r*f;                         //rotate
      rai::Vector c_f = c^f;

      //what about different scales in force vs torque??!!
      if(torqueWeights>=0.) { //forceClosure
        X(i*S+j, 0) = f.x;
        X(i*S+j, 1) = f.y;
        X(i*S+j, 2) = f.z;
      } else { //torqueClosure
        X(i*S+j, 0) = c_f.x;
        X(i*S+j, 1) = c_f.y;
        X(i*S+j, 2) = c_f.z;
      }
      if(torqueWeights>0.) { //both (wrench)
        X(i*S+j, 3) = torqueWeights * c_f.x;
        X(i*S+j, 4) = torqueWeights * c_f.y;
        X(i*S+j, 5) = torqueWeights * c_f.z;
      }
      if(dFdC) {
        dXdC(i*S+j, 3, 0) =  0   ;  dXdC(i*S+j, 3, 1) =  f.z;  dXdC(i*S+j, 3, 2) = -f.y;
        dXdC(i*S+j, 4, 0) = -f.z;  dXdC(i*S+j, 4, 1) =  0   ;  dXdC(i*S+j, 4, 2) =  f.x;
        dXdC(i*S+j, 5, 0) =  f.y;  dXdC(i*S+j, 5, 1) = -f.x;  dXdC(i*S+j, 5, 2) =  0   ;
      }
      /*if(dFdCn){
      HALT("");
      dXdCn(i*S+j, 3, 0) =  0   ; dXdCn(i*S+j, 3, 1) = -c(2); dXdCn(i*S+j, 3, 2) =  c(1);
      dXdCn(i*S+j, 4, 0) =  c(2); dXdCn(i*S+j, 4, 1) =  0   ; dXdCn(i*S+j, 4, 2) = -c(0);
      dXdCn(i*S+j, 5, 0) = -c(1); dXdCn(i*S+j, 5, 1) =  c(0); dXdCn(i*S+j, 5, 2) =  0   ;
      }*/
    }
  }

  if(dFdC)  dXdC *= (double)torqueWeights;

  double d;
  arr origin(X.d1);
  origin.setZero();
  if(!dFdC) {
    //note: distance to hull is negative if inside the hull
    d = -distanceToConvexHull(X, origin, NoArr, NoArr, nullptr, true);
  } else {
    arr dFdX;
    d = -distanceToConvexHullGradient(dFdX, X, origin, true);
    dFdX *= -1.;
    dFdX.reshape(TUP(C.d0, S, origin.N));
    dXdC.reshape(TUP(C.d0, S, origin.N, 3));
    dFdC->resize(TUP(C.d0, 3));
    tensorEquation(*dFdC, dFdX, TUP(0u, 2u, 3u), dXdC, TUP(0u, 2u, 3u, 1u), 2);
  }
  return d;
}

//===========================================================================

arr getHull(const arr& V, uintA& T) {
  auto lock = qhullMutex(RAI_HERE);

  int exitcode;
  uint dim=V.d1;
  static char* cmd = (char*) "qhull Qt ";
  exitcode = qh_new_qhull(V.d1, V.d0, V.p, false, cmd, nullptr, stderr);
  if(exitcode) HALT("qh_new_qhull error - exitcode " <<exitcode);

  qh_triangulate();

  facetT* facet;
  vertexT* vertex, **vertexp;
  uint f, i, v;

  arr Vnew;
  Vnew.resize(qh num_vertices, dim);
  i=0;
  FORALLvertices {
    vertex->id = i;
    memmove(&Vnew(i, 0), vertex->point,  dim*sizeof(double));
    i++;
  }
  if(!!T) { //retrieve also the triangulation
    T.resize(qh num_facets, dim);
    f=0;
    FORALLfacets {
      i=0;
      FOREACHvertex_(facet->vertices) {
        if(i<3) T(f, i)=vertex->id; else RAI_MSG("face " <<f <<" has " <<i <<" vertices" <<endl);
        i++;
      }
      if(facet->toporient) {
        v=T(f, 0);  T(f, 0)=T(f, 1);  T(f, 1)=v;
      }
      f++;
    }
    CHECK_EQ(f, T.d0, "");
  }

  qh_freeqhull(!qh_ALL);
  int curlong, totlong;
  qh_memfreeshort(&curlong, &totlong);
  if(curlong || totlong)
    RAI_MSG("qhull internal warning (main): did not free " <<totlong <<" bytes of long memory (" <<curlong <<" pieces)\n");

  return Vnew;
}

//===========================================================================

void getDelaunayEdges(uintA& E, const arr& V) {
  auto lock = qhullMutex(RAI_HERE);

  if(V.d0<3) { E.clear(); return; }
  int exitcode;
  static char* cmd = (char*) "qhull d Qbb Qt ";
  exitcode = qh_new_qhull(V.d1, V.d0, V.p, false, cmd, nullptr, stderr);
  if(exitcode) HALT("qh_new_qhull error - exitcode " <<exitcode);

  facetT* facet;
  vertexT* vertex, **vertexp;
  uint i, j, k, dim=V.d1;

  E.clear();
  std::vector<uint> face(dim+1);
  FORALLfacets {
    if(!facet->upperdelaunay) {
      i=0;
      FOREACHvertex_(facet->vertices) face[i++]=qh_pointid(vertex->point);//vertex->id;
      CHECK_EQ(i, dim+1, "strange number of vertices of a facet!");
      for(j=0; j<dim+1; j++) for(k=j+1; k<dim+1; k++) {
          E.append(TUP(face[j], face[k]));
        }
    }
  }
  E.reshape(E.N/2, 2);

  qh_freeqhull(!qh_ALL);
  int curlong, totlong;
  qh_memfreeshort(&curlong, &totlong);
  if(curlong || totlong)
    RAI_MSG("qhull internal warning (main): did not free " <<totlong <<" bytes of long memory (" <<curlong <<" pieces)\n");
}

//===========================================================================

#ifdef OLD_CODE
#include "graph.h"

/** this calls the delaunay triangulation of the qhull library

    It first deletes all existing edges! Then adds edges according to
    the delaunay triangulation.

    PRECONDITION: It is assumed that the node type #N# can be casted
    into an #doubleA&# with proper dimensionality
    (#dynamic_cast<doubleA& >(N& n)# has to be defined); if
    the node does not have this member, the code won't compile... */
template<class N, class E>
void delaunay(Graph<N, E>& g, uint dim=2) {
  uint i;

  g.clear_edges();

  doubleA P;
  P.resize(g.N, dim);
  for(i=0; i<g.N; i++) {
    CHECK_EQ(g.nodes(i)->point.N, dim, "point doesn't have expected dim in delaunay");
    P[i]=(doubleA&)(*(g.nodes(i)));
    //P(i, 0)=g.nodes(i)->feat.x;
    //P(i, 1)=g.nodes(i)->feat.y;
    //P(i, 2)=g.nodes(i)->feat.z;
  }

  if(!qh_new_qhull(dim, g.N, P.p, false, "qhull d Qbb T0", nullptr, stderr)) {
    facetT* facet;
    vertexT* vertex, **vertexp;
    uint* face, k, l;
    face=new uint[dim+1];

    FORALLfacets {
      if(!facet->upperdelaunay) {
        uint j=0;
        FOREACHvertex_(facet->vertices) face[j++]=qh_pointid(vertex->point);
        CHECK_EQ(j, dim+1, "strange number of vertices of a facet!");
        for(k=0; k<dim+1; k++) for(l=0; l<dim+1; l++) if(k!=l)
              if(!g.getEdge(g.nodes(face[k]), g.nodes(face[l])))
                g.new_edge(g.nodes(face[k]), g.nodes(face[l]));
        i++;
      }
    }

    delete[] face;
  }

  int curlong, totlong;
  qh_freeqhull(!qh_ALL);                 //free long memory
  qh_memfreeshort(&curlong, &totlong);   //free short memory and memory allocator

  if(curlong || totlong)
    RAI_MSG("qhull did not free " <<totlong <<" bytes of long memory (" <<curlong <<" pieces)");
}

#endif

#else ///RAI_QHULL
#include "../Core/util.h"
#include "../Core/array.h"
#include "geo.h"
int QHULL_DEBUG_LEVEL=0;
const char* qhullVersion() { return "NONE"; }
void getTriangulatedHull(uintA& T, arr& V) { NICO }
double forceClosure(const arr& C, const arr& Cn, const rai::Vector& center,
                    double mu, double torqueWeights, arr* dFdC) { NICO }
double distanceToConvexHull(const arr& X, const arr& y, arr* projectedPoint, uintA* faceVertices, bool freeqhull) { NICO }
double distanceToConvexHullGradient(arr& dDdX, const arr& X, const arr& y, bool freeqhull) { NICO }
void getDelaunayEdges(uintA& E, const arr& V) { NICO }
#endif

typedef struct { double x, y; } vec_t;
typedef vec_t* vec;

inline double dot(vec a, vec b) {
  return a->x * b->x + a->y * b->y;
}

inline double cross(vec a, vec b) {
  return a->x * b->y - a->y * b->x;
}

inline vec vsub(vec a, vec b, vec res) {
  res->x = a->x - b->x;
  res->y = a->y - b->y;
  return res;
}

/* tells if vec c lies on the left side of directed edge a->b
 * 1 if left, -1 if right, 0 if colinear
 */
int left_of(vec a, vec b, vec c) {
  vec_t tmp1, tmp2;
  double x;
  vsub(b, a, &tmp1);
  vsub(c, b, &tmp2);
  x = cross(&tmp1, &tmp2);
  return x < 0 ? -1 : x > 0;
}

int line_sect(vec x0, vec x1, vec y0, vec y1, vec res) {
  vec_t dx, dy, d;
  vsub(x1, x0, &dx);
  vsub(y1, y0, &dy);
  vsub(x0, y0, &d);
  /* x0 + a dx = y0 + b dy ->
       x0 X dx = y0 X dx + b dy X dx ->
       b = (x0 - y0) X dx / (dy X dx) */
  double dyx = cross(&dy, &dx);
  if(!dyx) return 0;
  dyx = cross(&d, &dx) / dyx;
  if(dyx <= 0 || dyx >= 1) return 0;

  res->x = y0->x + dyx * dy.x;
  res->y = y0->y + dyx * dy.y;
  return 1;
}

/* === polygon stuff === */
typedef struct { int len, alloc; vec v; } poly_t;
typedef poly_t* poly;

poly poly_new() {
  return (poly)calloc(1, sizeof(poly_t));
}

void poly_free(poly p) {
  free(p->v);
  free(p);
}

void poly_append(poly p, vec v) {
  if(p->len >= p->alloc) {
    p->alloc *= 2;
    if(!p->alloc) p->alloc = 4;
    p->v = (vec)realloc(p->v, sizeof(vec_t) * p->alloc);
  }
  p->v[p->len++] = *v;
}

/* this works only if all of the following are true:
 *   1. poly has no colinear edges;
 *   2. poly has no duplicate vertices;
 *   3. poly has at least three vertices;
 *   4. poly is convex (implying 3).
*/
int poly_winding(poly p) {
  return left_of(p->v, p->v + 1, p->v + 2);
}

void poly_edge_clip(poly sub, vec x0, vec x1, int left, poly res) {
  int i, side0, side1;
  vec_t tmp;
  vec v0 = sub->v + sub->len - 1, v1;
  res->len = 0;

  side0 = left_of(x0, x1, v0);
  if(side0 != -left) poly_append(res, v0);

  for(i = 0; i < sub->len; i++) {
    v1 = sub->v + i;
    side1 = left_of(x0, x1, v1);
    if(side0 + side1 == 0 && side0)
      /* last point and current straddle the edge */
      if(line_sect(x0, x1, v0, v1, &tmp))
        poly_append(res, &tmp);
    if(i == sub->len - 1) break;
    if(side1 != -left) poly_append(res, v1);
    v0 = v1;
    side0 = side1;
  }
}

poly poly_clip(poly sub, poly clip) {
  int i;
  poly p1 = poly_new(), p2 = poly_new(), tmp;

  int dir = poly_winding(clip);
  poly_edge_clip(sub, clip->v + clip->len - 1, clip->v, dir, p2);
  for(i = 0; i < clip->len - 1; i++) {
    tmp = p2; p2 = p1; p1 = tmp;
    if(p1->len == 0) {
      p2->len = 0;
      break;
    }
    poly_edge_clip(p1, clip->v + i, clip->v + i + 1, dir, p2);
  }

  poly_free(p1);
  return p2;
}

void sort2Dpoints(arr& A) {
  arr m = mean(A);
  arr d(A.d0);
  for(uint i=0; i<A.d0; i++) {
    arr a = A[i]-m;
    d(i) = atan2(a(1), a(0));
  }
  uintA perm;
  perm.setStraightPerm(A.d0);
  perm.sort([&d](const uint&i, const uint&j) { return d(i)<d(j); });
  A.permuteRows(perm);
}

arr convconv_intersect(const arr& A, const arr& B) {
  if(A.d0==1) return A;
  if(B.d0==1) return B;
  if(A.d0==2) return A;
  if(B.d0==2) return B;

  arr AA = getHull(A); //rndGauss(AA, 1e-4, true);
  arr BB = getHull(B); //rndGauss(BB, 1e-4, true);
  sort2Dpoints(AA);
  sort2Dpoints(BB);

  poly_t Pa = {(int)AA.d0, 0, (vec_t*)AA.p};
  poly_t Pb = {(int)BB.d0, 0, (vec_t*)BB.p};

  poly res;
  if(AA.d0<BB.d0)
    res = poly_clip(&Pa, &Pb);
  else
    res = poly_clip(&Pb, &Pa);

  arr C;
  C.setCarray((double*)res->v, 2*res->len);
  C.reshape(C.N/2, 2);

//  plot()->Clear();
//  plot()->Line(C+.002, true); cout <<"#C=" <<C.d0 <<endl;
//  plot()->Line(C-.002, true);
//  plot()->Line(AA, true);
//  plot()->Line(BB, true);
//  plot()->Opengl();
//  cout <<"\n====\n" <<AA <<"\n----\n" <<B <<"\n----\n" <<C<<"\n====\n" <<endl;
//  rai::wait();

  poly_free(res);

  return C;
}

void pullPointsIntoHull(arr& P, const arr& X) {
  arr pulled, D;
  distanceToConvexHull(X, P, D, pulled);
  CHECK_EQ(D.N, P.d0, "");
  CHECK_EQ(D.N, pulled.d0, "");
  for(uint i=0; i<D.N; i++) if(D(i)>0.) P[i]()=pulled[i];
}
