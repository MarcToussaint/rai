/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

/**
 * @file
 * @ingroup group_ors
 */
/**
 * @ingroup group_ors
 * @{
 */


#ifdef MLR_QHULL

#include "mesh.h"

extern "C" {
  #include <qhull/qhull_a.h>
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
  double *point, *pointtemp;
  vertexT *vertex, **vertexp;
  facetT *facet;
  
//  plotOpengl();
//  plotClear();
  
  cout <<"\n** points:";
  FORALLpoints {
    points.setCarray(point, D);
    cout <<"\n  " <<points;
//    plotPoints(x);
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
//    plotLine(line);
  }
  cout <<endl;
}

//===========================================================================

double distanceToConvexHull(const arr &X, const arr &y, arr *projectedPoint, uintA *faceVertices, bool freeqhull) {
  auto lock = qhullMutex();

  int exitcode;
  //static const char* cmd = "qhull Tv i p";
  static char* cmd = (char*) "qhull ";
  exitcode = qh_new_qhull(X.d1, X.d0, X.p, false, cmd, NULL, stderr);
  if(exitcode) HALT("qh_new_qhull error - exitcode " <<exitcode);
  
  uint i;
  facetT *bestfacet;
  double bestdist;
  boolT isoutside;
  int totpart;
  
  bestfacet = qh_findbest(y.p, qh facet_list,
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
  
  if(projectedPoint) {
    *projectedPoint=y;
    double *normal=bestfacet->normal;
    for(i=X.d1; i--;)(*projectedPoint)(i) -= bestdist * normal[i];
  }
  
  if(faceVertices) {
    faceVertices->clear();
    vertexT *vertex, **vertexp;
    FOREACHvertex_(bestfacet->vertices) {
      i = (vertex->point - (qh first_point))/X.d1;
      faceVertices->append(i);
    }
  }
  
//  if(QHULL_DEBUG_LEVEL>1) {
//    arr line;
//    NIY;
////    plotQhullState(X.d1);
////    plotPoints(y);
//    if(projectedPoint) {
//      line.clear();
//      line.append(y);
//      line.append(*projectedPoint);
////      plotPoints(*projectedPoint);
//      line.reshape(2, X.d1);
////      plotLine(line);
//    }
//    plot();
    
//    //cout <<"**best facet: " <<bestfacet->id <<endl;
//    //FOREACHvertex_(facet->vertices) cout <<vertex->id <<' ';
//  }
  
  if(freeqhull) {
    qh_freeqhull(!qh_ALL);
    int curlong, totlong;
    qh_memfreeshort(&curlong, &totlong);
    if(curlong || totlong)
      MLR_MSG("qhull internal warning (main): did not free " <<totlong <<" bytes of long memory (" <<curlong <<" pieces)\n");
  }
  
  return bestdist;
}

//===========================================================================

void makeNormal(arr& a, const arr& b) { a -= b * scalarProduct(a, b)/sumOfSqr(b); }

double distanceToConvexHullGradient(arr& dDdX, const arr &X, const arr &y, bool freeqhull) {
  arr p;
  uintA vertices;
  double d;
  
  d=distanceToConvexHull(X, y, &p, &vertices, freeqhull);
  
  dDdX.resizeAs(X);
  dDdX.setZero();
  
  uint i, j, k, l;
  arr v, f, w, v_f, y_f, dv, subn, wk, W;
  double dd;
  for(i=0; i<vertices.N; i++) {
    v.referToDim(X, vertices(i)); //v is the vertex in question
    
    // subn: normal of the sub-facet opposit to v
    if(i) j=0; else j=1;
    w.referToDim(X, vertices(j)); //take w as origin of local frame
    CHECK(vertices.N>=X.d1, ""); //won't work otherwise..
    W.resize(vertices.N, X.d1);      //compose matrix of basis vectors
    for(k=0, l=0; k<vertices.N; k++) if(k!=i && k!=j) {
        wk.referToDim(X, vertices(k));
        W[l]() = wk-w;
        l++;
      }
    CHECK_EQ(l,vertices.N-2, "");
    W[l]() = v-w;
    W[l+1]() = p-y; //not important (is already orthogonal to the full facet)
    mlr::Array<double*> tmp;
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
    dd = sumOfSqr(y_f) - yf_vf * yf_vf_norm;
    CHECK(fabs(dd - d*d)<1e-8, "");
    
    //compute gradient
    dv.referToDim(dDdX, vertices(i));
    dv = f - y + yf_vf_norm*v_f;
    dv *= 2.*yf_vf_norm;
    dv *= .5/d;
  }
  
  return d;
}

//===========================================================================

double forceClosure(const arr& C, const arr& Cn, const mlr::Vector& center,
                    double mu, double torqueWeights, arr *dFdC) { //, arr *dFdCn
  CHECK_EQ(C.d0,Cn.d0, "different number of points and normals");
  CHECK_EQ(C.d1,3, "");
  
  uint i, j, S=7;
  mlr::Vector c, n;
  
  arr X;
  if(torqueWeights>0.)  X.resize(C.d0*S, 6);  //store 6d points for convex wrench hull
  else X.resize(C.d0*S, 3);                //store 3d points for convex force hull
  
  arr dXdC;
  if(dFdC) {
    dXdC.resize(X.d0,X.d1, 3);
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
    
    mlr::Quaternion r;
    r.setDiff(Vector_z, n);//rotate cone's z-axis into contact normal n
    
    for(j=0; j<S; j++) {   //each sample, equidistant on a circle
      double angle = j*MLR_2PI/S;
      mlr::Vector f(cos(angle)*mu, sin(angle)*mu, 1.);  //force point sampled from cone
      
      f = r*f;                         //rotate
      mlr::Vector c_f = c^f;
      
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
    d = -distanceToConvexHull(X, origin, 0, 0, true);
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
  auto lock = qhullMutex();

  int exitcode;
  uint dim=V.d1;
  static char* cmd = (char*) "qhull Qt ";
  exitcode = qh_new_qhull(V.d1, V.d0, V.p, false, cmd, NULL, stderr);
  if(exitcode) HALT("qh_new_qhull error - exitcode " <<exitcode);


  qh_triangulate();

  facetT *facet;
  vertexT *vertex, **vertexp;
  uint f, i, v;

  arr Vnew;
  Vnew.resize(qh num_vertices, dim);
  i=0;
  FORALLvertices {
    vertex->id = i;
    memmove(&Vnew(i, 0), vertex->point,  dim*sizeof(double));
    i++;
  }
  if(&T){ //retrieve also the triangulation
    T.resize(qh num_facets, dim);
    f=0;
    FORALLfacets {
      i=0;
      FOREACHvertex_(facet->vertices) {
        if(i<3) T(f, i)=vertex->id; else MLR_MSG("face " <<f <<" has " <<i <<" vertices" <<endl);
        i++;
      }
      if(facet->toporient) {
        v=T(f, 0);  T(f, 0)=T(f, 1);  T(f, 1)=v;
      }
      f++;
    }
    CHECK_EQ(f,T.d0, "");
  }

  qh_freeqhull(!qh_ALL);
  int curlong, totlong;
  qh_memfreeshort(&curlong, &totlong);
  if(curlong || totlong)
    MLR_MSG("qhull internal warning (main): did not free " <<totlong <<" bytes of long memory (" <<curlong <<" pieces)\n");
    
  return Vnew;
}

void getDelaunayEdges(uintA& E, const arr& V) {
  auto lock = qhullMutex();

  if(V.d0<3) { E.clear(); return; }
  int exitcode;
  static char* cmd = (char*) "qhull d Qbb Qt ";
  exitcode = qh_new_qhull(V.d1, V.d0, V.p, false, cmd, NULL, stderr);
  if(exitcode) HALT("qh_new_qhull error - exitcode " <<exitcode);
  
  facetT *facet;
  vertexT *vertex, **vertexp;
  uint i, j, k, dim=V.d1;
  
  E.clear();
  uint face[dim+1];
  FORALLfacets {
    if(!facet->upperdelaunay) {
      i=0;
      FOREACHvertex_(facet->vertices) face[i++]=qh_pointid(vertex->point);//vertex->id;
      CHECK_EQ(i,dim+1, "strange number of vertices of a facet!");
      for(j=0; j<dim+1; j++) for(k=j+1; k<dim+1; k++) {
          E.append(TUP(face[j], face[k]));
        }
    }
  }
  E.reshape(E.N/2,2);
  
  qh_freeqhull(!qh_ALL);
  int curlong, totlong;
  qh_memfreeshort(&curlong, &totlong);
  if(curlong || totlong)
    MLR_MSG("qhull internal warning (main): did not free " <<totlong <<" bytes of long memory (" <<curlong <<" pieces)\n");
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
    CHECK_EQ(g.nodes(i)->point.N,dim, "point doesn't have expected dim in delaunay");
    P[i]=(doubleA&)(*(g.nodes(i)));
    //P(i, 0)=g.nodes(i)->feat.x;
    //P(i, 1)=g.nodes(i)->feat.y;
    //P(i, 2)=g.nodes(i)->feat.z;
  }
  
  if(!qh_new_qhull(dim, g.N, P.p, false, "qhull d Qbb T0", NULL, stderr)) {
    facetT *facet;
    vertexT *vertex, **vertexp;
    uint *face, k, l;
    face=new uint[dim+1];
    
    FORALLfacets {
      if(!facet->upperdelaunay) {
        uint j=0;
        FOREACHvertex_(facet->vertices) face[j++]=qh_pointid(vertex->point);
        CHECK_EQ(j,dim+1, "strange number of vertices of a facet!");
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
    MLR_MSG("qhull did not free " <<totlong <<" bytes of long memory (" <<curlong <<" pieces)");
}

#endif

#else ///MLR_QHULL
#include <Core/util.h>
#include <Core/array.h>
#include "geo.h"
int QHULL_DEBUG_LEVEL=0;
const char* qhullVersion() { return "NONE"; }
void getTriangulatedHull(uintA& T, arr& V) { NICO }
double forceClosure(const arr& C, const arr& Cn, const mlr::Vector& center,
                    double mu, double torqueWeights, arr *dFdC) { NICO }
double distanceToConvexHull(const arr &X, const arr &y, arr *projectedPoint, uintA *faceVertices, bool freeqhull) { NICO }
double distanceToConvexHullGradient(arr& dDdX, const arr &X, const arr &y, bool freeqhull) { NICO }
void getDelaunayEdges(uintA& E, const arr& V) { NICO }
#endif
/** @} */


