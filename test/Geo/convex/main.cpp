#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Algo/algos.h>
#include <Plot/plot.h>
#include <qhull/qhull_a.h>
#include <Optim/optimization.h>
#include <Geo/qhull.h>
#include <Optim/convert.h>

//===========================================================================

void plotQhullState(uint D) {
  double *point, *pointtemp;
  vertexT *vertex, **vertexp;
  facetT *facet;
  arr x, lines;

  plot()->Clear();

  cout <<"\n** points:";
  FORALLpoints {
    x.setCarray(point, D);
    cout <<"\n  " <<x;
    plot()->Points(x);
  }

  cout <<"\n** vertices:";
  FORALLvertices {
//    vertices.setCarray(vertex->point, D);
    uint i = (vertex->point - (qh first_point))/D;
    cout <<"\n  " <<vertex->id <<": " <<i;
  }

  cout <<"\n** facets:";
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
    plot()->Line(lines);
  }
  cout <<endl;
}

//===========================================================================
//
// 1. test: convex hull distance and its gradient
//

void TEST(ConvexHull) {
  uint N=20,D=2;
  arr X(N,D);
  rndUniform(X,-1.,1.,false);
  cout <<"X=" <<X <<endl;

  arr origin(D),p;
  origin.setZero();
  uintA V;
  double d;

  QHULL_DEBUG_LEVEL=2;

  for(uint i=0;i<100;i++){
    d = distanceToConvexHull(X, origin, NoArr, p, &V, false);
    plotQhullState(D);
    qhull_free();

    if(p.d0) {
      arr line = ~origin;
      line.append(p);
      plot()->Line(line);
    }
//    plot()->Opengl();

    cout
      <<"\ndistance = " <<d
      <<"\nprojected point = " <<p
      <<"\nface vertices = " <<V
      <<endl;

    origin(0) += .02;
  }

  arr grad;
//  d = distanceToConvexHullGradient(grad,X,origin,true);
  cout <<"gradient = " <<grad <<endl;

  QHULL_DEBUG_LEVEL=0;

  N=20;
  D=6;
  X.resize(N,D);
  origin.resize(D);

  ScalarFunction f = [&origin, &X](arr& g, arr& H, const arr& x) -> double {
    double d=distanceToConvexHull(X, origin);
    if(!!g) distanceToConvexHullGradient(g, X, origin);
    return d;
  };

  for(uint k=0;k<20;k++){
    rndUniform(origin, -1.2, 1.2, false);
    rndUniform(X, -1., 1.,false);
    checkGradient(f, X, 1e-4);
  }
}

//===========================================================================
//
// 2. test: force closure measure and its gradient
//

namespace FCtest{
  rai::Vector center;
  arr Xn;
  double f(arr *grad,const arr& X,void*){
    return forceClosure(X,Xn,center,.5,10.,grad);
  }
}

void TEST(ForceClosure) {
  uint N=4,i,k;
  arr X(N,3),Xn(N,3),c(3);
  c.setZero();
  rndUniform(X,-1.,1.,false);

  Xn=X;  for(i=0;i<N;i++) Xn[i]() /= length(Xn[i]);

  plot()->Opengl();
  plot()->Clear(); plot()->Points(c); plot()->Points(X); plot()->VectorField(X,Xn);
  plot()->update(false);

  arr dFdX;

  double d;
  rai::Vector center;
  center.set(c.p);

  //gradient descent on force closure
  for(k=0;k<1000;k++){
    d=forceClosure(X,Xn,center,.5,10.,&dFdX);
    cout <<"d=" <<d <<endl;
    plot()->Clear(); plot()->Points(c); plot()->Points(X); plot()->VectorField(X,Xn);
    plot()->update(false);
    X -= .005*dFdX;
    Xn=X;  for(i=0;i<N;i++) Xn[i]() /= -length(Xn[i]);
  }

  //gradient check
  center.setZero();
  for(k=0;k<100;k++){
    rndUniform(X,-1.,1.,false);
    Xn=X;  for(i=0;i<N;i++) Xn[i]() /= -length(Xn[i]);

    d=forceClosure(X,Xn,center,.5,10.,0);
    cout <<"FC= " <<d <<endl;

    FCtest::center=center;
    FCtest::Xn=Xn;
    checkGradient(Convert(FCtest::f, nullptr), X, 1e-4);
  }
}

//===========================================================================
//
// 3. test: force closure measure applied on ors scenario
//

void drawInit(void*){
  //glStandardLight();
  //glColor(1.,.5,0.);
  //glDrawAxes(1.);
}

/*
void TEST(FCinOrs){
  rai::Configuration C;
  C <<FILE("../../data/configurations/forceClosureTest.g");

  OpenGL gl;
  gl.add(drawInit,0);
  gl.add(rai::glDrawGraph,&C);
  gl.watch();

  SwiftInterface swift;
  swift.cutoff=1000.;
  swift.init(C);

  uint t;
  arr x,v,grad;
  double fc;
  C.getJointState(x,v);
  for(t=0;t<100;t++){
    C.setJointState(x);
    swift.computeProxies(C,false);
    C.sortProxies(true);

    fc=forceClosureFromProxies(C,0);
    cout <<"forceclosure=" <<fc <<endl;
    // C.reportProxies();
    // c=C.getContactGradient(grad,.1); //generate a gradient pushing away to 10cm distance
    // gl->text.clear() <<"t=" <<t <<"  movement along negative contact gradient (using SWIFT to get contacts)";
    gl.watch();
    //x += inverse(grad)*(-.1*c);
    //x -= .01*grad; //.1 * (invJ * grad);
  }
}*/

//===========================================================================

void sort2Dpoints(arr& A);

void testConvConvIntersect(){
  //box cases
//  uint N=20,D=2;
  arr X = {{4,2}, {-1.,-1.,1.,-1.,-1.,1.,1.,1.}};

  arr C1 = .5*X;
  arr C2 = .25*X;
  sort2Dpoints(C1);
  sort2Dpoints(C2);
  for(uint k=0;k<4;k++){
    C2 += .2;

    arr C = convconv_intersect(C1, C2);
    cout <<"#C=" <<C.d0 <<endl;

    plot()->Clear();
    plot()->Line(C+.002, true);
    plot()->Line(C-.002, true);
    plot()->Line(C1, true);
    plot()->Line(C2, true);
    plot()->Opengl();

    rai::wait();
  }

  //rnd case
  for(uint k=0;k<20;k++){
    uint N=20,D=2;
    arr X(N,D);
    rndUniform(X,-1.,1.,false);

    arr C1, C2;
    C1 = getHull(X);
    rndUniform(X,-1.,1.,false);
    C2 = getHull(X);

    sort2Dpoints(C1);
    sort2Dpoints(C2);

    arr C = convconv_intersect(C1, C2);

    plot()->Clear();
    plot()->Line(C+.002, true);
    plot()->Line(C-.002, true);
    plot()->Line(C1, true);
    plot()->Line(C2, true);
    plot()->Opengl();

    rai::wait();
  }

  plot()->Close();
}

//===========================================================================

// void TEST(Speed){
//   uint N=20,D=2;
//   arr X(N,D);
//   rai::timerReset();
//   for(uint i=0;i<100;i++){
//     rndUniform(X,-1.,1.,false);
//     distanceToConvexHull(X,origin,&p,&V,true);
//   }
// }

int MAIN(int argc, char *argv[]){
  rai::initCmdLine(argc, argv);
  cout <<"QHull version = " <<qhullVersion() <<endl;

  //testConvexHull();
  //testForceClosure();
  //testFCinOrs();

  testConvConvIntersect();

  return 0;
}

