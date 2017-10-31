#include <Core/util.h>
#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Kin/frame.h>
#include <Kin/proxy.h>

double
pqp_RectDist(double Rab[9], double Tab[3],
double a[2], double b[2], double Pa[3], double Pb[3]);

using mlr::Shape;

mlr::Vector Pa, Pb;

void draw(void*){
  glLoadIdentity();
  glColor(1., 0., 0., .9);
  glDrawDiamond(Pa.x, Pa.y, Pa.z, .1, .1, .1);
  glDrawDiamond(Pb.x, Pb.y, Pb.z, .1, .1, .1);
  glBegin(GL_LINES);
  glVertex3f(Pa.x, Pa.y, Pa.z);
  glVertex3f(Pb.x, Pb.y, Pb.z);
  glEnd();
  glLoadIdentity();
}

inline void clip(double& x, double r){
  if(x<0.) x=0.; else if(x>r) x=r;
}


double distance_SSPoints(mlr::Frame& A, mlr::Frame& B,mlr::Vector& Pa, mlr::Vector& Pb){
  CHECK(A.shape && A.shape->type()==mlr::ST_retired_SSBox && B.shape && B.shape->type()==mlr::ST_retired_SSBox,"");
  CHECK(!A.shape->size(0) && !B.shape->size(0) && !A.shape->size(1) && !B.shape->size(1) && !A.shape->size(2) && !B.shape->size(2), "can only handle SSpoints");
  Pa = A.X.pos;
  Pb = B.X.pos;
  mlr::Vector c = Pa-Pb;
  double d = c.length();
  //account for radii
  Pa -= A.shape->size(3)*c/d;
  Pb += B.shape->size(3)*c/d;
  return d-A.shape->size(3)-B.shape->size(3);
}

double distance_SSLinePoint(mlr::Frame& A, mlr::Frame& B,mlr::Vector& Pa, mlr::Vector& Pb){
  CHECK(A.shape && A.shape->type()==mlr::ST_retired_SSBox && B.shape && B.shape->type()==mlr::ST_retired_SSBox,"");
  CHECK(!B.shape->size(0) && !A.shape->size(1) && !B.shape->size(1) && !A.shape->size(2) && !B.shape->size(2), "can only handle SSLinePoint");
  if(!A.shape->size(0)){ //SSLinePoint
    return distance_SSPoints(A, B, Pa, Pb);
  }
  mlr::Vector a=A.X.rot.getX();
  mlr::Vector c=B.X.pos - A.X.pos;
  //get the 'coordinate' along the line segment
  double t = c*a;
  clip(t, A.shape->size(0));
  //compute closest points
  Pa = A.X.pos + t*a;
  Pb = B.X.pos;
  //distance
  c = Pa-Pb;
  double d = c.length();
  //account for radii
  Pa -= A.shape->size(3)*c/d;
  Pb += B.shape->size(3)*c/d;
  return d-A.shape->size(3)-B.shape->size(3);
}

double distance_SSLines(mlr::Frame& A, mlr::Frame& B,mlr::Vector& Pa, mlr::Vector& Pb){
  CHECK(A.shape && A.shape->type()==mlr::ST_retired_SSBox && B.shape && B.shape->type()==mlr::ST_retired_SSBox,"");
  CHECK(!A.shape->size(1) && !B.shape->size(1) && !A.shape->size(2) && !B.shape->size(2), "can only handle SS line segments (cylinders)");
  if(!B.shape->size(0)){ //SSLinePoint
    return distance_SSLinePoint(A, B, Pa, Pb);
  }
  mlr::Vector a=A.X.rot.getX();
  mlr::Vector b=B.X.rot.getX();
  mlr::Vector c=B.X.pos - A.X.pos;
  //get the 'coordinates' along the line segments
  double A_dot_B = a*b;
  double A_dot_C = a*c;
  double B_dot_C = b*c;
  double denom = 1. - A_dot_B*A_dot_B;
  double t, u;
  if(denom==0.) t=0.; else t = (A_dot_C - B_dot_C*A_dot_B)/denom;
  clip(t, A.shape->size(0));
  u = t*A_dot_B - B_dot_C;
  clip(u, B.shape->size(0));
  t = u*A_dot_B + A_dot_C;
  clip(t, A.shape->size(0));
  //compute closest points
  Pa = A.X.pos + t*a;
  Pb = B.X.pos + u*b;
  //distance
  c = Pa-Pb;
  double d = c.length();
  //account for radii
  Pa -= A.shape->size(3)*c/d;
  Pb += B.shape->size(3)*c/d;
  return d-A.shape->size(3)-B.shape->size(3);
}

double distance_SSRects(mlr::Frame& A, mlr::Frame& B, mlr::Vector& Pa, mlr::Vector& Pb){
  CHECK(A.shape && A.shape->type()==mlr::ST_ssBox && B.shape && B.shape->type()==mlr::ST_ssBox,"");
  CHECK(!A.shape->size(2) && !B.shape->size(2), "can only handle spheres, cylinders & rectangles yet - no boxes");
  if(!A.shape->size(1) && !B.shape->size(1)){ //SSLines
    return distance_SSLines(A, B, Pa, Pb);
  }
  mlr::Transformation f;
  f.setDifference(A.X, B.X);
  mlr::Matrix R = ((f.rot)).getMatrix();
  mlr::Vector Asize={A.shape->size(0), A.shape->size(1), 0.};
  mlr::Vector Bsize={B.shape->size(0), B.shape->size(1), 0.};
  mlr::Vector trans = f.pos; //Asize + f.pos - R*Bsize;
  double dist = pqp_RectDist(R.p(), trans.p(), (Asize).p(), (Bsize).p(), Pa.p(), Pb.p());
  Pa = A.X * Pa;
  Pb = A.X * Pb;
   //distance
  mlr::Vector c = Pa-Pb;
  double d = c.length();
  if(dist>0.) CHECK_ZERO(dist-d, 1e-4, "NOT EQUAL!");
  if(dist==0.) d *= -1.; //if the rects penetrate already, measure the penetration as negative!
  //account for radii
  Pa -= A.shape->size(3)*c/d;
  Pb += B.shape->size(3)*c/d;
  return d-A.shape->size(3)-B.shape->size(3);
}


/* NOTE: All functions above: Internally they assume the shape's not centered, but extended from (0,0,0) to the positive coordinates
 * That is different to the 'Shape' convention, where shapes are centered and extend (with half length) to negative and positive coordinates
 * In the code this is transformed back and forth... */
double distance_(mlr::Frame& A, mlr::Frame& B, mlr::Vector& Pa, mlr::Vector& Pb){
  CHECK(A.shape && B.shape, "");
  arr& As = A.shape->size();
  arr& Bs = B.shape->size();
  As(0)-=2.*As(3);  As(1)-=2.*As(3);  As(2)-=2.*As(3);
  Bs(0)-=2.*Bs(3);  Bs(1)-=2.*Bs(3);  Bs(2)-=2.*Bs(3);
  A.X.pos -= 0.5*(A.X.rot*mlr::Vector(As(0), As(1), As(2)));
  B.X.pos -= 0.5*(B.X.rot*mlr::Vector(Bs(0), Bs(1), Bs(2)));
  double d=distance_SSRects(A, B, Pa, Pb);
  A.X.pos += 0.5*(A.X.rot*mlr::Vector(As(0), As(1), As(2)));
  B.X.pos += 0.5*(B.X.rot*mlr::Vector(Bs(0), Bs(1), Bs(2)));
  As(0)+=2.*As(3);  As(1)+=2.*As(3);  As(2)+=2.*As(3);
  Bs(0)+=2.*Bs(3);  Bs(1)+=2.*Bs(3);  Bs(2)+=2.*Bs(3);
  return d;
}

void TEST(Distance){
  mlr::KinematicWorld K;
  mlr::Frame A(K), B(K);
  new mlr::Shape(A);
  new mlr::Shape(B);
  A.shape->type() = B.shape->type() = mlr::ST_ssBox;
  A.shape->size() = ARR(1.6, 1.6, .0, .0);
  B.shape->size() = ARR(1.6, 1.6, .0, .0);
  for(uint k=0;k<20;k++){
    A.X.setRandom(); A.X.pos(2) += 2.;
    B.X.setRandom(); B.X.pos(2) += 2.;
    double d=distance_(A, B, Pa, Pb);
    double d2=(Pa-Pb).length();
    cout <<"d=" <<d <<' ' <<d2 <<' ' <<Pa <<Pb <<endl;
    if(d>0.) CHECK_ZERO(d-d2, 1e-4, "NOT EQUAL!");
    mlr::Proxy p; p.posA=Pa; p.posB=Pb; p.colorCode=1;
    K.proxies.append( &p );
    K.gl().timedupdate(.1);
    K.gl().watch();
    K.proxies.clear();
  }
}

int MAIN(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  testDistance();

  return 0;
}
