#include <Core/util.h>
#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Kin/frame.h>
#include <Kin/proxy.h>

double
pqp_RectDist(double Rab[9], double Tab[3],
double a[2], double b[2], double Pa[3], double Pb[3]);

using rai::Shape;

rai::Vector Pa, Pb;

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

double distance_SSPoints(rai::Frame& A, rai::Frame& B,rai::Vector& Pa, rai::Vector& Pb){
  CHECK(A.shape && A.shape->type()==rai::ST_ssBox && B.shape && B.shape->type()==rai::ST_ssBox,"");
  CHECK(!A.shape->size(0) && !B.shape->size(0) && !A.shape->size(1) && !B.shape->size(1) && !A.shape->size(2) && !B.shape->size(2), "can only handle SSpoints");
  Pa = A.ensure_X().pos;
  Pb = B.ensure_X().pos;
  rai::Vector c = Pa-Pb;
  double d = c.length();
  //account for radii
  Pa -= A.shape->size(3)*c/d;
  Pb += B.shape->size(3)*c/d;
  return d-A.shape->size(3)-B.shape->size(3);
}

double distance_SSLinePoint(rai::Frame& A, rai::Frame& B,rai::Vector& Pa, rai::Vector& Pb){
  CHECK(A.shape && A.shape->type()==rai::ST_ssBox && B.shape && B.shape->type()==rai::ST_ssBox,"");
  CHECK(!B.shape->size(0) && !A.shape->size(1) && !B.shape->size(1) && !A.shape->size(2) && !B.shape->size(2), "can only handle SSLinePoint");
  if(!A.shape->size(0)){ //SSLinePoint
    return distance_SSPoints(A, B, Pa, Pb);
  }
  rai::Vector a=A.ensure_X().rot.getX();
  rai::Vector c=B.ensure_X().pos - A.ensure_X().pos;
  //get the 'coordinate' along the line segment
  double t = c*a;
  clip(t, A.shape->size(0));
  //compute closest points
  Pa = A.ensure_X().pos + t*a;
  Pb = B.ensure_X().pos;
  //distance
  c = Pa-Pb;
  double d = c.length();
  //account for radii
  Pa -= A.shape->size(3)*c/d;
  Pb += B.shape->size(3)*c/d;
  return d-A.shape->size(3)-B.shape->size(3);
}

double distance_SSLines(rai::Frame& A, rai::Frame& B,rai::Vector& Pa, rai::Vector& Pb){
  CHECK(A.shape && A.shape->type()==rai::ST_ssBox && B.shape && B.shape->type()==rai::ST_ssBox,"");
  CHECK(!A.shape->size(1) && !B.shape->size(1) && !A.shape->size(2) && !B.shape->size(2), "can only handle SS line segments (cylinders)");
  if(!B.shape->size(0)){ //SSLinePoint
    return distance_SSLinePoint(A, B, Pa, Pb);
  }
  rai::Vector a=A.ensure_X().rot.getX();
  rai::Vector b=B.ensure_X().rot.getX();
  rai::Vector c=B.ensure_X().pos - A.ensure_X().pos;
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
  Pa = A.ensure_X().pos + t*a;
  Pb = B.ensure_X().pos + u*b;
  //distance
  c = Pa-Pb;
  double d = c.length();
  //account for radii
  Pa -= A.shape->size(3)*c/d;
  Pb += B.shape->size(3)*c/d;
  return d-A.shape->size(3)-B.shape->size(3);
}

double distance_SSRects(rai::Frame& A, rai::Frame& B, rai::Vector& Pa, rai::Vector& Pb){
  CHECK(A.shape && A.shape->type()==rai::ST_ssBox && B.shape && B.shape->type()==rai::ST_ssBox,"");
  CHECK(!A.shape->size(2) && !B.shape->size(2), "can only handle spheres, cylinders & rectangles yet - no boxes");
  if(!A.shape->size(1) && !B.shape->size(1)){ //SSLines
    return distance_SSLines(A, B, Pa, Pb);
  }
  rai::Transformation f;
  f.setDifference(A.ensure_X(), B.ensure_X());
  rai::Matrix R = ((f.rot)).getMatrix();
  rai::Vector Asize={A.shape->size(0), A.shape->size(1), 0.};
  rai::Vector Bsize={B.shape->size(0), B.shape->size(1), 0.};
  rai::Vector trans = f.pos; //Asize + f.pos - R*Bsize;
  double dist = pqp_RectDist(R.p(), trans.p(), (Asize).p(), (Bsize).p(), Pa.p(), Pb.p());
  Pa = A.ensure_X() * Pa;
  Pb = A.ensure_X() * Pb;
   //distance
  rai::Vector c = Pa-Pb;
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
double distance_(rai::Frame& A, rai::Frame& B, rai::Vector& Pa, rai::Vector& Pb){
  CHECK(A.shape && B.shape, "");
  arr& As = A.shape->size();
  arr& Bs = B.shape->size();
  As(0)-=2.*As(3);  As(1)-=2.*As(3);  As(2)-=2.*As(3);
  Bs(0)-=2.*Bs(3);  Bs(1)-=2.*Bs(3);  Bs(2)-=2.*Bs(3);
  A.set_X()->pos -= 0.5*(A.ensure_X().rot*rai::Vector(As(0), As(1), As(2)));
  B.set_X()->pos -= 0.5*(B.ensure_X().rot*rai::Vector(Bs(0), Bs(1), Bs(2)));
  double d=distance_SSRects(A, B, Pa, Pb);
  A.set_X()->pos += 0.5*(A.ensure_X().rot*rai::Vector(As(0), As(1), As(2)));
  B.set_X()->pos += 0.5*(B.ensure_X().rot*rai::Vector(Bs(0), Bs(1), Bs(2)));
  As(0)+=2.*As(3);  As(1)+=2.*As(3);  As(2)+=2.*As(3);
  Bs(0)+=2.*Bs(3);  Bs(1)+=2.*Bs(3);  Bs(2)+=2.*Bs(3);
  return d;
}

void TEST(Distance){
  rai::Configuration C;
  rai::Frame A(C), B(C);
  new rai::Shape(A);
  new rai::Shape(B);
  A.setShape(rai::ST_ssBox, {1.6, 1.6, .0, .0});
  B.setShape(rai::ST_ssBox, {1.6, 1.6, .0, .0});
  for(uint k=0;k<20;k++){
    A.set_X()->setRandom(); A.set_X()->pos(2) += 2.;
    B.set_X()->setRandom(); B.set_X()->pos(2) += 2.;
    double d=distance_(A, B, Pa, Pb);
    double d2=(Pa-Pb).length();
    cout <<"d=" <<d <<' ' <<d2 <<' ' <<Pa <<Pb <<endl;
    if(d>0.) CHECK_ZERO(d-d2, 1e-4, "NOT EQUAL!");
    rai::Proxy p; p.a=&A, p.b=&B; p.posA=Pa; p.posB=Pb; p.colorCode=1;
    C.proxies.clear();
    C.proxies.append( p );
    C.watch(true);
  }
}

int MAIN(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  testDistance();

  return 0;
}
