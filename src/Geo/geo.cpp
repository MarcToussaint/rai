/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "geo.h"

#include "../Core/array.h"
#include "../Core/util.h"
#include "../Core/graph.h"

#include <algorithm>
#include <math.h>

#ifdef RAI_GL
#  include <GL/glew.h>
#  include <GL/glu.h>
#endif

#pragma GCC diagnostic ignored "-Wclass-memaccess"

const rai::Vector Vector_x(1, 0, 0);
const rai::Vector Vector_y(0, 1, 0);
const rai::Vector Vector_z(0, 0, 1);
const rai::Transformation Transformation_Id(rai::Transformation().setZero());
const rai::Quaternion Quaternion_Id(1, 0, 0, 0);
const rai::Quaternion Quaternion_x(RAI_SQRT2/2., RAI_SQRT2/2., 0, 0);
const rai::Quaternion Quaternion_y(RAI_SQRT2/2., 0, RAI_SQRT2/2., 0);
const rai::Quaternion Quaternion_z(RAI_SQRT2/2., 0, 0, RAI_SQRT2/2.);
rai::Vector __NoVector;
rai::Vector& NoVector = __NoVector;
rai::Transformation __NoTransformation;
rai::Transformation& NoTransformation = __NoTransformation;

namespace rai {

bool Vector::operator!() const { return this==&NoVector; }

double& Vector::operator()(uint i) {
  CHECK(i<3, "out of range");
  isZero=false;
  return (&x)[i];
}

/// set the vector
void Vector::set(double _x, double _y, double _z) { x=_x; y=_y; z=_z; isZero=(x==0. && y==0. && z==0.); }

/// set the vector
void Vector::set(const double* p) { x=p[0]; y=p[1]; z=p[2]; isZero=(x==0. && y==0. && z==0.); }

/// set the vector
void Vector::setZero() { memset(this, 0, sizeof(Vector)); isZero=true; }

/// a random vector in [-1, 1]^3
void Vector::setRandom(double range) { x=rnd.uni(-range, range); y=rnd.uni(-range, range); z=rnd.uni(-range, range); isZero=false; }

//{ vector operations

/// this=this/length(this)
void Vector::normalize() {
  if(isZero) {
    RAI_MSG("can't normalize length of null vector");
    return;
  }
  (*this)/=length();
}

/// this=this*l/length(this)
void Vector::setLength(double l) {
  if(isZero) RAI_MSG("can't change length of null vector");
  (*this)*=l/length();
}

/// this=component of this normal to \c b, (unnormalized!)
void Vector::makeNormal(const Vector& b) {
  if(b.isZero) RAI_MSG("can't makeNormal with null vector");
  double l=b.length(), s=x*b.x+y*b.y+z*b.z;
  s/=l*l;
  x-=s*b.x; y-=s*b.y; z-=s*b.z;
}

/// this=component of this colinear to \c b, (unnormalized!)
void Vector::makeColinear(const Vector& b) {
  if(b.isZero) RAI_MSG("can't makeColinear with null vector");
  // *this = ((*this)*b)/b.length()) * (*this);
  double l=b.length(), s=x*b.x+y*b.y+z*b.z;
  s/=l*l;
  x=s*b.x; y=s*b.y; z=s*b.z;
}

//{ measuring the vector

/// L1-norm to zero
double Vector::diffZero() const { return fabs(x)+fabs(y)+fabs(z); }

/// check whether isZero is true
void Vector::checkZero() const {
  bool iszero = (x==0. && y==0. && z==0.);
  if(isZero) if(!iszero) HALT("you must have set this by hand!");
}

/// is it normalized?
bool Vector::isNormalized() const { return fabs(lengthSqr()-1.)<1e-6; }

/// returns the length of this
double Vector::length() const { return ::sqrt(lengthSqr()); }

/// returns the square of length |a|^2
double Vector::lengthSqr() const { return x*x + y*y + z*z; }

double Vector::sum() const { return x+y+z; }

/// angle in [0..pi] between this and b
double Vector::angle(const Vector& b) const {
  double a=((*this)*b)/(length()*b.length());
  if(a<-1.) a=-1.;
  if(a>1.) a=1.;
  return ::acos(a);
}

/** @brief if \c this and \c b are colinear, it returns the factor c such
    that this=c*b; otherwise it returns zero */
double Vector::isColinear(const Vector& b) const {
  double c=x/b.x;
  if(y==c*b.y && z==c*b.z) return c;
  return 0.;
}

//{ sphere coordinates

/// the radius in the x/y-plane
double Vector::radius() const { return ::sqrt(x*x+y*y); }

/// the angle in the x/y-plane in [-pi, pi]
double Vector::phi() const {
  double ph;
  if(x==0. || ::fabs(x)<1e-10) ph=RAI_PI/2.; else ph=::atan(y/x);
  if(x<0.) { if(y<0.) ph-=RAI_PI; else ph+=RAI_PI; }
  return ph;
}

/// the angle from the x/y-plane
double Vector::theta() const { return ::atan(z/radius())+RAI_PI/2.; }

Vector Vector::getNormalVectorNormalToThis() const {
  if(isZero) {
    RAI_MSG("every vector is normal to a zero vector");
  }
  arr s = arr{fabs(x), fabs(y), fabs(z)};
  uint c = argmax(s);
  double xv, yv, zv;
  if(c == 0) {
    xv = -(y+z)/x;
    yv = 1.0;
    zv = 1.0;
  } else if(c == 1) {
    xv = 1.0;
    yv = -(x+z)/y;
    zv = 1.0;
  } else {
    xv = 1.0;
    yv = 1.0;
    zv = -(x+y)/z;
  }
  Vector v(xv, yv, zv);
  v.normalize();
  return v;
}

void Vector::generateOrthonormalSystem(Vector& u, Vector& v) const {
  u = getNormalVectorNormalToThis();
  v = (*this)^u;
  v.normalize();
}

arr Vector::generateOrthonormalSystemMatrix() const {
  arr V;
  Vector n = *this;
  n.normalize();
  Vector u = getNormalVectorNormalToThis();
  Vector v = n^u;
  v.normalize();
  V.append(~conv_vec2arr(n));
  V.append(~conv_vec2arr(u));
  V.append(~conv_vec2arr(v));
  return ~V;
}

//{ I/O
void Vector::write(std::ostream& os) const {
  os <<'[' <<x <<", " <<y <<", " <<z <<']';
}

void Vector::read(std::istream& is) {
  is >>PARSE("[") >>x >>y >>z >>PARSE("]");
}
//}

/// scalar product (inner product)
double operator*(const Vector& a, const Vector& b) {
  return a.x*b.x+a.y*b.y+a.z*b.z;
}

/// cross product (corresponds to antisymmetric exterior product)
Vector operator^(const Vector& b, const Vector& c) {
  Vector a;
  a.x=b.y*c.z-b.z*c.y;
  a.y=b.z*c.x-b.x*c.z;
  a.z=b.x*c.y-b.y*c.x;
  a.isZero = (a.x==0. && a.y==0. && a.z==0.);
  return a;
}

/// sum of two vectors
Vector operator+(const Vector& b, const Vector& c) {
  Vector a;
  a.x=b.x+c.x;
  a.y=b.y+c.y;
  a.z=b.z+c.z;
  a.isZero = (a.x==0. && a.y==0. && a.z==0.);
  return a;
}

/// difference between two vectors
Vector operator-(const Vector& b, const Vector& c) {
  Vector a;
  a.x=b.x-c.x;
  a.y=b.y-c.y;
  a.z=b.z-c.z;
  a.isZero = (a.x==0. && a.y==0. && a.z==0.);
  return a;
}

/// multiplication with a scalar
Vector operator*(double b, const Vector& c) {
  Vector a;
  a.x=b*c.x;
  a.y=b*c.y;
  a.z=b*c.z;
  a.isZero = c.isZero && (b==0.);
  return a;
}

/// multiplication with a scalar
Vector operator*(const Vector& b, double c) { return c*b; }

/// division by a scalar
Vector operator/(const Vector& b, double c) { return (1./c)*b; }

/// multiplication with a scalar
Vector& operator*=(Vector& a, double c) {
  a.x*=c; a.y*=c; a.z*=c;
  a.isZero = a.isZero && (c==0.);
  return a;
}

/// divide by a scalar
Vector& operator/=(Vector& a, double c) {
  a.x/=c; a.y/=c; a.z/=c;
  return a;
}

/// add a vector
Vector& operator+=(Vector& a, const Vector& b) {
  a.x+=b.x; a.y+=b.y; a.z+=b.z;
  a.isZero = (a.x==0. && a.y==0. && a.z==0.);
  return a;
}

/// subtract a vector
Vector& operator-=(Vector& a, const Vector& b) {
  a.x-=b.x; a.y-=b.y; a.z-=b.z;
  a.isZero = (a.x==0. && a.y==0. && a.z==0.);
  return a;
}

/// return the negative of a vector
Vector operator-(const Vector& b) {
  Vector a;
  a.x=-b.x; a.y=-b.y; a.z=-b.z;
  a.isZero = b.isZero;
  return a;
}

// all operator== and operator!=
bool operator==(const Quaternion& lhs, const Quaternion& rhs) {
  return lhs.w == rhs.w && lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool operator!=(const Quaternion& lhs, const Quaternion& rhs) {
  return !(lhs == rhs);
}

bool operator==(const Transformation& lhs, const Transformation& rhs) {
  if(!rhs) return !lhs; //if rhs==NoArr
  return lhs.pos == rhs.pos && lhs.rot == rhs.rot;
}

bool operator==(const DynamicTransformation& lhs, const DynamicTransformation& rhs) {
  bool vel_equal = false;
  if(lhs.zeroVels == rhs.zeroVels && rhs.zeroVels == false)
    vel_equal = lhs.vel == rhs.vel && lhs.angvel == rhs.angvel;
  else if(lhs.zeroVels == rhs.zeroVels && rhs.zeroVels == true)
    vel_equal = true;
  return vel_equal && lhs.pos == rhs.pos && lhs.rot == rhs.rot;
}

bool operator!=(const Transformation& lhs, const Transformation& rhs) {
  return !(lhs == rhs);
}

bool operator==(const Matrix& lhs, const Matrix& rhs) {
  return lhs.m00 == rhs.m00 && lhs.m01 == rhs.m01 && lhs.m02 == rhs.m02 &&
         lhs.m10 == rhs.m10 && lhs.m11 == rhs.m11 && lhs.m12 == rhs.m12 &&
         lhs.m20 == rhs.m20 && lhs.m21 == rhs.m21 && lhs.m22 == rhs.m22;
}

bool operator!=(const Matrix& lhs, const Matrix& rhs) {
  return !(lhs == rhs);
}

bool operator==(const Vector& lhs, const Vector& rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool operator!=(const Vector& lhs, const Vector& rhs) {
  return !(lhs == rhs);
}

double sqrDistance(const Vector& a, const Vector& b) {
  return (a-b).lengthSqr();
}

//==============================================================================

/// reset to zero
void Matrix::setZero() { memset(this, 0, sizeof(Matrix)); }

void Matrix::setRandom(double range) {
  for(uint i=0; i<9; i++) p()[i]=rnd.uni(-range, range);
}

/// reset to identity
void Matrix::setId() {
  m00=m11=m22=1.;
  m01=m02=m10=m12=m20=m21=0.;
}

void Matrix::setDiag(const arr& diag) {
  CHECK_EQ(diag.N, 3, "");
  setZero();
  m00=diag.elem(0);
  m11=diag.elem(1);
  m22=diag.elem(2);
}

void Matrix::setSymmetric(const arr& entries6) {
  CHECK_EQ(entries6.N, 6, "");
  setZero();
  m00=entries6.elem(0);
  m11=entries6.elem(3);
  m22=entries6.elem(5);
  m01=m10=entries6.elem(1);
  m02=m20=entries6.elem(2);
  m12=m21=entries6.elem(4);
}

/// set the matrix
void Matrix::set(double* p) {
  m00=p[0]; m01=p[1]; m02=p[2];
  m10=p[3]; m11=p[4]; m12=p[5];
  m20=p[6]; m21=p[7]; m22=p[8];
}

/// assign the matrix to the transformation from unit frame to given XYZ frame
void Matrix::setFrame(Vector& X, Vector& Y, Vector& Z) {
  m00=X.x; m01=Y.x; m02=Z.x;
  m10=X.y; m11=Y.y; m12=Z.y;
  m20=X.z; m21=Y.z; m22=Z.z;
}

/// assign the matrix to the transformation from the ORTHOGONAL XYZ frame to the unit frame
void Matrix::setInvFrame(Vector& X, Vector& Y, Vector& Z) {
  m00=X.x; m01=X.y; m02=X.z;
  m10=Y.x; m11=Y.y; m12=Y.z;
  m20=Z.x; m21=Z.y; m22=Z.z;
}

/// assign the matrix to a rotation around the X-axis with angle a (in rad units)
void Matrix::setXrot(double a) {
  m00=1.; m01=0.;     m02=0.;
  m10=0.; m11=cos(a); m12=-sin(a);
  m20=0.; m21=sin(a); m22= cos(a);
}

void Matrix::setSkew(const Vector& a) {
  m00=  0.; m01=-a.z; m02= a.y;
  m10= a.z; m11=  0.; m12=-a.x;
  m20=-a.y; m21= a.x; m22=  0.;
}

void Matrix::setExponential(const Vector& a) {
  Matrix S;
  double phi=a.length();
  if(phi<1e-10) { setId(); return; }
  S.setSkew(a/phi);
  *this = sin(phi)*S + (1.-cos(phi))*S*S;
  m00+=1.; m11+=1.; m22+=1.;
}

void Matrix::setOdeMatrix(double* o) {
  m00=o[0]; m01=o[1]; m02=o[2];
  m10=o[4]; m11=o[5]; m12=o[6];
  m20=o[8]; m21=o[9]; m22=o[10];
}

void Matrix::setTensorProduct(const Vector& b, const Vector& c) {
  m00=b.x*c.x; m01=b.x*c.y; m02=b.x*c.z;
  m10=b.y*c.x; m11=b.y*c.y; m12=b.y*c.z;
  m20=b.z*c.x; m21=b.z*c.y; m22=b.z*c.z;
}

/// 1-norm to zero
double Matrix::diffZero() const {
  double d=0.;
  for(uint i=0; i<9; i++) d += (&m00)[i];
  return d;
}

void Matrix::write(std::ostream& os) const {
  os <<"\n[" <<m00 <<", " <<m01 <<", " <<m02;
  os <<"\n " <<m10 <<", " <<m11 <<", " <<m12;
  os <<"\n " <<m20 <<", " <<m21 <<", " <<m22;
  os <<']' <<endl;
}
void Matrix::read(std::istream& is) {
  NIY;
}
//}

/// multiplication of two matrices
Matrix operator*(const Matrix& b, const Matrix& c) {
  Matrix a;
  a.m00=b.m00*c.m00+b.m01*c.m10+b.m02*c.m20;
  a.m01=b.m00*c.m01+b.m01*c.m11+b.m02*c.m21;
  a.m02=b.m00*c.m02+b.m01*c.m12+b.m02*c.m22;

  a.m10=b.m10*c.m00+b.m11*c.m10+b.m12*c.m20;
  a.m11=b.m10*c.m01+b.m11*c.m11+b.m12*c.m21;
  a.m12=b.m10*c.m02+b.m11*c.m12+b.m12*c.m22;

  a.m20=b.m20*c.m00+b.m21*c.m10+b.m22*c.m20;
  a.m21=b.m20*c.m01+b.m21*c.m11+b.m22*c.m21;
  a.m22=b.m20*c.m02+b.m21*c.m12+b.m22*c.m22;
  return a;
}
/// sum of two matrices
Matrix operator+(const Matrix& b, const Matrix& c) {
  Matrix a;
  a.m00=b.m00+c.m00; a.m01=b.m01+c.m01; a.m02=b.m02+c.m02;
  a.m10=b.m10+c.m10; a.m11=b.m11+c.m11; a.m12=b.m12+c.m12;
  a.m20=b.m20+c.m20; a.m21=b.m21+c.m21; a.m22=b.m22+c.m22;
  return a;
}
/// transformation of a vector
Vector operator*(const Matrix& b, const Vector& c) {
  Vector a;
  a.x=b.m00*c.x+b.m01*c.y+b.m02*c.z;
  a.y=b.m10*c.x+b.m11*c.y+b.m12*c.z;
  a.z=b.m20*c.x+b.m21*c.y+b.m22*c.z;
  a.isZero = (a.x==0. && a.y==0. && a.z==0.);
  return a;
}
/// multiplication with a scalar
Matrix& operator*=(Matrix& a, double c) {
  a.m00*=c; a.m01*=c; a.m02*=c;
  a.m10*=c; a.m11*=c; a.m12*=c;
  a.m20*=c; a.m21*=c; a.m22*=c;
  return a;
}
/// multiplication with scalar
Matrix operator*(double b, const Matrix& c) {
  Matrix a;
  a=c;
  a*=b;
  return a;
}
/// sum of two matrices
Matrix& operator+=(Matrix& a, const Matrix& b) {
  a.m00+=b.m00; a.m01+=b.m01; a.m02+=b.m02;
  a.m10+=b.m10; a.m11+=b.m11; a.m12+=b.m12;
  a.m20+=b.m20; a.m21+=b.m21; a.m22+=b.m22;
  return a;
}

//==============================================================================

/// inverts the current rotation
Quaternion& Quaternion::invert() { w=-w; return *this; }

/// flips the sign of the quaterion -- which still represents the same rotation
void Quaternion::flipSign() { w=-w; x=-x; y=-y; z=-z; }

void Quaternion::uniqueSign() {
  if(w<0.) flipSign();
  else if(w==0.) {
    if(x<0.) flipSign();
    else if(x==0.) {
      if(y<0.) flipSign();
    }
  }
}

/// multiplies the rotation by a factor f (i.e., makes f-times the rotation)
void Quaternion::multiply(double f) {
  if(w<0.) flipSign();
  if(1.-w<1e-10 || f==1.) return;
  double phi=acos(w);
  phi*=f;
  w=cos(phi);
  f=sin(phi)/sqrt(x*x + y*y + z*z);
  x*=f; y*=f; z*=f;
}

double Quaternion::normalization() const {
  return w*w + x*x + y*y + z*z;
}

bool Quaternion::isNormalized() const {
  double n=w*w + x*x + y*y + z*z;
  return fabs(n-1.)<1e-6;
}

void Quaternion::normalize() {
  if(isZero) return;
  double n=w*w + x*x + y*y + z*z;
  n=sqrt(n);
  w/=n; x/=n; y/=n; z/=n;
}

/** @brief roughly, removes all ``components'' of the rotation that are not
    around the given vector v. More precisely, aligns/projects
    the rotation axis (given by q[1], q[2], q[3] of the quaternion)
    with v and re-normalizes afterwards. */
void Quaternion::alignWith(const Vector& v) {
  double s=x*v.x + y*v.y + z*v.z;
  if(!s) { setZero(); return; }  // are orthogonal
  s/=v*v;
  x=s*v.x; y=s*v.y; z=s*v.z;
  normalize();
}

void Quaternion::addX(double radians) {
  if(isZero) { setRadX(radians); return; }
  if(!radians) { return; }
  radians/=2.;
  double cw=cos(radians);
  double cx=sin(radians);

  Quaternion a;
  a.w = w*cw - x*cx;
  a.x = w*cx + x*cw;
  a.y = y*cw + z*cx;
  a.z = z*cw - y*cx;

  set(a.w, a.x, a.y, a.z);
}

void Quaternion::addY(double radians) {
  if(isZero) { setRadY(radians); return; }
  if(!radians) { return; }
  radians/=2.;
  double cw=cos(radians);
  double cy=sin(radians);

  Quaternion a;
  a.w = w*cw - y*cy;
  a.x = x*cw - z*cy;
  a.y = w*cy + y*cw;
  a.z = z*cw + x*cy;

  set(a.w, a.x, a.y, a.z);
}

Quaternion& Quaternion::addZ(double radians) {
  if(isZero) { setRadZ(radians); return *this; }
  if(!radians) { return *this; }
  radians/=2.;
  double cw=cos(radians);
  double cz=sin(radians);

  Quaternion a;
  a.w = w*cw - z*cz;
  a.x = x*cw + y*cz;
  a.y = y*cw - x*cz;
  a.z = w*cz + z*cw;

  set(a.w, a.x, a.y, a.z);
  return *this;
}

void Quaternion::append(const Quaternion& q) {
  if(q.isZero) return;
  double aw = w*q.w;
  double ax = x*q.w;
  double ay = y*q.w;
  double az = z*q.w;
  if(q.x) { aw -= x*q.x;  ax += w*q.x;  ay += z*q.x;  az -= y*q.x; }
  if(q.y) { aw -= y*q.y;  ax -= z*q.y;  ay += w*q.y;  az += x*q.y; }
  if(q.z) { aw -= z*q.z;  ax += y*q.z;  ay -= x*q.z;  az += w*q.z; }
  w=aw; x=ax; y=ay; z=az; isZero=false;
}

/// set the quad
void Quaternion::set(const double* p) { w=p[0]; x=p[1]; y=p[2]; z=p[3]; isZero=((w==1. || w==-1.) && x==0. && y==0. && z==0.); }

/// set the quad
void Quaternion::set(const arr& q) { CHECK_EQ(q.N, 4, "");  set(q.p); }

/// set the quad
void Quaternion::set(double _w, double _x, double _y, double _z) { w=_w; x=_x; y=_y; z=_z; isZero=((w==1. || w==-1.) && x==0. && y==0. && z==0.); }

/// reset the rotation to identity
void Quaternion::setZero() { memset(this, 0, sizeof(Quaternion));  w=1.; isZero=true; }

/// samples the rotation uniformly from the whole SO(3)
rai::Quaternion& Quaternion::setRandom() {
  arr q = randn(4);
  set(q);
  normalize();
  return *this;
}

/// sets this to a smooth interpolation between two rotations
void Quaternion::setInterpolate(double t, const Quaternion& a, const Quaternion b) {
  double sign=1.;
  if(quat_scalarProduct(a, b)<0) sign=-1.;
  w=a.w+t*(sign*b.w-a.w);
  x=a.x+t*(sign*b.x-a.x);
  y=a.y+t*(sign*b.y-a.y);
  z=a.z+t*(sign*b.z-a.z);
  normalize();
  isZero=false;
}

/// euclidean addition (with weights) modulated by scalar product -- leaves you with UNNORMALIZED quaternion
void Quaternion::add(const Quaternion b, double w_b, double w_this) {
  if(quat_scalarProduct(*this, b)<0.) w_b *= -1.;
  if(w_this!=-1.) {
    w *= w_this;
    x *= w_this;
    y *= w_this;
    z *= w_this;
  }
  w += w_b*b.w;
  x += w_b*b.x;
  y += w_b*b.y;
  z += w_b*b.z;
  isZero=false;
}

/// assigns the rotation to \c a DEGREES around the vector (x, y, z)
void Quaternion::setDeg(double degree, double _x, double _y, double _z) { setRad(degree*RAI_PI/180., _x, _y, _z); }

void Quaternion::setDeg(double degree, const Vector& vec) { setRad(degree*RAI_PI/180., vec.x, vec.y, vec.z); }

/// assigns the rotation to \c a RADIANTS (2*PI-units) around the vector (x, y, z)
void Quaternion::setRad(double angle, double _x, double _y, double _z) {
  if(!angle) { setZero(); return; }
  double l = _x*_x + _y*_y + _z*_z;
  if(l<1e-15) { setZero(); return; }
  angle/=2.;
  l=sin(angle)/sqrt(l);
  w=cos(angle);
  x=_x*l;
  y=_y*l;
  z=_z*l;
  isZero=false;
}

/// ..
void Quaternion::setRad(double angle, const Vector& axis) {
  setRad(angle, axis.x, axis.y, axis.z);
}

/// assigns the rotation to \c a RADIANTS (2*PI-units) around the current axis
void Quaternion::setRad(double angle) {
  if(!angle) { setZero(); return; }
  double l = x*x + y*y + z*z;
  if(l<1e-15) { setZero(); return; }
  angle/=2.;
  l=sin(angle)/sqrt(l);
  w=cos(angle);
  x*=l;
  y*=l;
  z*=l;
  isZero=false;
}

/// rotation around X-axis by given radiants
void Quaternion::setRadX(double radians) {
  if(!radians) { setZero(); return; }
  radians/=2.;
  w=cos(radians);
  x=sin(radians);
  y=z=0.;
  isZero=false;
}

/// rotation around Y-axis by given radiants
void Quaternion::setRadY(double radians) {
  if(!radians) { setZero(); return; }
  radians/=2.;
  w=cos(radians);
  y=sin(radians);
  x=z=0.;
  isZero=false;
}

/// rotation around Z-axis by given radiants
void Quaternion::setRadZ(double radians) {
  if(!radians) { setZero(); return; }
  radians/=2.;
  w=cos(radians);
  z=sin(radians);
  x=y=0.;
  isZero=false;
}

Quaternion& Quaternion::setRpy(double r, double p, double y) {
#if 1
  Quaternion q;
  setZero();
  q.setRadZ(y); *this = *this * q;
  q.setRadY(p); *this = *this * q;
  q.setRadX(r); *this = *this * q;
  return *this;
#else
  double cr=::cos(.5*r), sr=::sin(.5*r);
  double cp=::cos(.5*p), sp=::sin(.5*p);
  double cy=::cos(.5*y), sy=::sin(.5*y);
  w = cr*cp*cy + sr*sp*sy;
  x = sr*cp*cy - cr*sp*sy;
  y = cr*sp*cy + sr*cp*sy;
  z = cr*cp*sy - sr*sp*cy;
#endif
  isZero=(w==1. || w==-1.);
  CHECK(isNormalized(), "bad luck");
  return *this;
}

/// rotation around the given vector with angle (in rad) equal to norm of the vector
void Quaternion::setVec(Vector w) {
  double phi=w.length();
  setRad(phi, w.x, w.y, w.z);
}

/// rotation that will rotate 'from' to 'to' on direct path
void Quaternion::setDiff(const Vector& from, const Vector& to) {
  Vector a = from/from.length();
  Vector b = to/to.length();
  double scalarProduct = a*b;
  double phi=acos(scalarProduct);
  if(!phi) { setZero(); return; }
  Vector axis(a^b);
  if(axis.length()<1e-10) { //a and b are co-linear -> rotate around any! axis orthogonal to a or b
    axis = Vector_x^b; //try x
    if(axis.length()<1e-10) axis = Vector_y^b; //try y
  }
  setRad(phi, axis);
}

/// L1-norm to zero (i.e., identical rotation)
double Quaternion::diffZero() const { return (w>0.?fabs(w-1.):fabs(w+1.))+fabs(x)+fabs(y)+fabs(z); }

double Quaternion::sqrDiffZero() const { return (w>0.?rai::sqr(w-1.):rai::sqr(w+1.))+rai::sqr(x)+rai::sqr(y)+rai::sqr(z); }

/// check whether isZero is true
void Quaternion::checkZero() const {
  bool iszero = ((w==1. || w==-1.) && x==0. && y==0. && z==0.);
  if(isZero) if(!iszero) HALT("you must have set this by hand!");
}

/// return the squared-error between two quads, modulo flipping
double Quaternion::sqrDiff(const Quaternion& _q2) const {
  arr q1(&w, 4, true);
  arr q2(&_q2.w, 4, true);
  if(scalarProduct(q1, q2)>=0) return sumOfSqr(q1-q2);
  return sumOfSqr(q1+q2);
}

/// gets rotation angle (in rad [0, 2pi])
double Quaternion::getRad() const {
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) return 0;
  return 2.*acos(w);
}

/// gets rotation angle (in degree [0, 360])
double Quaternion::getDeg() const {
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) return 0;
  return 360./RAI_PI*acos(w);
}

/// gets rotation angle (in degree [0, 360]) and vector
void Quaternion::getDeg(double& degree, Vector& vec) const {
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) { degree=0.; vec.set(0., 0., 1.); return; }
  degree=acos(w);
  double s=sin(degree);
  degree*=360./RAI_PI;
  vec.x=x/s; vec.y=y/s; vec.z=z/s;
}

/// gets rotation angle (in rad [0, 2pi]) and vector
void Quaternion::getRad(double& angle, Vector& vec) const {
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) { angle=0.; vec.set(0., 0., 1.); return; }
  angle=acos(w);
  double s=1./sin(angle);
  angle*=2;
  vec.x=s*x; vec.y=s*y; vec.z=s*z;
  CHECK(angle>=0. && angle<=RAI_2PI, "");
}

/// gets the axis rotation vector with length equal to the rotation angle in rad
Vector Quaternion::getVec() const {
  Vector vec;
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) { vec.setZero(); return vec; }
  double phi, s;
  if(w>=0.) {
    phi=acos(w);
    s=2.*phi/sin(phi);
  } else { //flip quaternion sign to get rotation vector of length < PI
    phi=acos(-w);
    s=-2.*phi/sin(phi);
  }
  vec.x=s*x; vec.y=s*y; vec.z=s*z;
  return vec;
}

Vector Quaternion::getX() const {
  Vector Rx;
  double q22 = 2.*y*y;
  double q33 = 2.*z*z;
  double q12 = 2.*x*y;
  double q13 = 2.*x*z;
  double q02 = 2.*w*y;
  double q03 = 2.*w*z;
  Rx.x=1-q22-q33;
  Rx.y=q12+q03;
  Rx.z=q13-q02;
  return Rx;
}
Vector Quaternion::getY() const { return (*this)*Vector_y; }
Vector Quaternion::getZ() const { return (*this)*Vector_z; }

void Quaternion::setMatrix(double* m) {
  w = .5*sqrt(1.+m[0]+m[4]+m[8]); //sqrt(1.-(3.-(m[0]+m[4]+m[8]))/4.);
  z = (m[3]-m[1])/(4.*w);
  y = (m[2]-m[6])/(4.*w);
  x = (m[7]-m[5])/(4.*w);
  isZero=(w==1. || w==-1.);
  normalize();
  //CHECK(normalized(), "failed :-(");
}

/// exports the rotation to a double[9] matrix, row-by-row
Matrix Quaternion::getMatrix() const {
  Matrix R;
  double P1=2.*x, P2=2.*y, P3=2.*z;
  double q11 = x*P1;
  double q22 = y*P2;
  double q33 = z*P3;
  double q12 = x*P2;
  double q13 = x*P3;
  double q23 = y*P3;
  double q01 = w*P1;
  double q02 = w*P2;
  double q03 = w*P3;
  R.m00=1.-q22-q33; R.m01=q12-q03;     R.m02=q13+q02;
  R.m10=q12+q03;    R.m11=1.-q11-q33;  R.m12=q23-q01;
  R.m20=q13-q02;    R.m21=q23+q01;     R.m22=1.-q11-q22;
  return R;
}

arr Quaternion::getArr() const {
  arr R(3, 3);
  getMatrix(R.p);
  return R;
}

double* Quaternion::getMatrix(double* m) const {
  double P1=2.*x, P2=2.*y, P3=2.*z;
  double q11 = x*P1;
  double q22 = y*P2;
  double q33 = z*P3;
  double q12 = x*P2;
  double q13 = x*P3;
  double q23 = y*P3;
  double q01 = w*P1;
  double q02 = w*P2;
  double q03 = w*P3;
  m[0]=1.-q22-q33; m[1]=q12-q03;    m[2] =q13+q02;
  m[3]=q12+q03;    m[4]=1.-q11-q33; m[5] =q23-q01;
  m[6]=q13-q02;    m[7]=q23+q01;    m[8]=1.-q11-q22;
  return m;
}

/// exports the rotation to an ODE format matrix of type double[12]
double* Quaternion::getMatrixOde(double* m) const {
  double P1=2.*x, P2=2.*y, P3=2.*z;
  double q11 = x*P1;
  double q22 = y*P2;
  double q33 = z*P3;
  double q12 = x*P2;
  double q13 = x*P3;
  double q23 = y*P3;
  double q01 = w*P1;
  double q02 = w*P2;
  double q03 = w*P3;
  m[0]=1.-q22-q33; m[1]=q12-q03;    m[2] =q13+q02;
  m[4]=q12+q03;    m[5]=1.-q11-q33; m[6] =q23-q01;
  m[8]=q13-q02;    m[9]=q23+q01;    m[10]=1.-q11-q22;
  m[3]=m[7]=m[11]=0.;
  return m;
}

/// exports the rotation to an OpenGL format matrix of type double[16]
double* Quaternion::getMatrixGL(double* m) const {
  double P1=2.*x, P2=2.*y, P3=2.*z;
  double q11 = x*P1;
  double q22 = y*P2;
  double q33 = z*P3;
  double q12 = x*P2;
  double q13 = x*P3;
  double q23 = y*P3;
  double q01 = w*P1;
  double q02 = w*P2;
  double q03 = w*P3;
  m[0]=1.-q22-q33; m[4]=q12-q03;    m[8] =q13+q02;
  m[1]=q12+q03;    m[5]=1.-q11-q33; m[9] =q23-q01;
  m[2]=q13-q02;    m[6]=q23+q01;    m[10]=1.-q11-q22;
  m[3]=m[7]=m[11]=m[12]=m[13]=m[14]=0.;
  m[15]=1.;
  return m;
}

double Quaternion::getRoll_X() const {
  double sinr = +2.0 * (w * x + y * z);
  double cosr = +1.0 - 2.0 * (x * x + y * y);
  return atan2(sinr, cosr);
}

double Quaternion::getPitch_Y() const {
  double sinp = +2.0 * (w * y - z * x);
  if(fabs(sinp) >= 1)
    return copysign(RAI_PI / 2, sinp); // use 90 degrees if out of range
  return asin(sinp);
}

double Quaternion::getYaw_Z() const {
  double siny = +2.0 * (w * z + x * y);
  double cosy = +1.0 - 2.0 * (y * y + z * z);
  return atan2(siny, cosy);
}

arr Quaternion::getEulerRPY() const {
  return {getRoll_X(), getPitch_Y(), getYaw_Z()};
}

void Quaternion::applyOnPointArray(arr& pts) const {
  arr R = ~getArr(); //transposed, to make it applicable to an n-times-3 array
  pts = pts * R;
}

/// this is a 3-by-4 matrix $J$, giving the angular velocity vector $w = J \dot q$  induced by a $\dot q$
arr Quaternion::getJacobian() const {
  arr J(3, 4);
  rai::Quaternion e;
  for(uint i=0; i<4; i++) {
    if(i==0) e.set(1., 0., 0., 0.);
    if(i==1) e.set(0., 1., 0., 0.);
    if(i==2) e.set(0., 0., 1., 0.);
    if(i==3) e.set(0., 0., 0., 1.); //TODO: the following could be simplified/compressed/made more efficient
    e = e / *this;
    J(0, i) = -2.*e.x;
    J(1, i) = -2.*e.y;
    J(2, i) = -2.*e.z;
  }
  return J;
}

/// this is a 4x(3x3) matrix, such that ~(J*x) is the jacobian of (R*x), and ~qdelta*J is (del R/del q)(qdelta)
arr Quaternion::getMatrixJacobian() const {
  arr J(4, 9); //transpose!
  double r0=w, r1=x, r2=y, r3=z;
  J[0] = {      0,    -r3,     r2,
                r3,      0,    -r1,
                -r2,     r1,      0
         };
  J[1] = {      0,     r2,     r3,
                r2, -2.*r1,    -r0,
                r3,     r0, -2.*r1
         };
  J[2] = { -2.*r2,     r1,    r0,
           r1,      0,     r3,
           -r0,     r3, -2.*r2
         };
  J[3] = { -2.*r3,    -r0,     r1,
           r0, -2.*r3,     r2,
           r1,     r2,      0
         };
  J *= 2.;
  J.reshape(4, 3, 3);
  return J;
}

arr Quaternion::getQuaternionMultiplicationMatrix() const {
//  a.w = b.w*c.w - b.x*c.x - b.y*c.y - b.z*c.z;
//  a.x = b.w*c.x + b.x*c.w + b.y*c.z - b.z*c.y;
//  a.y = b.w*c.y - b.x*c.z + b.y*c.w + b.z*c.x;
//  a.z = b.w*c.z + b.x*c.y - b.y*c.x + b.z*c.w;
  return arr(
  {4, 4}, {
    +w, -x, -y, -z,
      +x, +w, +z, -y,
      +y, -z, +w, +x,
      +z, +y, -x, +w
    });
}

void Quaternion::writeNice(std::ostream& os) const { os <<"Quaternion: " <<getDeg() <<" around " <<getVec() <<"\n"; }
void Quaternion::write(std::ostream& os) const {
  os <<'[' <<w <<", " <<x <<", " <<y <<", " <<z <<']';
}
void Quaternion::read(std::istream& is) {
  is >>PARSE("[") >>w >>x >>y  >>z >>PARSE("]");
  normalize();
}
//}

/// inverse rotation
Quaternion operator-(const Quaternion& b) {
  return Quaternion(b).invert();
}

/// compound of two rotations (A=B*C)
Quaternion operator*(const Quaternion& b, const Quaternion& c) {
  if(c.isZero) return b;
  if(b.isZero) return c;
  Quaternion a;
#if 0
  a.w = b.w*c.w - b.x*c.x - b.y*c.y - b.z*c.z;
  a.x = b.w*c.x + b.x*c.w + b.y*c.z - b.z*c.y;
  a.y = b.w*c.y + b.y*c.w + b.z*c.x - b.x*c.z;
  a.z = b.w*c.z + b.z*c.w + b.x*c.y - b.y*c.x;
#else
  a.w = b.w*c.w;
  a.x = b.x*c.w;
  a.y = b.y*c.w;
  a.z = b.z*c.w;
  if(c.x) { a.w -= b.x*c.x;  a.x += b.w*c.x;  a.y += b.z*c.x;  a.z -= b.y*c.x; }
  if(c.y) { a.w -= b.y*c.y;  a.x -= b.z*c.y;  a.y += b.w*c.y;  a.z += b.x*c.y; }
  if(c.z) { a.w -= b.z*c.z;  a.x += b.y*c.z;  a.y -= b.x*c.z;  a.z += b.w*c.z; }
#endif
  a.isZero=(a.w==1. || a.w==-1.);
  return a;
}

/// A=B*C^{-1}
Quaternion operator/(const Quaternion& b, const Quaternion& c) {
  // same as b * (-c), where c is just inverted (c.w \gets - c.w)
  Quaternion a;
  a.w =-b.w*c.w - b.x*c.x - b.y*c.y - b.z*c.z;
  a.x = b.w*c.x - b.x*c.w + b.y*c.z - b.z*c.y;
  a.y = b.w*c.y - b.y*c.w + b.z*c.x - b.x*c.z;
  a.z = b.w*c.z - b.z*c.w + b.x*c.y - b.y*c.x;
  a.isZero=(a.w==1. || a.w==-1.);
  return a;
}

/// Euclidean(!) difference between two quaternions
Quaternion operator-(const Quaternion& b, const Quaternion& c) {
  HALT("don't use that..?");
  Quaternion a;
  a.w = b.w-c.w;
  a.x = b.x-c.x;
  a.y = b.y-c.y;
  a.z = b.z-c.z;
  a.isZero = false;
  return a;
}

Quaternion operator*=(Quaternion& a, double s) {
  a.multiply(s);
  return a;
}

void mult(Vector& a, const Quaternion& b, const Vector& c, bool add) {
  if(c.isZero) {
    if(!add) a.setZero();
    return;
  }
  if(b.isZero) {
    a=c;
    return;
  }
  double Bx=2.*b.x, By=2.*b.y, Bz=2.*b.z;
  double q11 = b.x*Bx;
  double q22 = b.y*By;
  double q33 = b.z*Bz;
  double q12 = b.x*By;
  double q13 = b.x*Bz;
  double q23 = b.y*Bz;
  double q01 = b.w*Bx;
  double q02 = b.w*By;
  double q03 = b.w*Bz;
  if(!add) a.x=a.y=a.z=0.;
  if(c.x) { a.x += (1.-q22-q33)*c.x; a.y += (q12+q03)*c.x; a.z += (q13-q02)*c.x; }
  if(c.y) { a.x += (q12-q03)*c.y; a.y += (1.-q11-q33)*c.y; a.z += (q23+q01)*c.y; }
  if(c.z) { a.x += (q13+q02)*c.z; a.y += (q23-q01)*c.z; a.z += (1.-q11-q22)*c.z; }
  a.isZero = false;
}

/// transform of a vector by a rotation
Vector operator*(const Quaternion& b, const Vector& c) {
  if(c.isZero) return Vector(0);
  Vector a;
  mult(a, b, c, false);
  return a;
}

/// inverse transform of a vector by a rotation
Vector operator/(const Vector& c, const Quaternion& b) {
  Matrix M = b.getMatrix();
  Vector a;
  a.x = M.m00*c.x + M.m10*c.y + M.m20*c.z;
  a.y = M.m01*c.x + M.m11*c.y + M.m21*c.z;
  a.z = M.m02*c.x + M.m12*c.y + M.m22*c.z;
  a.isZero = (a.x==0. && a.y==0. && a.z==0.);
  return a;
}

Transformation operator-(const Transformation& X) {
  Transformation Y;
  Y.setInverse(X);
  return Y;
}

Transformation operator*(const Transformation& X, const Transformation& c) {
  Transformation f(X);
  f.appendTransformation(c);
  return f;
}

Transformation operator/(const Transformation& to, const Transformation& from) {
  // same as (-from) * to
  Transformation f;
  f.setDifference(from, to);
  return f;
}

/// transform of a vector by a frame
Vector operator*(const Transformation& X, const Vector& c) {
  Vector a;
  a = X.rot * c;
  a += X.pos;
  return a;
}

/// inverse transform of a vector by a frame
Vector operator/(const Vector& c, const Transformation& X) {
  Vector a(c);
  a -= X.pos;
  a = a / X.rot;
  return a;
}

/// use as similarity measure (distance = 1 - |scalarprod|)
double quat_scalarProduct(const Quaternion& a, const Quaternion& b) {
  return a.w*b.w+a.x*b.x+a.y*b.y+a.z*b.z;
}

double quat_sqrDistance(const Quaternion& a, const Quaternion& b) {
  double sign=1.;
  if(quat_scalarProduct(a, b)<0) sign=-1.;
  double w = (sign*b.w-a.w);
  double x = (sign*b.x-a.x);
  double y = (sign*b.y-a.y);
  double z = (sign*b.z-a.z);
  return w*w + x*x + y*y + z*z;
}

void quat_concat(arr& y, arr& Ja, arr& Jb, const arr& A, const arr& B) {
  rai::Quaternion a(A);
  rai::Quaternion b(B);
  a.isZero=b.isZero=false;
  y = (a * b).getArr4d();
  if(!!Ja) {
    Ja.resize(4, 4);
    Ja(0, 0) =  b.w;
    Ja(0, 1) = -b.x; Ja(0, 2) = -b.y; Ja(0, 3) = -b.z;
    Ja(1, 0) =  b.x; Ja(2, 0) =  b.y; Ja(3, 0) =  b.z;
    //skew
    Ja(1, 1) = b.w; Ja(1, 2) = b.z; Ja(1, 3) =-b.y;
    Ja(2, 1) =-b.z; Ja(2, 2) = b.w; Ja(2, 3) = b.x;
    Ja(3, 1) = b.y; Ja(3, 2) =-b.x; Ja(3, 3) = b.w;
  }
  if(!!Jb) {
    Jb.resize(4, 4);
    Jb(0, 0) =  a.w;
    Jb(0, 1) = -a.x; Jb(0, 2) = -a.y; Jb(0, 3) = -a.z;
    Jb(1, 0) =  a.x; Jb(2, 0) =  a.y; Jb(3, 0) =  a.z;
    //skew
    Jb(1, 1) = a.w; Jb(1, 2) =-a.z; Jb(1, 3) = a.y;
    Jb(2, 1) = a.z; Jb(2, 2) = a.w; Jb(2, 3) =-a.x;
    Jb(3, 1) =-a.y; Jb(3, 2) = a.x; Jb(3, 3) = a.w;
  }
}

void quat_normalize(arr& y, arr& J, const arr& a) {
  y = a;
  double l2 = sumOfSqr(y);
  double l = sqrt(l2);
  y /= l;
  if(!!J) {
    J = eye(4);
    J -= y^y;
    J /= l;
  }
}

void quat_getVec(arr& y, arr& J, const arr& A) {
  rai::Quaternion a(A);
  y.resize(3);
  double phi, sinphi, s;
  double dphi, dsinphi, ds=0.;
  if(a.w>=1. || a.w<=-1. || (a.x==0. && a.y==0. && a.z==0.)) {
    y.setZero();
    if(!!J) {
      J.resize(3, 4).setZero();
      J(0, 1) = J(1, 2) = J(2, 3) = 2.;
    }
    return;
  }

  if(false && a.w>=0.) {
    phi=acos(a.w);
    sinphi = sin(phi);
    s=2.*phi/sinphi;
    if(!!J) {
      dphi = -1./sqrt(1.-a.w*a.w);
      dsinphi = cos(phi) * dphi;
      ds = 2.*(dphi/sinphi - phi/(sinphi*sinphi)*dsinphi);
    }
  } else { //flip quaternion sign to get rotation vector of length < PI
    phi=acos(-a.w);
    sinphi = sin(phi);
    s=-2.*phi/sinphi;
    if(!!J) {
      dphi = 1./sqrt(1.-a.w*a.w);
      dsinphi = cos(phi) * dphi;
      ds = -2.*(dphi/sinphi - phi/(sinphi*sinphi)*dsinphi);
    }
  }
  if(fabs(phi)<1e-8) { s=2.; ds=0.; }
  y(0) = s*a.x;
  y(1) = s*a.y;
  y(2) = s*a.z;
  if(!!J) {
    J.resize(3, 4).setZero();
    J(0, 1) = J(1, 2) = J(2, 3) = s;
    J(0, 0) = a.x*ds;
    J(1, 0) = a.y*ds;
    J(2, 0) = a.z*ds;
  }
}

void quat_diffVector(arr& y, arr& Ja, arr& Jb, const arr& a, const arr& b) {
  arr ab, Jca, Jcb;
  arr binv = b;
  binv(0) *= -1.;
  quat_concat(ab, Jca, Jcb, a, binv);
  for(uint i=0; i<Jcb.d0; i++) Jcb(i, 0) *= -1.;

  arr Jvec;
  quat_getVec(y, Jvec, ab);
  Ja = Jvec * Jca;
  Jb = Jvec * Jcb;
}

//==============================================================================

/// initialize by reading from the string
Transformation& Transformation::setText(const char* txt) { read(rai::String(txt).stream()); return *this; }

bool Transformation::operator!() const { return this==&NoTransformation; }

/// resets the position to origin, rotation to identity, velocities to zero, scale to unit
Transformation& Transformation::setZero() {
  memset(this, 0, sizeof(Transformation));
  rot.w = 1.;
  pos.isZero = rot.isZero = true;
  return *this;
}

void Transformation::set(const double* p) { pos.set(p); rot.set(p+3); }

void Transformation::set(const arr& t) {
  if(t.N==7) set(t.p);
  else if(t.N==3) { pos.set(t.p); rot.setZero(); }
  else if(t.N==4) { pos.setZero(); rot.set(t.p); }
  else HALT("transformation can be assigned only to a 7D, 3D, or 4D array");
}

/// randomize the frame
Transformation& Transformation::setRandom() {
  rot.setRandom();
  pos.setRandom();
  return *this;
}

/// move the turtle by the vector (x, z, y) WITH RESPECT TO the current orientation/scale
Transformation& Transformation::addRelativeTranslation(double x, double y, double z) {
  addRelativeTranslation(Vector(x, y, z));
  return *this;
}

Transformation& Transformation::addRelativeTranslation(const Vector& x_rel) {
  pos += rot*x_rel;
  return *this;
}

/// rotate the turtle orientation
void Transformation::addRelativeRotation(const Quaternion& q) {
  rot = rot*q;
}

/// rotate the turtle orientation by an angle (given in DEGREE) around the vector (x, y, z) (given relative to the current orientation)
void Transformation::addRelativeRotationDeg(double degree, double x, double y, double z) {
  Quaternion R;
  R.setDeg(degree, x, y, z);
  rot = rot*R;
}

/// rotate the turtle orientation by an angle (given in radiants) around the vector (x, y, z) (given relative to the current orientation)
void Transformation::addRelativeRotationRad(double rad, double x, double y, double z) {
  Quaternion R;
  R.setRad(rad, x, y, z);
  rot = rot*R;
}

/// rotate the turtle orientation as given by a quaternion
void Transformation::addRelativeRotationQuat(double w, double x, double y, double z) {
  Quaternion R(w, x, y, z);
  rot = rot*R;
}

/** @brief transform the turtle by f,
    which is interpreted RELATIVE to the current frame
    (new = old * f) */
void Transformation::appendTransformation(const Transformation& f) {
  //below is same as ``pos += rot*f.pos;  rot = rot*f.rot;''
  if(!f.pos.isZero) {
    if(rot.isZero) pos += f.pos;
    else mult(pos, rot, f.pos, true);
  }
  if(!f.rot.isZero) {
    if(rot.isZero) rot = f.rot;
    else rot.append(f.rot);
  }
}

/// inverse transform (new = old * f^{-1})
void Transformation::appendInvTransformation(const Transformation& f) {
  if(!f.rot.isZero) {
    rot = rot/f.rot;
  }
  if(!f.pos.isZero) {
    pos -= rot*f.pos;
  }
}

/// this = f^{-1}
void Transformation::setInverse(const Transformation& f) {
  rot = -f.rot;
  pos = - (rot * f.pos);
}

/// set double[4*4] to Transformation. Matrix needs to be orthogonal
void Transformation::setAffineMatrix(const double* m) {
  double M[9];
  uint i, j;
  for(i=0; i<3; ++i)
    for(j=0; j<3; ++j)
      M[i*3+j] = m[i*4+j];
  rot.setMatrix(M);                 // set 3x3 submatrix as rotation
  pos.x=m[3];  // set last column as translation
  pos.y=m[7];  // set last column as translation
  pos.z=m[11];  // set last column as translation
}

///  to = new * from
void Transformation::setDifference(const Transformation& from, const Transformation& to) {
  // same as (-from) * to
  rot = (-from.rot) * to.rot;
  pos = (-from.rot) * (to.pos-from.pos);
  rot.normalize();
}

void Transformation::setInterpolate(double t, const Transformation& a, const Transformation b) {
  pos = (1.-t)*a.pos + t*b.pos;
  rot.setInterpolate(t, a.rot, b.rot);
}

/// get the current position/orientation/scale in an OpenGL format matrix (of type double[16])
double* Transformation::getAffineMatrix(double* m) const {
  Matrix M = rot.getMatrix();
  m[0] = M.m00; m[1] = M.m01; m[2] = M.m02; m[3] =pos.x;
  m[4] = M.m10; m[5] = M.m11; m[6] = M.m12; m[7] =pos.y;
  m[8] = M.m20; m[9] = M.m21; m[10]= M.m22; m[11]=pos.z;
  m[12]= 0.;    m[13]= 0.;    m[14]= 0.;    m[15]=1.;
  return m;
}

arr Transformation::getAffineMatrix() const {
  arr T(4, 4);
  getAffineMatrix(T.p);
  return T;
}

/// get inverse OpenGL matrix for this frame (of type double[16])
double* Transformation::getInverseAffineMatrix(double* m) const {
  Matrix M = rot.getMatrix();
  Vector pinv; pinv=pos/rot;
  m[0] =M.m00; m[1] =M.m10; m[2] =M.m20; m[3] =-pinv.x;
  m[4] =M.m01; m[5] =M.m11; m[6] =M.m21; m[7] =-pinv.y;
  m[8] =M.m02; m[9] =M.m12; m[10]=M.m22; m[11]=-pinv.z;
  m[12]=0.;    m[13]=0.;    m[14]=0.;    m[15]=1.;
  return m;
}

arr Transformation::getInverseAffineMatrix() const {
  arr T(4, 4);
  getInverseAffineMatrix(T.p);
  return T;
}

/// get the current position/orientation/scale in an OpenGL format matrix (of type double[16])
double* Transformation::getAffineMatrixGL(double* m) const {
  Matrix M = rot.getMatrix();
  m[0]=M.m00; m[4]=M.m01; m[8] =M.m02; m[12]=pos.x;
  m[1]=M.m10; m[5]=M.m11; m[9] =M.m12; m[13]=pos.y;
  m[2]=M.m20; m[6]=M.m21; m[10]=M.m22; m[14]=pos.z;
  m[3]=0.;   m[7]=0.;   m[11]=0.;   m[15]=1.;
  return m;
}

/// get inverse OpenGL matrix for this frame (of type double[16]) */
double* Transformation::getInverseAffineMatrixGL(double* m) const {
  Matrix M = rot.getMatrix();
  Vector pinv; pinv=pos/rot;
  m[0]=M.m00; m[4]=M.m10; m[8] =M.m20; m[12]=-pinv.x;
  m[1]=M.m01; m[5]=M.m11; m[9] =M.m21; m[13]=-pinv.y;
  m[2]=M.m02; m[6]=M.m12; m[10]=M.m22; m[14]=-pinv.z;
  m[3]=0.;   m[7]=0.;   m[11]=0.;   m[15]=1.;
  return m;
}

arr Transformation::getArr7d() const {
  arr t(7);
  t.p[0]=pos.x;
  t.p[1]=pos.y;
  t.p[2]=pos.z;
  t.p[3]=rot.w;
  t.p[4]=rot.x;
  t.p[5]=rot.y;
  t.p[6]=rot.z;
  return t;
}

arr Transformation::getWrenchTransform() const {
  arr z(3, 3);  z.setZero();
  arr r = skew(pos.getArr()); //(3, 3);  Featherstone::skew(r, &pos.x); skew pos
  arr R = rot.getArr(); //(3, 3);  rot.getMatrix(R.p);
  transpose(R);
  arr X(6, 6);
  X.setMatrixBlock(R, 0, 0);
  X.setMatrixBlock(z, 0, 3);
  X.setMatrixBlock(R*~r, 3, 0);
  X.setMatrixBlock(R, 3, 3); //[[unklar!!]]
  return X;
  //cout <<"\nz=" <<z <<"\nr=" <<r <<"\nR=" <<R <<"\nX=" <<X <<endl;
}

void Transformation::applyOnPoint(arr& pt) const {
  CHECK_EQ(pt.N, 3, "");
  if(!rot.isZero) pt = rot.getArr() * pt;
  if(!pos.isZero) pt += pos.getArr();
}

arr& Transformation::applyOnPointArray(arr& pts) const {
  if(!((pts.nd==2 && pts.d1==3) || (pts.nd==3 && pts.d2==3))) {
    LOG(-1) <<"wrong pts dimensions for transformation:" <<pts.dim();
    return pts;
  }
  if(!rot.isZero) {
    arr R = ~rot.getArr(); //transposed, only to make it applicable to an n-times-3 array
    pts = pts * R;
  }
  if(!pos.isZero) {
    for(double* p=pts.p, *pstop=pts.p+pts.N; p<pstop; p+=3) {
      p[0] += pos.x;
      p[1] += pos.y;
      p[2] += pos.z;
    }
  }
  return pts;
}

bool Transformation::isZero() const {
  return pos.isZero && rot.isZero;
}

/// L1-norm to zero
double Transformation::diffZero() const {
  return pos.diffZero() + rot.diffZero();
}

void Transformation::checkNan() const {
  CHECK_EQ(pos.x, pos.x, "inconsistent: " <<pos.x);
  CHECK_EQ(pos.y, pos.y, "inconsistent: " <<pos.y);
  CHECK_EQ(pos.z, pos.z, "inconsistent: " <<pos.z);
  CHECK_EQ(rot.x, rot.x, "inconsistent: " <<rot.x);
  CHECK_EQ(rot.w, rot.w, "inconsistent: " <<rot.w);
  CHECK_EQ(rot.x, rot.x, "inconsistent: " <<rot.x);
  CHECK_EQ(rot.y, rot.y, "inconsistent: " <<rot.y);
  CHECK_EQ(rot.z, rot.z, "inconsistent: " <<rot.z);
}

/// operator<<
void Transformation::write(std::ostream& os) const {
  if(rot.isZero) {
    os <<'[' <<pos.x <<", " <<pos.y <<", " <<pos.z <<']';
  } else {
    os <<'[' <<pos.x <<", " <<pos.y <<", " <<pos.z <<", "
       <<rot.w <<", " <<rot.x <<", " <<rot.y <<", " <<rot.z <<']';
  }
}

/// operator>>
void Transformation::read(std::istream& is) {
  setZero();
  char c;
  double x[7];
  rai::skip(is, " \n\r\t<|");
  for(;;) {
    is >>c;
    if(is.fail()) return;  //EOF I guess
    if((c>='0' && c<='9') || c=='.' || c=='-' || c=='[') { //read a 7-vector (pos+quat) for the transformation
      if(c=='[') {
        is>>x[0]>>PARSE(",") >>x[1]>>PARSE(",") >>x[2]>>PARSE(",") >>x[3]>>PARSE(",") >>x[4]>>PARSE(",") >>x[5]>>PARSE(",") >>x[6] >>PARSE("]");
      } else {
        is.putback(c);
        is>>x[0]>>x[1]>>x[2]>>x[3]>>x[4]>>x[5]>>x[6];
      }
      addRelativeTranslation(x[0], x[1], x[2]);
      addRelativeRotationQuat(x[3], x[4], x[5], x[6]);
      break;
    } else switch(c) {
        //case '<': break; //do nothing -- assume this is an opening tag
        case 't': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>PARSE(")");       addRelativeTranslation(x[0], x[1], x[2]); break;
        case 'q': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>x[3]>>PARSE(")"); addRelativeRotationQuat(x[0], x[1], x[2], x[3]); break;
        case 'r': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>x[3]>>PARSE(")"); addRelativeRotationRad(x[0], x[1], x[2], x[3]); break;
        case 'd': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>x[3]>>PARSE(")"); addRelativeRotationDeg(x[0], x[1], x[2], x[3]); break;
        case 'E': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>PARSE(")"); addRelativeRotation(Quaternion().setRpy(x[0], x[1], x[2])); break;
        case 'p': {
          is>>PARSE("(")>>x[0]>>x[1]>>x[2];       addRelativeTranslation(x[0], x[1], x[2]);
          is>>x[0]>>x[1]>>x[2]>>x[3]>>PARSE(")"); addRelativeRotationQuat(x[0], x[1], x[2], x[3]);
        } break;
        //case 's': is>>PARSE("(")>>x[0]>>PARSE(")");                   scale(x[0]); break;
        case 'T': break; //old convention
        case '|':
        case '>': is.putback(c); return; //those symbols finish the reading without error
        default: {
          RAI_MSG("unknown Transformation read tag: '" <<c <<"' abort reading this frame"); is.putback(c); return;
        }
      }
    if(is.fail()) HALT("error reading '" <<c <<"' parameters in Transformation");
  }
  if(is.fail()) HALT("could not read Transformation struct");
  rot.normalize();
}

//==============================================================================

/// initialize by reading from the string
DynamicTransformation& DynamicTransformation::setText(const char* txt) { read(rai::String(txt)()); return *this; }

/// resets the position to origin, rotation to identity, velocities to zero, scale to unit
DynamicTransformation::DynamicTransformation(const char* init) { read(rai::String(init).stream()); }

DynamicTransformation& DynamicTransformation::setZero() {
  memset(this, 0, sizeof(DynamicTransformation));
  rot.w = 1.;
  pos.isZero = rot.isZero = vel.isZero = angvel.isZero = true;
  zeroVels = true;
  return *this;
}

/// randomize the frame
void DynamicTransformation::setRandom() {
  rot.setRandom();
  pos.setRandom();
  if(rnd.uni()<.8) {
    vel.setZero(); angvel.setZero(); zeroVels=true;
  } else {
    vel.setRandom(); angvel.setRandom(); zeroVels = false;
  }
}

/// move the turtle by the vector (x, z, y) WITH RESPECT TO the current orientation/scale
void DynamicTransformation::addRelativeTranslation(double x, double y, double z) {
  addRelativeTranslation(Vector(x, y, z));
}

void DynamicTransformation::addRelativeTranslation(const Vector& x_rel) {
  Vector x = rot*x_rel;
  pos+=x;
  if(!zeroVels) vel+=angvel^x;
}

/// add a velocity to the turtle's inertial frame
void DynamicTransformation::addRelativeVelocity(double x, double y, double z) {
  Vector X(x, y, z);
  //v+=r*(s*X);
  vel+=rot*X;
  zeroVels = false;
}

/// add an angular velocity to the turtle inertial frame
void DynamicTransformation::addRelativeAngVelocityDeg(double degree, double x, double y, double z) {
  Vector W(x, y, z); W.normalize();
  W*=degree*RAI_PI/180.;
  angvel+=rot*W;
  zeroVels = false;
}

/// add an angular velocity to the turtle inertial frame
void DynamicTransformation::addRelativeAngVelocityRad(double rad, double x, double y, double z) {
  Vector W(x, y, z); W.normalize();
  W*=rad;
  angvel+=rot*W;
  zeroVels = false;
}

/// add an angular velocity to the turtle inertial frame
void DynamicTransformation::addRelativeAngVelocityRad(double wx, double wy, double wz) {
  Vector W(wx, wy, wz);
  angvel+=rot*W;
  zeroVels = false;
}

/** @brief transform the turtle into the frame f,
    which is interpreted RELATIVE to the current frame
    (new = f * old) */
void DynamicTransformation::appendTransformation(const DynamicTransformation& f) {
  if(zeroVels && f.zeroVels) {
    if(!f.pos.isZero) { if(rot.isZero) pos += f.pos; else pos += rot*f.pos; }
    if(!f.rot.isZero) { if(rot.isZero) rot = f.rot; else rot = rot*f.rot; }
  } else {
    //Vector P(r*(s*f.p)); //relative offset in global coords
    //Vector V(r*(s*f.v)); //relative vel in global coords
    Matrix R = rot.getMatrix();
    Vector P(R*f.pos); //relative offset in global coords
    Vector V(R*f.vel); //relative vel in global coords
    Vector W(R*f.angvel); //relative ang vel in global coords
    pos += P;
    vel += angvel^P;
    vel += V;
    //a += b^P;
    //a += w^((w^P) + 2.*V);
    //a += r*(s*f.a);
    //b += w^W;
    //b += r*f.b;
    angvel += W;
    rot = rot*f.rot;
    //s*=f.s;
    zeroVels = false;
  }
}

/// inverse transform (new = f^{-1} * old) or (old = f * new)
void DynamicTransformation::appendInvTransformation(const DynamicTransformation& f) {
  if(zeroVels && f.zeroVels) {
    rot = rot/f.rot;
    pos -= rot*f.pos;
  } else {
    rot=rot/f.rot;
    Matrix R = rot.getMatrix();
    Vector P(R*f.pos);
    angvel -= R*f.angvel;
    vel -= R*f.vel;
    vel -= angvel^P;
    pos -= P;
    zeroVels = false;
  }
}

/// this = f^{-1}
void DynamicTransformation::setInverse(const DynamicTransformation& f) {
  if(f.zeroVels) {
    rot = -f.rot;
    pos = - (rot * f.pos);
    vel.setZero();
    angvel.setZero();
    zeroVels = true;
  } else {
    rot = -f.rot;
    Matrix R = rot.getMatrix();
    pos = - (R * f.pos);
    vel = R * ((f.angvel^f.pos) - f.vel);
    angvel = - (R * f.angvel);
    zeroVels = false;
  }
}

/// set double[4*4] to Transformation. Matrix needs to be orthogonal
void DynamicTransformation::setAffineMatrix(const double* m) {
  double M[9];
  uint i, j;
  for(i=0; i<3; ++i)
    for(j=0; j<3; ++j)
      M[i*3+j] = m[i*4+j];
  rot.setMatrix(M);                 // set 3x3 submatrix as rotation
  pos.x=m[3];  // set last column as translation
  pos.y=m[7];  // set last column as translation
  pos.z=m[11];  // set last column as translation
  zeroVels=true;
}

///  to = new * from
void DynamicTransformation::setDifference(const DynamicTransformation& from, const DynamicTransformation& to) {
  if(from.zeroVels && to.zeroVels) {
    rot = Quaternion_Id / from.rot * to.rot;
    pos = (-from.rot) * (to.pos-from.pos);
    zeroVels = true;
  } else {
    rot = Quaternion_Id / from.rot * to.rot;
    angvel = (-from.rot) * (to.angvel-from.angvel);
    vel = (-from.rot) * (to.vel-from.vel);
    vel-= (-from.rot) * (from.angvel^(to.pos-from.pos));
    pos = (-from.rot) * (to.pos-from.pos);
    zeroVels = false;
  }
}

bool DynamicTransformation::isZero() const {
  return pos.isZero && rot.isZero && vel.isZero && angvel.isZero;
}

/// L1-norm to zero
double DynamicTransformation::diffZero() const {
  return pos.diffZero() + rot.diffZero() + vel.diffZero() + angvel.diffZero();
}

/// operator<<
void DynamicTransformation::write(std::ostream& os) const {
  os <<pos.x <<", " <<pos.y <<", " <<pos.z <<", "
     <<rot.w <<", " <<rot.x <<", " <<rot.y <<", " <<rot.z;
  if(!zeroVels) {
    os <<" v" <<vel <<" w" <<angvel;
  }
}

/// operator>>
void DynamicTransformation::read(std::istream& is) {
  setZero();
  char c;
  double x[4];
  rai::skip(is, " \n\r\t<|");
  for(;;) {
    is >>c;
    if(is.fail()) return;  //EOF I guess
    //if(c==';') break;
    //if(c==',') is >>c;
    if((c>='0' && c<='9') || c=='.' || c=='-') {  //read a 7-vector (pos+quat) for the transformation
      is.putback(c);
      is>>x[0]>>x[1]>>x[2];       addRelativeTranslation(x[0], x[1], x[2]);
      is>>x[0]>>x[1]>>x[2]>>x[3]; addRelativeRotationQuat(x[0], x[1], x[2], x[3]);
    } else switch(c) {
        //case '<': break; //do nothing -- assume this is an opening tag
        case 't': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>PARSE(")");       addRelativeTranslation(x[0], x[1], x[2]); break;
        case 'q': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>x[3]>>PARSE(")"); addRelativeRotationQuat(x[0], x[1], x[2], x[3]); break;
        case 'r': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>x[3]>>PARSE(")"); addRelativeRotationRad(x[0], x[1], x[2], x[3]); break;
        case 'd': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>x[3]>>PARSE(")"); addRelativeRotationDeg(x[0], x[1], x[2], x[3]); break;
        case 'E': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>PARSE(")"); addRelativeRotation(Quaternion().setRpy(x[0], x[1], x[2])); break;
        case 'v': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>PARSE(")");       addRelativeVelocity(x[0], x[1], x[2]); break;
        case 'w': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>PARSE(")");       addRelativeAngVelocityRad(x[0], x[1], x[2]); break;
        //case 's': is>>PARSE("(")>>x[0]>>PARSE(")");                   scale(x[0]); break;
        case '|':
        case '>': is.putback(c); return; //those symbols finish the reading without error
        default: RAI_MSG("unknown DynamicTransformation read tag: " <<c <<"abort reading this frame"); is.putback(c); return;
      }
    if(is.fail()) HALT("error reading '" <<c <<"' parameters in frame");
  }
  if(is.fail()) HALT("could not read DynamicTransformation struct");
  zeroVels = vel.isZero && angvel.isZero;
}

//===========================================================================
//
// camera class
//

/** @brief constructor; specify a frame if the camera is to be attached
   to an existing frame. Otherwise the camera creates its own
      frame */
Camera::Camera() {
  setZero();

  setPosition(0., 0., 10.);
  focus(0., 0., 0.);
  setHeightAngle(45.);
}

void Camera::setZero() {
  X.setZero();
  foc.setZero();
  setHeightAngle(45.);
  setWHRatio(1.);
  setZRange(.02, 200.);
}

/// the height angle (in degrees) of the camera perspective; set it 0 for orthogonal projection
void Camera::setHeightAngle(float a) { heightAbs=0.; focalLength=1./tan(0.5*a*RAI_PI/180.); }
/// the absolute height of the camera perspective (automatically also sets heightAngle=0)
void Camera::setHeightAbs(float h) { focalLength=0.; heightAbs=h; }
/// the z-range (depth range) visible for the camera
void Camera::setZRange(float znear, float zfar) { zNear=znear; zFar=zfar; }
/// set the width/height ratio of your viewport to see a non-distorted picture
void Camera::setWHRatio(float ratio) { whRatio=ratio; }
/// set the width/height ratio of your viewport to see a non-distorted picture
void Camera::setFocalLength(float f) { heightAbs=0;  focalLength = f; }
/// the frame's position
void Camera::setPosition(float x, float y, float z) { X.pos.set(x, y, z); }
/// rotate the frame to focus the point given by the vector
void Camera::focus(float x, float y, float z, bool makeUpright) { foc.set(x, y, z); watchDirection(foc-X.pos); if(makeUpright) upright(); }
/// rotate the frame to watch in the direction vector D
void Camera::watchDirection(const Vector& d) {
  if(d.x==0. && d.y==0.) {
    X.rot.setZero();
    if(d.z<0) X.rot.setDeg(180, 1, 0, 0);
    return;
  }
  Quaternion r;
  r.setDiff(X.rot.getZ(), d);
  X.rot=r*X.rot;
}
/// rotate the frame to set it upright (i.e. camera's y aligned with world's z)
void Camera::upright(const Vector& up) {
#if 1
  //construct desired X:
  Vector y = -X.rot.getY();
  Vector fwd = X.rot.getZ();
  Vector yDesired=fwd^(up^fwd); //desired Y
  if(yDesired*up<=0) yDesired=-yDesired;
  Quaternion r;
  r.setDiff(y, yDesired);
  X.rot=r*X.rot;
#else
  if(X.Z[2]<1.) X.Y.set(0, 0, 1); else X.Y.set(0, 1, 0);
  X.X=X.Y^X.Z; X.X.normalize();
  X.Y=X.Z^X.X; X.Y.normalize();
#endif
}

//}

void Camera::setCameraProjectionMatrix(const arr& P) {
  //P is in standard convention -> computes fixedProjectionMatrix in OpenGL convention from this
  cout <<"desired P=" <<P <<endl;
  arr Kview=arr{200., 0., 200., 0., 200., 200., 0., 0., 1.}; //OpenGL's calibration matrix
  Kview.reshape(3, 3);
  //arr glP=inverse(Kview)*P;
  arr glP=P;
  //glP[2]()*=-1.;
  glP.append(glP[2]);
  glP[2] *= .99; glP(2, 2)*=1.02; //some hack to invent a culling coordinate (usually determined via near and far...)
  glP = ~glP;
  glP *= 1./glP(3, 3);
  cout <<"glP=" <<glP <<endl;
  //glLoadMatrixd(glP.p);
  //fixedProjectionMatrix = glP;
}

void Camera::read(Graph& ats) {
  focalLength = ats.get<double>("focalLength", -1.);
  heightAbs = ats.get<double>("orthoAbsHeight", -1.);
  arr range =  ats.get<arr>("zRange", {});
  if(range.N) { zNear=range(0); zFar=range(1); }
  whRatio = ats.get<double>("width", 400.) / ats.get<double>("height", 200.);
}

void Camera::report(std::ostream& os) {
  os <<"camera pose X=" <<X <<endl;
  os <<"camera focal length=" <<focalLength <<endl;
  os <<"intrinsic matrix=\n" <<getIntrinsicMatrix(640, 480) <<endl;
}

arr Camera::getT_IC() const{
  arr P(4, 4);
  P.setZero();
  if(focalLength>0.) { //normal perspective mode
    P(0, 0) = 2.*focalLength/whRatio;
    P(1, 1) = -2.*focalLength;
    P(2, 2) = -(zFar + zNear)/(zNear-zFar);
    P(3, 2) = 1.;
    P(2, 3) = 2. * zFar * zNear / (zNear-zFar);
  }else if(heightAbs > 0.) { //ortho mode
    P(0, 0) = 2./heightAbs/whRatio;
    P(1, 1) = -2./heightAbs;
    P(2, 2) = -2./(zNear-zFar);
    P(2, 3) = 1.;
    P(3, 3) = 1.;
  }
  return P;
}

arr Camera::getT_CW() const{
  return X.getInverseAffineMatrix();
}

/** sets OpenGL's GL_PROJECTION matrix accordingly -- should be
    called in an opengl draw routine */
void Camera::glSetProjectionMatrix() const {
#ifdef RAI_GL
//  if(fixedProjectionMatrix.N) {
//    glLoadMatrixd(fixedProjectionMatrix.p);
//  } else {
  if(focalLength > 0.) { //focal lengh mode
#if 1
    CHECK(!heightAbs, "");
    arr P(4, 4);
    P.setZero();
    P(0, 0) = 2.*focalLength/whRatio;
    P(1, 1) = -2.*focalLength;
    P(2, 2) = -(zFar + zNear)/(zNear-zFar);
    P(2, 3) = 1.;
    P(3, 2) = 2. * zFar * zNear / (zNear-zFar);
    glLoadMatrixd(P.p);
#else
    double heightAngle = atan(1./focalLength)/RAI_PI*180.;
    gluPerspective(heightAngle, whRatio, zNear, zFar);
#endif
  }
  if(heightAbs > 0.) { //ortho mode
    CHECK(!focalLength, "");
    glOrtho(-whRatio*heightAbs/2., whRatio*heightAbs/2.,
            -heightAbs/2., heightAbs/2., zNear, zFar);
  }
//  if(heightAngle > 0.) { //normal perspective mode
//    CHECK(!focalLength, "");
//    CHECK(!heightAbs, "");
//    gluPerspective(heightAngle, whRatio, zNear, zFar);
//  }
  double m[16];
  glMultMatrixd(X.getInverseAffineMatrixGL(m));
#endif
}

arr Camera::getProjectionMatrix() const {
  arr Tinv = X.getInverseAffineMatrix();

  if(focalLength>0.) { //normal perspective mode
    CHECK(!heightAbs, "");
    arr P(4, 4);
    P.setZero();
    P(0, 0) = 2.*focalLength/whRatio;
    P(1, 1) = -2.*focalLength;
    P(2, 2) = 1.; //depth is flipped to become positive for 'in front of camera'
    P(3, 3) = 1.; //homogeneous 3D is kept
    return P * Tinv;
  }
  if(heightAbs > 0.) { //ortho mode
    NIY;
  }
  NIY;
  return arr();
}

arr Camera::getGLProjectionMatrix(bool includeCameraPose) const {
  arr Tinv = X.getInverseAffineMatrix();

  if(focalLength > 0.) { //focal lengh mode
    CHECK(!heightAbs, "");
    arr P(4, 4);
    P.setZero();
    P(0, 0) = 2.*focalLength/whRatio;
    P(1, 1) = -2.*focalLength;
    P(2, 2) = -(zFar + zNear)/(zNear-zFar);
    P(2, 3) = 1.;
    P(3, 2) = 2. * zFar * zNear / (zNear-zFar);
    if(!includeCameraPose) return P;
    return ~Tinv * P; //(P is already transposed!)
  }
#ifdef RAI_GL
  if(heightAbs > 0.) { //ortho mode
    CHECK(!focalLength, "");
    glOrtho(-whRatio*heightAbs/2., whRatio*heightAbs/2.,
            -heightAbs/2., heightAbs/2., zNear, zFar);
    NIY;
//    return T * Pinv;
  }
#endif
  NIY;
  return arr();
}

arr Camera::getInverseProjectionMatrix() const {
  arr T = X.getAffineMatrix();

  if(focalLength>0.) { //normal perspective mode
    arr Pinv(4, 4);
    Pinv.setZero();
    Pinv(0, 0) = 1./(2.*focalLength/whRatio);
    Pinv(1, 1) = -1./(2.*focalLength);
    Pinv(2, 2) = 1.; //flips 'positive depth' back to Right-Handed frame
    Pinv(3, 3) = 1.; //homogeneous 3D is kept
    return T * Pinv;
  }
  if(heightAbs > 0.) { //ortho mode
    arr Pinv(4, 4);
    Pinv.setZero();
    Pinv(0, 0) = 1./(2.*focalLength/whRatio);
    Pinv(1, 1) = -1./(2.*focalLength);
    Pinv(2, 2) = 1.; //flips 'positive depth' back to Right-Handed frame
    Pinv(3, 3) = 1.; //homogeneous 3D is kept
    NIY;
  }
  NIY;
  return arr();
}

/// convert from gluPerspective's non-linear [0, 1] depth to the true [zNear, zFar] depth
double Camera::glConvertToTrueDepth(double d) const {
  if(heightAbs) return zNear + (zFar-zNear)*d;
  return zNear + (zFar-zNear)*d/((zFar-zNear)/zNear*(1.-d)+1.); //TODO: optimize numerically
}

/// convert from gluPerspective's non-linear [0, 1] depth to the linear [0, 1] depth
double Camera::glConvertToLinearDepth(double d) const {
  CHECK(!heightAbs, "I think this is wrong for ortho view");
  return d/((zFar-zNear)/zNear*(1.-d)+1.);
}

void Camera::project2PixelsAndTrueDepth(arr& x, double width, double height) const {
  CHECK_LE(fabs(width/height - whRatio), 1e-6, "given width and height don't match whRatio");
  if(x.N==3) x.append(1.);
  CHECK_EQ(x.N, 4, "");
  arr P = getProjectionMatrix();
  x = P * x;
  double depth=x(2);
  x /= depth;
  x(2) = depth;
  x(1) = (x(1)+1.)*.5*(double)height;
  x(0) = (x(0)+1.)*.5*(double)width;
}

void Camera::unproject_fromPixelsAndTrueDepth(arr& x, double width, double height) const {
  if(heightAbs>0.) {
    x(0) = 2.*(x(0)/height) - 1.;
    x(1) = 2.*(x(1)/height) - 1.;
    x(0) *= .5*heightAbs;
    x(1) *= -.5*heightAbs;
    x(2) *= 1.;
    x.resizeCopy(3);
    X.applyOnPoint(x);
    return;
  }
  CHECK_LE(fabs(width/height - whRatio), 1e-2, "given width and height don't match whRatio");
  if(x.N==3) x.append(1.);
  CHECK_EQ(x.N, 4, "");
  arr Pinv = getInverseProjectionMatrix();
  double depth=x(2);
  x(0) = 2.*(x(0)/width) - 1.;
  x(1) = 2.*(x(1)/height) - 1.;
  x(2) = 1.;
  x *= depth;
  x(3) = 1.;
  x = Pinv * x;
  x.resizeCopy(3);
}

void Camera::unproject_fromPixelsAndGLDepth(arr& x, uint width, uint height) const {
#if 0
  CHECK_LE(fabs(double(width)/height - whRatio), 1e-6, "given width and height don't match whRatio");
  arr I = eye(4);
  arr P = getGLProjectionMatrix();
//  transpose(P);
  intA viewPort = {0, 0, (int)width, (int)height};
  double _x, _y, _z;
//  cout <<"\nM=\n" <<I <<"\nP=\n" <<P <<"\nV=\n" <<viewPort <<endl;
  gluUnProject(x(0), x(1), x(2), I.p, P.p, viewPort.p, &_x, &_y, &_z);
  x(0)=_x; x(1)=_y; x(2)=_z;
#else
  if(x.N==3) x.append(1.);
  CHECK_EQ(x.N, 4, "");
  x(2) = glConvertToTrueDepth(x(2));
  unproject_fromPixelsAndTrueDepth(x, width, height);
#endif
}

arr Camera::getFxycxy(double width, double height) { return arr{focalLength*height, focalLength*height, .5*width, .5*height}; }

arr Camera::getIntrinsicMatrix(double width, double height) const {
  if(focalLength>0.) { //normal perspective mode
    CHECK(!heightAbs, "");
    arr K(3, 3);
    K.setZero();
    K(0, 0) = focalLength*height;
    K(1, 1) = focalLength*height;
    K(2, 2) = 1.; //depth is flipped to become positive for 'in front of camera'
    K(0, 2) = -0.5*width;
    K(1, 2) = -0.5*height;
    return K;
  }
  NIY;
  return arr();
}

void Camera::setKinect() {
  setZero();
  setPosition(0., 0., 0.);
  focus(0., 0., 5.);
  setZRange(.1, 50.);
  setFocalLength(580./480.);
  whRatio = 640./480.;
}

void Camera::setDefault() {
  setHeightAngle(30.);
  setZRange(.02, 200.);
  setPosition(5., 10., 2.);
//  setPosition(10., -4., 10.);
  focus(0, .0, 1., true);
//  focus(.9, 0., 1.3);
}

//==============================================================================

std::istream& operator>>(std::istream& is, Vector& x)    { x.read(is); return is; }
std::istream& operator>>(std::istream& is, Matrix& x)    { x.read(is); return is; }
std::istream& operator>>(std::istream& is, Quaternion& x) { x.read(is); return is; }
std::istream& operator>>(std::istream& is, Transformation& x)     { x.read(is); return is; }
std::ostream& operator<<(std::ostream& os, const Vector& x)    { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Matrix& x)    { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Quaternion& x) { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Transformation& x)     { x.write(os); return os; }

} //namespace rai

//===========================================================================
//
// low level drivers
//

/** distance to surface, distance gradient, and hessian for this shape
 *
 * Details in inf cylinder section of
 * rai/stanio/concepts/note-analytic-impl-shapes-hessian
 */

//===========================================================================
//
// explicit instantiations
//

template rai::Array<rai::Vector>::Array();
template rai::Array<rai::Vector>::~Array();

template rai::Array<rai::Transformation*>::Array();
template rai::Array<rai::Transformation*>::~Array();

#include "../Core/util.ipp"
template rai::Vector rai::getParameter(const char*, const rai::Vector&);
