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


#include <algorithm>
#include <Core/array.h>
#include "geo.h"

#ifndef MLR_NO_REGISTRY
#include <Core/graph.h>
REGISTER_TYPE(T, mlr::Transformation);
#endif


#include <GL/glu.h>

const mlr::Vector Vector_x(1, 0, 0);
const mlr::Vector Vector_y(0, 1, 0);
const mlr::Vector Vector_z(0, 0, 1);
const mlr::Transformation Transformation_Id(mlr::Transformation().setZero());
const mlr::Quaternion Quaternion_Id(1, 0, 0, 0);
const mlr::Quaternion Quaternion_x(MLR_SQRT2/2., MLR_SQRT2/2., 0, 0);
const mlr::Quaternion Quaternion_y(MLR_SQRT2/2., 0, MLR_SQRT2/2., 0);
const mlr::Quaternion Quaternion_z(MLR_SQRT2/2., 0, 0, MLR_SQRT2/2.);
mlr::Vector& NoVector = *((mlr::Vector*)NULL);
mlr::Transformation& NoTransformation = *((mlr::Transformation*)NULL);

namespace mlr {

double scalarProduct(const mlr::Quaternion& a, const mlr::Quaternion& b);

double& Vector::operator()(uint i) {
  CHECK(i<3,"out of range");
  isZero=false;
  return (&x)[i];
}

/// set the vector
void Vector::set(double _x, double _y, double _z) { x=_x; y=_y; z=_z; isZero=(x==0. && y==0. && z==0.); }

/// set the vector
void Vector::set(double* p) { x=p[0]; y=p[1]; z=p[2]; isZero=(x==0. && y==0. && z==0.); }

/// set the vector
void Vector::setZero() { memset(this, 0, sizeof(Vector)); isZero=true; }

/// a random vector in [-1, 1]^3
void Vector::setRandom(double range) { x=rnd.uni(-range, range); y=rnd.uni(-range, range); z=rnd.uni(-range, range); isZero=false; }

//{ vector operations

/// this=this/length(this)
void Vector::normalize() {
  if(isZero){
    MLR_MSG("can't normalize length of null vector");
  }
  (*this)/=length();
}

/// this=this*l/length(this)
void Vector::setLength(double l) {
  if(isZero) MLR_MSG("can't change length of null vector");
  (*this)*=l/length();
}

/// this=component of this normal to \c b, (unnormalized!)
void Vector::makeNormal(const Vector& b) {
  if(b.isZero) MLR_MSG("can't makeNormal with null vector");
  double l=b.length(), s=x*b.x+y*b.y+z*b.z;
  s/=l*l;
  x-=s*b.x; y-=s*b.y; z-=s*b.z;
}

/// this=component of this colinear to \c b, (unnormalized!)
void Vector::makeColinear(const Vector& b) {
  if(b.isZero) MLR_MSG("can't makeColinear with null vector");
  // *this = ((*this)*b)/b.length()) * (*this);
  double l=b.length(), s=x*b.x+y*b.y+z*b.z;
  s/=l*l;
  x=s*b.x; y=s*b.y; z=s*b.z;
}

//{ measuring the vector

/// L1-norm to zero
double Vector::diffZero() const { return fabs(x)+fabs(y)+fabs(z); }

/// is it normalized?
bool Vector::isNormalized() const { return fabs(lengthSqr()-1.)<1e-6; }

/// returns the length of this
double Vector::length() const { return ::sqrt(lengthSqr()); }

/// returns the square of length |a|^2
double Vector::lengthSqr() const { return x*x + y*y + z*z; }

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
  if(x==0. || ::fabs(x)<1e-10) ph=MLR_PI/2.; else ph=::atan(y/x);
  if(x<0.) { if(y<0.) ph-=MLR_PI; else ph+=MLR_PI; }
  return ph;
}

/// the angle from the x/y-plane
double Vector::theta() const { return ::atan(z/radius())+MLR_PI/2.; }

Vector Vector::getNormalVectorNormalToThis() const {
  if(isZero){
    MLR_MSG("every vector is normal to a zero vector");
  }
  arr s = ARR(fabs(x), fabs(y), fabs(z));
  uint c = s.maxIndex();
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
  Vector v(xv,yv,zv);
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
  if(!mlr::IOraw) os <<'(' <<x <<' ' <<y <<' ' <<z <<')';
  else os <<' ' <<x <<' ' <<y <<' ' <<z;
}

void Vector::read(std::istream& is) {
  if(!mlr::IOraw) is >>PARSE("(") >>x >>y >>z >>PARSE(")");
  else is >>x >>y >>z;
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
  if(!&rhs) return !&lhs; //if rhs==NoArr
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
  os <<"\n " <<m00 <<' ' <<m01 <<' ' <<m02;
  os <<"\n " <<m10 <<' ' <<m11 <<' ' <<m12;
  os <<"\n " <<m20 <<' ' <<m21 <<' ' <<m22;
  os <<endl;
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

/// multiplies the rotation by a factor f (i.e., makes f-times the rotation)
void Quaternion::multiply(double f) {
  if(w==1. || f==1.) return;
  double phi=acos(w);
  phi*=f;
  w=cos(phi);
  f=sin(phi)/sqrt(x*x + y*y + z*z);
  x*=f; y*=f; z*=f;
}

bool Quaternion::isNormalized() const {
  double n=w*w + x*x + y*y + z*z;
  return fabs(n-1.)<1e-6;
}

void Quaternion::normalize() {
  double n=w*w + x*x + y*y + z*z;
  n=sqrt(n);
  w/=n; x/=n; y/=n; z/=n;
  isZero=(w==1. || w==-1.);
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

void Quaternion::addX(double angle){
  if(!angle){ return; }
  angle/=2.;
  double cw=cos(angle);
  double cx=sin(angle);

  Quaternion a;
  a.w = w*cw - x*cx;
  a.x = w*cx + x*cw;
  a.y = y*cw + z*cx;
  a.z = z*cw - y*cx;

  set(a.w, a.x, a.y, a.z);
}

void Quaternion::addY(double angle){
  if(!angle){ return; }
  angle/=2.;
  double cw=cos(angle);
  double cy=sin(angle);

  Quaternion a;
  a.w = w*cw - y*cy;
  a.x = x*cw - z*cy;
  a.y = w*cy + y*cw;
  a.z = z*cw + x*cy;

  set(a.w, a.x, a.y, a.z);
}

void Quaternion::addZ(double radians){
  if(!radians){ return; }
  radians/=2.;
  double cw=cos(radians);
  double cz=sin(radians);

  Quaternion a;
  a.w = w*cw - z*cz;
  a.x = x*cw + y*cz;
  a.y = y*cw - x*cz;
  a.z = w*cz + z*cw;

  set(a.w, a.x, a.y, a.z);
}

void Quaternion::append(const Quaternion& q){
  if(q.isZero) return;
  double aw = w*q.w;
  double ax = x*q.w;
  double ay = y*q.w;
  double az = z*q.w;
  if(q.x){ aw -= x*q.x;  ax += w*q.x;  ay += z*q.x;  az -= y*q.x; }
  if(q.y){ aw -= y*q.y;  ax -= z*q.y;  ay += w*q.y;  az += x*q.y; }
  if(q.z){ aw -= z*q.z;  ax += y*q.z;  ay -= x*q.z;  az += w*q.z; }
  w=aw; x=ax; y=ay; z=az; isZero=false;
}

/// set the quad
void Quaternion::set(double* p) { w=p[0]; x=p[1]; y=p[2]; z=p[3]; isZero=(w==1. || w==-1.); }

/// set the quad
void Quaternion::set(const arr& q) { CHECK_EQ(q.N,4, "");  set(q.p); }

/// set the quad
void Quaternion::set(double _w, double _x, double _y, double _z) { w=_w; x=_x; y=_y; z=_z; isZero=(w==1. || w==-1.); }

/// reset the rotation to identity
void Quaternion::setZero() { memset(this, 0, sizeof(Quaternion));  w=1.; isZero=true; }

/// samples the rotation uniformly from the whole SO(3)
void Quaternion::setRandom() {
  double s, s1, s2, t1, t2;
  s=rnd.uni();
  s1=sqrt(1-s);
  s2=sqrt(s);
  t1=MLR_2PI*rnd.uni();
  t2=MLR_2PI*rnd.uni();
  w=cos(t2)*s2;
  x=sin(t1)*s1;
  y=cos(t1)*s1;
  z=sin(t2)*s2;
  isZero=false;
}

/// sets this to a smooth interpolation between two rotations
void Quaternion::setInterpolate(double t, const Quaternion& a, const Quaternion b) {
  double sign=1.;
  if(scalarProduct(a, b)<0) sign=-1.;
  w=a.w+t*(sign*b.w-a.w);
  x=a.x+t*(sign*b.x-a.x);
  y=a.y+t*(sign*b.y-a.y);
  z=a.z+t*(sign*b.z-a.z);
  normalize();
  isZero=false;
}

/// assigns the rotation to \c a DEGREES around the vector (x, y, z)
void Quaternion::setDeg(double degree, double _x, double _y, double _z) { setRad(degree*MLR_PI/180., _x, _y, _z); }

void Quaternion::setDeg(double degree, const Vector& vec) { setRad(degree*MLR_PI/180., vec.x, vec.y, vec.z); }

/// assigns the rotation to \c a RADIANTS (2*PI-units) around the vector (x, y, z)
void Quaternion::setRad(double angle, double _x, double _y, double _z) {
  if(!angle){ setZero(); return; }
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
void Quaternion::setRad(double angle, const Vector &axis) {
  setRad(angle, axis.x, axis.y, axis.z);
}

/// assigns the rotation to \c a RADIANTS (2*PI-units) around the current axis
void Quaternion::setRad(double angle) {
  if(!angle){ setZero(); return; }
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
void Quaternion::setRadX(double angle) {
  if(!angle){ setZero(); return; }
  angle/=2.;
  w=cos(angle);
  x=sin(angle);
  y=z=0.;
  isZero=false;
}

/// rotation around Y-axis by given radiants
void Quaternion::setRadY(double angle) {
  if(!angle){ setZero(); return; }
  angle/=2.;
  w=cos(angle);
  y=sin(angle);
  x=z=0.;
  isZero=false;
}

/// rotation around Z-axis by given radiants
void Quaternion::setRadZ(double angle) {
  if(!angle){ setZero(); return; }
  angle/=2.;
  w=cos(angle);
  z=sin(angle);
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
  CHECK(isNormalized(),"bad luck");
  return *this;
}

/// rotation around the given vector with angle (in rad) equal to norm of the vector
void Quaternion::setVec(Vector w) {
  double phi=w.length();
  setRad(phi, w.x, w.y, w.z);
}

/// rotation that will rotate 'from' to 'to' on direct path
void Quaternion::setDiff(const Vector& from, const Vector& to) {
  double phi=acos(from*to/(from.length()*to.length()));
  if(!phi){ setZero(); return; }
  Vector axis(from^to);
  if(axis.isZero) axis=Vector(0, 0, 1)^to;
  setRad(phi, axis);
}

/// L1-norm to zero (i.e., identical rotation)
double Quaternion::diffZero() const { return (w>0.?fabs(w-1.):fabs(w+1.))+fabs(x)+fabs(y)+fabs(z); }

double Quaternion::sqrDiffZero() const { return (w>0.?mlr::sqr(w-1.):mlr::sqr(w+1.))+mlr::sqr(x)+mlr::sqr(y)+mlr::sqr(z); }

/// return the squared-error between two quads, modulo flipping
double Quaternion::sqrDiff(const Quaternion& _q2) const{
  arr q1(&w, 4, true);
  arr q2(&_q2.w, 4, true);
  if(scalarProduct(q1,q2)>=0) return sqrDistance(q1, q2);
  return sqrDistance(-q1,q2);
}

/// gets rotation angle (in rad [0, 2pi])
double Quaternion::getRad() const {
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) return 0;
  return 2.*acos(w);
}

/// gets rotation angle (in degree [0, 360])
double Quaternion::getDeg() const {
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) return 0;
  return 360./MLR_PI*acos(w);
}

/// gets rotation angle (in degree [0, 360]) and vector
void Quaternion::getDeg(double& degree, Vector& vec) const {
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) { degree=0.; vec.set(0., 0., 1.); return; }
  degree=acos(w);
  double s=sin(degree);
  degree*=360./MLR_PI;
  vec.x=x/s; vec.y=y/s; vec.z=z/s;
}

/// gets rotation angle (in rad [0, 2pi]) and vector
void Quaternion::getRad(double& angle, Vector& vec) const {
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) { angle=0.; vec.set(0., 0., 1.); return; }
  angle=acos(w);
  double s=1./sin(angle);
  angle*=2;
  vec.x=s*x; vec.y=s*y; vec.z=s*z;
  CHECK(angle>=0. && angle<=MLR_2PI, "");
}

/// gets the axis rotation vector with length equal to the rotation angle in rad
Vector Quaternion::getVec() const {
  Vector vec;
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) { vec.setZero(); return vec; }
  double phi=acos(w);
  double s=2.*phi/sin(phi);
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
  arr R(3,3);
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

/// this is a 3-by-4 matrix $J$, giving the angular velocity vector $w = J \dot q$  induced by a $\dot q$
arr Quaternion::getJacobian() const{
  arr J(3,4);
  mlr::Quaternion e;
  for(uint i=0;i<4;i++){
    if(i==0) e.set(1.,0.,0.,0.);
    if(i==1) e.set(0.,1.,0.,0.);
    if(i==2) e.set(0.,0.,1.,0.);
    if(i==3) e.set(0.,0.,0.,1.);//TODO: the following could be simplified/compressed/made more efficient
    e = e / *this;
    J(0, i) = -2.*e.x;
    J(1, i) = -2.*e.y;
    J(2, i) = -2.*e.z;
  }
  return J;
}

/// this is a 4x(3x3) matrix, such that ~(J*x) is the jacobian of (R*x), and ~qdelta*J is (del R/del q)(qdelta)
arr Quaternion::getMatrixJacobian() const{
  arr J(4,9); //transpose!
  double r0=w, r1=x, r2=y, r3=z;
  J[0] = {      0,    -r3,     r2,
               r3,      0,    -r1,
              -r2,     r1,      0 };
  J[1] = {      0,     r2,     r3,
               r2, -2.*r1,    -r0,
               r3,     r0, -2.*r1 };
  J[2] = { -2.*r2,     r1,    r0,
               r1,      0,     r3,
              -r0,     r3, -2.*r2 };
  J[3] = { -2.*r3,    -r0,     r1,
               r0, -2.*r3,     r2,
               r1,     r2,      0};
  J *= 2.;
  J.reshape(4,3,3);
  return J;
}

void Quaternion::writeNice(std::ostream& os) const { os <<"Quaternion: " <<getDeg() <<" around " <<getVec() <<"\n"; }
void Quaternion::write(std::ostream& os) const {
  if(!mlr::IOraw) os <<'(' <<w <<' ' <<x <<' ' <<y <<' ' <<z <<')';
  else os <<' ' <<w <<' ' <<x <<' ' <<y <<' ' <<z;
}
void Quaternion::read(std::istream& is) { is >>PARSE("(") >>w >>x >>y  >>z >>PARSE(")"); normalize();}
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
  if(c.x){ a.w -= b.x*c.x;  a.x += b.w*c.x;  a.y += b.z*c.x;  a.z -= b.y*c.x; }
  if(c.y){ a.w -= b.y*c.y;  a.x -= b.z*c.y;  a.y += b.w*c.y;  a.z += b.x*c.y; }
  if(c.z){ a.w -= b.z*c.z;  a.x += b.y*c.z;  a.y -= b.x*c.z;  a.z += b.w*c.z; }
#endif
  a.isZero=(a.w==1. || a.w==-1.);
  return a;
}

/// A=B*C^{-1}
Quaternion operator/(const Quaternion& b, const Quaternion& c) {
  Quaternion a;
  a.w =-b.w*c.w - b.x*c.x - b.y*c.y - b.z*c.z;
  a.x = b.w*c.x - b.x*c.w + b.y*c.z - b.z*c.y;
  a.y = b.w*c.y - b.y*c.w + b.z*c.x - b.x*c.z;
  a.z = b.w*c.z - b.z*c.w + b.x*c.y - b.y*c.x;
  a.isZero=(a.w==1. || a.w==-1.);
  return a;
}

void mult(Vector& a, const Quaternion& b, const Vector& c,bool add){
  if(c.isZero){
    if(!add) a.setZero();
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
  if(c.x){ a.x += (1.-q22-q33)*c.x; a.y += (q12+q03)*c.x; a.z += (q13-q02)*c.x; }
  if(c.y){ a.x += (q12-q03)*c.y; a.y += (1.-q11-q33)*c.y; a.z += (q23+q01)*c.y; }
  if(c.z){ a.x += (q13+q02)*c.z; a.y += (q23-q01)*c.z; a.z += (1.-q11-q22)*c.z; }
  a.isZero = false;
}

/// transform of a vector by a rotation
Vector operator*(const Quaternion& b, const Vector& c) {
  if(c.isZero) return Vector(0);
  Vector a;
  mult(a,b,c,false);
  return a;
}

/// inverse transform of a vector by a rotation
Vector operator/(const Quaternion& b, const Vector& c) {
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

Transformation operator/(const Transformation& X, const Transformation& c) {
  //TODO: check whether this is sensible, where is it used??
#if 0
  Transformation f(X);
  f.appendInvTransformation(c);
#else
  Transformation f;
  f.setDifference(c,X);
#endif
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
Vector operator/(const Transformation& X, const Vector& c) {
  Vector a(c);
  a -= X.pos;
  a = X.rot / a;
  return a;
}

//==============================================================================

/// initialize by reading from the string
Transformation& Transformation::setText(const char* txt) { read(mlr::String(txt).stream()); return *this; }

/// resets the position to origin, rotation to identity, velocities to zero, scale to unit
Transformation& Transformation::setZero() {
  memset(this, 0, sizeof(Transformation));
  rot.w = 1.;
  pos.isZero = rot.isZero = true;
  return *this;
}

/// randomize the frame
void Transformation::setRandom() {
  rot.setRandom();
  pos.setRandom();
}

/// move the turtle by the vector (x, z, y) WITH RESPECT TO the current orientation/scale
void Transformation::addRelativeTranslation(double x, double y, double z) {
  addRelativeTranslation(Vector(x, y, z));
}

void Transformation::addRelativeTranslation(const Vector& x_rel){
  pos += rot*x_rel;
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

/** @brief transform the turtle into the frame f,
    which is interpreted RELATIVE to the current frame
    (new = old * f) */
void Transformation::appendTransformation(const Transformation& f) {
  if(!f.pos.isZero){
    if(rot.isZero) pos += f.pos;
    else mult(pos, rot, f.pos, true);
  }
  if(!f.rot.isZero){
    if(rot.isZero) rot = f.rot;
    else rot.append(f.rot);
  }
}

/// inverse transform (new = old * f^{-1})
void Transformation::appendInvTransformation(const Transformation& f) {
  rot = rot/f.rot;
  pos -= rot*f.pos;
}

/// this = f^{-1}
void Transformation::setInverse(const Transformation& f) {
  rot = -f.rot;
  pos = - (rot * f.pos);
}

/// set double[4*4] to Transformation. Matrix needs to be orthogonal
void Transformation::setAffineMatrix(const double *m) {
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
  rot = Quaternion_Id / from.rot * to.rot;
  pos = from.rot/(to.pos-from.pos);
}

/// get the current position/orientation/scale in an OpenGL format matrix (of type double[16])
double* Transformation::getAffineMatrix(double *m) const {
  Matrix M = rot.getMatrix();
  m[0] = M.m00; m[1] = M.m01; m[2] = M.m02; m[3] =pos.x;
  m[4] = M.m10; m[5] = M.m11; m[6] = M.m12; m[7] =pos.y;
  m[8] = M.m20; m[9] = M.m21; m[10]= M.m22; m[11]=pos.z;
  m[12]=0.;    m[13]=0.;    m[14]=0.;    m[15]=1.;
  return m;
}

arr Transformation::getAffineMatrix() const{
  arr T(4,4);
  getAffineMatrix(T.p);
  return T;
}

/// get inverse OpenGL matrix for this frame (of type double[16])
double* Transformation::getInverseAffineMatrix(double *m) const {
  Matrix M = rot.getMatrix();
  Vector pinv; pinv=rot/pos;
  m[0] =M.m00; m[1] =M.m10; m[2] =M.m20; m[3] =-pinv.x;
  m[4] =M.m01; m[5] =M.m11; m[6] =M.m21; m[7] =-pinv.y;
  m[8] =M.m02; m[9] =M.m12; m[10]=M.m22; m[11]=-pinv.z;
  m[12]=0.;   m[13]=0.;   m[14]=0.;   m[15]=1.;
  return m;
}

/// get the current position/orientation/scale in an OpenGL format matrix (of type double[16])
double* Transformation::getAffineMatrixGL(double *m) const {
  Matrix M = rot.getMatrix();
  m[0]=M.m00; m[4]=M.m01; m[8] =M.m02; m[12]=pos.x;
  m[1]=M.m10; m[5]=M.m11; m[9] =M.m12; m[13]=pos.y;
  m[2]=M.m20; m[6]=M.m21; m[10]=M.m22; m[14]=pos.z;
  m[3]=0.;   m[7]=0.;   m[11]=0.;   m[15]=1.;
  return m;
}

/// get inverse OpenGL matrix for this frame (of type double[16]) */
double* Transformation::getInverseAffineMatrixGL(double *m) const {
  Matrix M = rot.getMatrix();
  Vector pinv; pinv=rot/pos;
  m[0]=M.m00; m[4]=M.m10; m[8] =M.m20; m[12]=-pinv.x;
  m[1]=M.m01; m[5]=M.m11; m[9] =M.m21; m[13]=-pinv.y;
  m[2]=M.m02; m[6]=M.m12; m[10]=M.m22; m[14]=-pinv.z;
  m[3]=0.;   m[7]=0.;   m[11]=0.;   m[15]=1.;
  return m;
}

arr Transformation::getArr7d(){
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

void Transformation::applyOnPointArray(arr& pts) const{
  if(!((pts.nd==2 && pts.d1==3) || (pts.nd==3 && pts.d2==3))){
    LOG(-1) <<"wrong pts dimensions for transformation:" <<pts.dim();
    return;
  }
  arr R = ~rot.getArr(); //transposed, only to make it applicable to an n-times-3 array
  arr t = conv_vec2arr(pos);
  pts = pts * R;
  for(double *p=pts.p, *pstop=pts.p+pts.N; p<pstop; p+=3){
    p[0] += pos.x;
    p[1] += pos.y;
    p[2] += pos.z;
  }
// for(uint i=0;i<pts.d0;i++) pts[i]() += t; //inefficient...
}

bool Transformation::isZero() const {
  return pos.isZero && rot.isZero;
}

/// L1-norm to zero
double Transformation::diffZero() const {
  return pos.diffZero() + rot.diffZero();
}

/// operator<<
void Transformation::write(std::ostream& os) const {
  os <<pos.x <<' ' <<pos.y <<' ' <<pos.z <<' '
     <<rot.w <<' ' <<rot.x <<' ' <<rot.y <<' ' <<rot.z;
}

/// operator>>
void Transformation::read(std::istream& is) {
  setZero();
  char c;
  double x[4];
  mlr::skip(is, " \n\r\t<|");
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
          //case 's': is>>PARSE("(")>>x[0]>>PARSE(")");                   scale(x[0]); break;
        case '|':
        case '>': is.putback(c); return; //those symbols finish the reading without error
        default: MLR_MSG("unknown Transformation read tag: " <<c <<"abort reading this frame"); is.putback(c); return;
      }
    if(is.fail()) HALT("error reading '" <<c <<"' parameters in frame");
  }
  if(is.fail()) HALT("could not read Transformation struct");
}

//==============================================================================

/// initialize by reading from the string
DynamicTransformation& DynamicTransformation::setText(const char* txt) { read(mlr::String(txt)()); return *this; }

/// resets the position to origin, rotation to identity, velocities to zero, scale to unit
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

void DynamicTransformation::addRelativeTranslation(const Vector& x_rel){
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
  W*=degree*MLR_PI/180.;
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
    if(!f.pos.isZero){ if(rot.isZero) pos += f.pos; else pos += rot*f.pos; }
    if(!f.rot.isZero){ if(rot.isZero) rot = f.rot; else rot = rot*f.rot; }
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
void DynamicTransformation::setAffineMatrix(const double *m) {
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
    pos = from.rot/(to.pos-from.pos);
    zeroVels = true;
  } else {
    rot = Quaternion_Id / from.rot * to.rot;
    angvel = from.rot/(to.angvel-from.angvel);
    vel = from.rot/(to.vel-from.vel);
    vel-= from.rot/(from.angvel^(to.pos-from.pos));
    pos = from.rot/(to.pos-from.pos);
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
  os <<pos.x <<' ' <<pos.y <<' ' <<pos.z <<' '
     <<rot.w <<' ' <<rot.x <<' ' <<rot.y <<' ' <<rot.z;
  if(!zeroVels) {
    os <<" v" <<vel <<" w" <<angvel;
  }
}

/// operator>>
void DynamicTransformation::read(std::istream& is) {
  setZero();
  char c;
  double x[4];
  mlr::skip(is, " \n\r\t<|");
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
        default: MLR_MSG("unknown DynamicTransformation read tag: " <<c <<"abort reading this frame"); is.putback(c); return;
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
  focus(0, 0, 0);
  setZRange(.1, 1000.);
  setHeightAngle(12.);
}

void Camera::setZero() {
  X.setZero();
  foc.setZero();
  heightAngle=90.;
  heightAbs=10.;
  focalLength=1.;
  whRatio=1.;
  zNear=.1;
  zFar=1000.;
}

/// the height angle (in degrees) of the camera perspective; set it 0 for orthogonal projection
void Camera::setHeightAngle(float a) { heightAngle=a; heightAbs=0.;}
/// the absolute height of the camera perspective (automatically also sets heightAngle=0)
void Camera::setHeightAbs(float h) { heightAngle=0.; heightAbs=h; }
/// the z-range (depth range) visible for the camera
void Camera::setZRange(float znear, float zfar) { zNear=znear; zFar=zfar; }
/// set the width/height ratio of your viewport to see a non-distorted picture
void Camera::setWHRatio(float ratio) { whRatio=ratio; }
/// the frame's position
void Camera::setPosition(float x, float y, float z) { X.pos.set(x, y, z); }
/// rotate the frame to focus the absolute coordinate origin (0, 0, 0)
void Camera::focusOrigin() { foc.setZero(); focus(); }
/// rotate the frame to focus the point (x, y, z)
void Camera::focus(float x, float y, float z) { foc.set(x, y, z); focus(); }
/// rotate the frame to focus the point given by the vector
void Camera::focus(const Vector& v) { foc=v; focus(); }
/// rotate the frame to focus (again) the previously given focus
void Camera::focus() { watchDirection(foc-X.pos); } //X.Z=X.pos; X.Z-=foc; X.Z.normalize(); upright(); }
/// rotate the frame to watch in the direction vector D
void Camera::watchDirection(const Vector& d) {
  if(d.x==0. && d.y==0.) {
    X.rot.setZero();
    if(d.z>0) X.rot.setDeg(180, 1, 0, 0);
    return;
  }
  Quaternion r;
  r.setDiff(-X.rot.getZ(), d);
  X.rot=r*X.rot;
}
/// rotate the frame to set it upright (i.e. camera's y aligned with world's z)
void Camera::upright(const Vector& up) {
#if 1
  //construct desired X:
  Vector fwd(0, 0, -1), x(1, 0, 0), xDesired;
  x=X.rot*x; //true X
  fwd=X.rot*fwd;
//  if(fabs(fwd.z)<1.) up.set(0, 0, 1); else up.set(0, 1, 0);
  xDesired=up^fwd; //desired X
  if(xDesired*x<=0) xDesired=-xDesired;
  Quaternion r;
  r.setDiff(x, xDesired);
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
  arr Kview=ARR(200., 0., 200., 0., 200., 200., 0., 0., 1.); //OpenGL's calibration matrix
  Kview.reshape(3, 3);
  //arr glP=inverse(Kview)*P;
  arr glP=P;
  //glP[2]()*=-1.;
  glP.append(glP[2]);
  glP[2]()*=.99; glP(2, 2)*=1.02; //some hack to invent a culling coordinate (usually determined via near and far...)
  glP = ~glP;
  glP *= 1./glP(3, 3);
  cout <<"glP=" <<glP <<endl;
  //glLoadMatrixd(glP.p);
  //fixedProjectionMatrix = glP;
}

/** sets OpenGL's GL_PROJECTION matrix accordingly -- should be
    called in an opengl draw routine */
void Camera::glSetProjectionMatrix() {
//  if(fixedProjectionMatrix.N) {
//    glLoadMatrixd(fixedProjectionMatrix.p);
//  } else {
  if(heightAngle==0) {
    if(heightAbs==0) {
      arr P(4,4);
      P.setZero();
      P(0,0) = 2.*focalLength/whRatio;
      P(1,1) = 2.*focalLength;
      P(2,2) = (zFar + zNear)/(zNear-zFar);
      P(2,3) = -1.;
      P(3,2) = 2. * zFar * zNear / (zNear-zFar);
      glLoadMatrixd(P.p);
    }else{
      glOrtho(-whRatio*heightAbs/2, whRatio*heightAbs/2,
              -heightAbs/2, heightAbs/2, zNear, zFar);
    }
  } else
    gluPerspective(heightAngle, whRatio, zNear, zFar);
  double m[16];
  glMultMatrixd(X.getInverseAffineMatrixGL(m));
}

/// convert from gluPerspective's non-linear [0, 1] depth to the true [zNear, zFar] depth
double Camera::glConvertToTrueDepth(double d) {
  return zNear + (zFar-zNear)*d/(zFar/zNear*(1.-d)+1.);
}

/// convert from gluPerspective's non-linear [0, 1] depth to the linear [0, 1] depth
double Camera::glConvertToLinearDepth(double d) {
  return d/(zFar/zNear*(1.-d)+1.);
}

void Camera::setKinect(){
  setZero();
  setPosition(0., 0., 0.);
  focus(0., 0., 5.);
  setZRange(.1, 50.);
#if 1
  heightAbs=heightAngle = 0;
  focalLength = 580./480.;
#else
  heightAngle=45;
#endif
  whRatio = 640./480.;
}

void Camera::setDefault(){
  setHeightAngle(12.);
  setPosition(10., -15., 8.);
  focus(0, 0, 1.);
  upright();
}

//==============================================================================

/// use as similarity measure (distance = 1 - |scalarprod|)
double scalarProduct(const Quaternion& a, const Quaternion& b) {
  return a.w*b.w+a.x*b.x+a.y*b.y+a.z*b.z;
}

std::istream& operator>>(std::istream& is, Vector& x)    { x.read(is); return is; }
std::istream& operator>>(std::istream& is, Matrix& x)    { x.read(is); return is; }
std::istream& operator>>(std::istream& is, Quaternion& x) { x.read(is); return is; }
std::istream& operator>>(std::istream& is, Transformation& x)     { x.read(is); return is; }
std::ostream& operator<<(std::ostream& os, const Vector& x)    { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Matrix& x)    { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Quaternion& x) { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Transformation& x)     { x.write(os); return os; }

} //namespace mlr


//===========================================================================
//
// low level drivers
//

/** distance to surface, distance gradient, and hessian for this shape
 *
 * Details in inf cylinder section of
 * mlr/stanio/concepts/note-analytic-impl-shapes-hessian
 */



//===========================================================================
//
// explicit instantiations
//

template mlr::Array<mlr::Vector>::Array();
template mlr::Array<mlr::Vector>::~Array();

template mlr::Array<mlr::Transformation*>::Array();
template mlr::Array<mlr::Transformation*>::~Array();
