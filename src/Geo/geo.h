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


/// @file
/// @ingroup group_Core
/// @addtogroup group_Core
/// @{

#ifndef MLR_geo_h
#define MLR_geo_h

#include <Core/array.h>

namespace mlr {

//===========================================================================
/// a 3D vector (double[3])
struct Vector {
  double x, y, z;
  bool isZero;

  Vector() {}
  Vector(int zero){ CHECK_EQ(zero,0,"this is only for initialization with zero"); setZero(); }
  Vector(double x, double y, double z) { set(x, y, z); }
  Vector(const Vector& v) { set(v.x, v.y, v.z); }
  Vector(const arr& x) { CHECK_EQ(x.N,3, "");  set(x.p); }
  Vector(const arrf& x) { CHECK_EQ(x.N,3, "");  set(x(0), x(1), x(2)); }
  double *p() { return &x; }
  
  double& operator()(uint i);
  void set(double, double, double);
  void set(const arr& x){ CHECK_EQ(x.N,3, "");  set(x.p); }
  void set(double*);
  void setZero();
  void setRandom(double range=1.);
  void normalize();
  void setLength(double);
  void makeNormal(const Vector&);
  void makeColinear(const Vector&);
  
  double diffZero() const;
  bool isNormalized() const;
  double isColinear(const Vector&) const;
  double length() const;
  double lengthSqr() const;
  double angle(const Vector&) const;
  double radius() const;
  double phi() const;
  double theta() const;
  arr getArr() const{ return arr(&x, 3, false); }

  Vector getNormalVectorNormalToThis() const;
  void generateOrthonormalSystem(Vector& u, Vector& v) const;
  arr generateOrthonormalSystemMatrix() const;
  
  void write(std::ostream&) const;
  void read(std::istream&);
};

/// a matrix in 3D (double[9])
struct Matrix {
  double m00, m01, m02, m10, m11, m12, m20, m21, m22;
  
  Matrix() {}
  Matrix(int zero){ CHECK_EQ(zero,0,"this is only for initialization with zero"); setZero(); }
  Matrix(const arr& m) { CHECK_EQ(m.N,9, "");  set(m.p); };
  Matrix(const Matrix& m) : m00(m.m00), m01(m.m01), m02(m.m02), m10(m.m10), m11(m.m11), m12(m.m12), m20(m.m20), m21(m.m21), m22(m.m22) {}
  double *p() { return &m00; }
  
  void set(double* m);
  void setZero();
  void setRandom(double range=1.);
  void setId();
  void setFrame(Vector&, Vector&, Vector&);
  void setInvFrame(Vector&, Vector&, Vector&);
  void setXrot(double);
  void setSkew(const Vector&);
  void setExponential(const Vector&);
  void setOdeMatrix(double*);
  void setTensorProduct(const Vector&, const Vector&);
  
  double diffZero() const;
  
  void write(std::ostream&) const;
  void read(std::istream&);
};

/// a quaternion (double[4])
struct Quaternion {
  double w, x, y, z;
  bool isZero;

  Quaternion() {}
  Quaternion(int zero){ CHECK_EQ(zero,0,"this is only for initialization with zero"); setZero(); }
  Quaternion(double w, double x, double y, double z) { set(w,x,y,z); }
  Quaternion(const arr& q) { CHECK_EQ(q.N,4, "");  set(q.p); }
  Quaternion(const Quaternion& q) { set(q.w, q.x, q.y, q.z); }
  double *p() { return &w; }

  double& operator()(uint i){ CHECK(i<4,"out of range"); return (&w)[i]; }
  void set(double w, double x, double y, double z);
  void set(const arr& q);
  void set(double* p);
  void setZero();
  void setRandom();
  void setDeg(double degree , double axis0, double axis1, double axis2);
  void setDeg(double degree , const Vector& axis);
  void setRad(double radians, double axis0, double axis1, double axis2);
  void setRad(double radians, const Vector& axis);
  void setRad(double angle);
  void setRadX(double angle);
  void setRadY(double angle);
  void setRadZ(double angle);
  Quaternion& setRpy(double r, double p, double y);
  void setVec(Vector w);
  void setMatrix(double* m);
  void setDiff(const Vector& from, const Vector& to);
  void setInterpolate(double t, const Quaternion& a, const Quaternion b);
  Quaternion& invert();
  void flipSign();
  void normalize();
  void multiply(double f);
  void alignWith(const Vector& v);

  void addX(double radians);
  void addY(double radians);
  void addZ(double radians);
  void append(const Quaternion& q);

  double diffZero() const;
  double sqrDiffZero() const;
  double sqrDiff(const Quaternion& q2) const;
  bool isNormalized() const;
  double getDeg() const;
  double getRad() const;
  void getDeg(double& degree, Vector& axis) const;
  void getRad(double& angle , Vector& axis) const;
  Vector getVec() const;
  Vector getX() const;
  Vector getY() const;
  Vector getZ() const;
  Matrix getMatrix() const;
  arr    getArr() const;
  arr    getArr4d() const{ return arr(&w, 4, false); }
  double* getMatrix(double* m) const;
  double* getMatrixOde(double* m) const; //in Ode foramt: 3x4 memory storae
  double* getMatrixGL(double* m) const;  //in OpenGL format: transposed 4x4 memory storage

  arr getJacobian() const;
  arr getMatrixJacobian() const;

  void writeNice(std::ostream& os) const;
  void write(std::ostream& os) const;
  void read(std::istream& is);
};

/// a transformation in 3D (position, orientation, linear & angular velocities)
struct Transformation {
  Vector pos;     ///< position (translation)
  Quaternion rot; ///< orientation
  
  Transformation() {}
  Transformation(int zero){ CHECK_EQ(zero,0,"this is only for initialization with zero"); setZero(); }
  Transformation(const Transformation &t) : pos(t.pos), rot(t.rot) {}
  Transformation(const char* init) { setText(init); }

  Transformation& setZero();
  Transformation& setText(const char* txt);
  void setRandom();
  void setInverse(const Transformation& f);
  void setDifference(const Transformation& from, const Transformation& to);
  void setAffineMatrix(const double *m);
  
  bool isZero() const;
  double diffZero() const;
  
  void addRelativeTranslation(const Vector& t);
  void addRelativeTranslation(double x, double y, double z);
  void addRelativeRotation(const Quaternion&);
  void addRelativeRotationDeg(double degree, double x, double y, double z);
  void addRelativeRotationRad(double rad, double x, double y, double z);
  void addRelativeRotationQuat(double w, double x, double y, double z);
  
  void appendTransformation(const Transformation& f);     // this = this * f
  void appendInvTransformation(const Transformation& f);     // this = this * f^{-1}
  
  double* getAffineMatrix(double *m) const;         // 4x4 matrix with 3x3=rotation and right-column=translation
  arr getAffineMatrix() const;                      // 4x4 matrix with 3x3=rotation and right-column=translation
  double* getInverseAffineMatrix(double *m) const;  // 4x4 matrix with 3x3=R^{-1}   and bottom-row=R^{-1}*translation
  double* getAffineMatrixGL(double *m) const;       // in OpenGL format (transposed memory storage!!)
  double* getInverseAffineMatrixGL(double *m) const;// in OpenGL format (transposed memory storage!!)
  arr getArr7d();
  
  void applyOnPointArray(arr& pts) const;

  void write(std::ostream& os) const;
  void read(std::istream& is);
};

/// includes linear & angular velocities
struct DynamicTransformation : Transformation{
  Vector vel;     ///< linear velocity
  Vector angvel;  ///< angular velocity
  bool zeroVels;    ///< velocities are identically zero

  DynamicTransformation() {}
  DynamicTransformation(int zero){ CHECK_EQ(zero,0,"this is only for initialization with zero"); setZero(); }
  DynamicTransformation(const DynamicTransformation &t) : Transformation(t), vel(t.vel), angvel(t.angvel), zeroVels(t.zeroVels) {}
  DynamicTransformation(const char* init) { read(mlr::String(init).stream()); }

  DynamicTransformation& setZero();
  DynamicTransformation& setText(const char* txt);
  void setRandom();
  void setInverse(const DynamicTransformation& f);
  void setDifference(const DynamicTransformation& from, const DynamicTransformation& to);
  void setAffineMatrix(const double *m);

  bool isZero() const;
  double diffZero() const;

  void addRelativeTranslation(double x, double y, double z);
  void addRelativeTranslation(const Vector& x_rel);
  void addRelativeVelocity(double x, double y, double z);
  void addRelativeAngVelocityDeg(double degree, double x, double y, double z);
  void addRelativeAngVelocityRad(double rad, double x, double y, double z);
  void addRelativeAngVelocityRad(double wx, double wy, double wz);

  void appendTransformation(const DynamicTransformation& f);     // this = this * f
  void appendInvTransformation(const DynamicTransformation& f);     // this = this * f^{-1}

  void write(std::ostream& os) const;
  void read(std::istream& is);
};

/// a camera projection in 3D
struct Camera {
  Transformation X;
  Vector foc;

  float heightAbs;
  float heightAngle;
  float focalLength;
  float whRatio;
  float zNear, zFar;

  Camera();

  void setZero();
  void setHeightAngle(float a);
  void setHeightAbs(float h);
  void setZRange(float znear, float zfar);
  void setWHRatio(float ratio);
  void setPosition(float x, float y, float z);
  void setOffset(float x, float y, float z);
  void setCameraProjectionMatrix(const arr& P); //P is in standard convention -> computes fixedProjectionMatrix in OpenGL convention from this
  void focusOrigin();
  void focus(float x, float y, float z);
  void focus(const Vector& v);
  void focus();
  void watchDirection(const Vector& d);
  void upright(const Vector& up=Vector(0,0,1));
  void glSetProjectionMatrix();
  double glConvertToTrueDepth(double d);
  double glConvertToLinearDepth(double d);
  void setKinect();
  void setDefault();
};


//===========================================================================
//
// operators
//

// efficient
void mult(Vector& a, const Quaternion& b, const Vector& c,bool add); //a += b*c (for add=true)

// VECTOR
double  operator*(const Vector&, const Vector&);
Vector  operator^(const Vector&, const Vector&);
Vector  operator+(const Vector&, const Vector&);
Vector  operator-(const Vector&, const Vector&);
Vector  operator*(double, const Vector&);
Vector  operator*(const Vector&, double);
Vector  operator/(const Vector&, double);
Vector& operator*=(Vector&, double);
Vector& operator/=(Vector&, double);
Vector& operator+=(Vector&, const Vector&);
Vector& operator-=(Vector&, const Vector&);
Vector  operator-(const Vector&);
bool    operator==(const Vector&, const Vector&);
bool    operator!=(const Vector&, const Vector&);

// MATRIX
Matrix  operator*(const Matrix& b, const Matrix& c);
Matrix  operator+(const Matrix& b, const Matrix& c);
Vector  operator*(const Matrix& b, const Vector& c);
Matrix& operator*=(Matrix& a, double c);
Matrix  operator*(double b, const Matrix& c);
Matrix& operator+=(Matrix& a, const Matrix& b);
bool    operator==(const Matrix&, const Matrix&);
bool    operator!=(const Matrix&, const Matrix&);

// QUATERNION
Quaternion operator-(const Quaternion&);
Quaternion operator*(const Quaternion& b, const Quaternion& c);
Quaternion operator/(const Quaternion& b, const Quaternion& c);
bool       operator==(const Quaternion&, const Quaternion&);
bool       operator!=(const Quaternion&, const Quaternion&);

// TRANSFORMATION
Transformation operator-(const Transformation&);
Transformation operator*(const Transformation& b, const Transformation& c);
Transformation operator/(const Transformation& b, const Transformation& c);
bool           operator==(const Transformation&, const Transformation&);
bool           operator!=(const Transformation&, const Transformation&);

// MIXED
Vector operator*(const Quaternion& b, const Vector& c);
Vector operator/(const Quaternion& b, const Vector& c);
Vector operator*(const Transformation& b, const Vector& c);
Vector operator/(const Transformation& b, const Vector& c);

std::istream& operator>>(std::istream&, Vector&);
std::istream& operator>>(std::istream&, Matrix&);
std::istream& operator>>(std::istream&, Quaternion&);
std::istream& operator>>(std::istream&, Transformation&);
std::ostream& operator<<(std::ostream&, const Vector&);
std::ostream& operator<<(std::ostream&, const Matrix&);
std::ostream& operator<<(std::ostream&, const Quaternion&);
std::ostream& operator<<(std::ostream&, const Transformation&);

//===========================================================================
//
// more complex operations
//

/// return the difference of two orientations as a 3D-rotation-vector,
/// optionally also return the 'Jacobians' w.r.t. q1 and q2, but in terms
/// of a 'cross-product-matrix'
void quatDiff(arr& y, arr& J1, arr& J2, const Quaternion& q1, const Quaternion& q2);


} //END of namespace

//===========================================================================
//
// conversions to arr
//

inline arr conv_vec2arr(const mlr::Vector& v) {      return arr(&v.x, 3, false); }
inline arr conv_quat2arr(const mlr::Quaternion& q) { return arr(&q.w, 4, false); }
inline arr conv_mat2arr(const mlr::Matrix& m) {      return arr(&m.m00, 9, false); }


//===========================================================================
//
// constants
//

extern const mlr::Vector Vector_x;
extern const mlr::Vector Vector_y;
extern const mlr::Vector Vector_z;
extern const mlr::Transformation Transformation_Id;
extern const mlr::Quaternion Quaternion_Id;
extern const mlr::Quaternion Quaternion_x;
extern const mlr::Quaternion Quaternion_y;
extern const mlr::Quaternion Quaternion_z;
extern mlr::Vector& NoVector;
extern mlr::Transformation& NoTransformation;


//===========================================================================
//
// low level drivers
//


#endif

/// @} //end group
