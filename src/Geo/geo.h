/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"

#include <iostream>

//fwd declarations
namespace rai {
struct Graph;
}

namespace rai {

//===========================================================================
/// a 3D vector (double[3])
struct Vector {
  double x, y, z;
  bool isZero;

  Vector() {}
  Vector(int zero) { CHECK_EQ(zero, 0, "this is only for initialization with zero"); setZero(); }
  Vector(double x, double y, double z) { set(x, y, z); }
  Vector(const Vector& v) { set(v.x, v.y, v.z); }
  Vector(const double* p) { set(p); }
  Vector(const arr& x) { CHECK_EQ(x.N, 3, "");  set(x.p); }
  Vector(const floatA& x) { CHECK_EQ(x.N, 3, "");  set(x(0), x(1), x(2)); }
  double* p() { return &x; }
  bool operator!() const;

  double& operator()(uint i);
  void set(double, double, double);
  void set(const arr& x) { CHECK_EQ(x.N, 3, "");  set(x.p); }
  void set(const std::vector<double>& x) { CHECK_EQ(x.size(), 3, "");  set(x.data()); }
  void set(const double*);
  void setZero();
  void setRandom(double range=1.);
  void normalize();
  void setLength(double);
  void makeNormal(const Vector&);
  void makeColinear(const Vector&);

  double diffZero() const;
  void checkZero() const;
  bool isNormalized() const;
  double isColinear(const Vector&) const;
  double length() const;
  double lengthSqr() const;
  double sum() const;
  double angle(const Vector&) const;
  double radius() const;
  double phi() const;
  double theta() const;
  arr getArr() const { return arr(&x, 3, false); }

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
  Matrix(int zero) { CHECK_EQ(zero, 0, "this is only for initialization with zero"); setZero(); }
  Matrix(const arr& m) { CHECK_EQ(m.N, 9, "");  set(m.p); }
  Matrix(const Matrix& m) : m00(m.m00), m01(m.m01), m02(m.m02), m10(m.m10), m11(m.m11), m12(m.m12), m20(m.m20), m21(m.m21), m22(m.m22) {}
  double* p() { return &m00; }
  arr getArr() const { return arr(&m00, 9, true).reshape(3, 3); }
  arr getDiag() const { return arr{m00, m11, m22}; }

  void set(double* m);
  void setZero();
  void setRandom(double range=1.);
  void setId();
  void setDiag(const arr& diag);
  void setSymmetric(const arr& entries6);
  void setFrame(Vector&, Vector&, Vector&);
  void setInvFrame(Vector&, Vector&, Vector&);
  void setXrot(double);
  void setSkew(const Vector&);
  void setExponential(const Vector&);
  void setOdeMatrix(double*);
  void setTensorProduct(const Vector&, const Vector&);

  double diffZero() const;

  bool isDiagonal() const { return !m01 && !m02 && !m10 && !m12 && !m20 && !m21; }
  void deleteOffDiagonal() { m01=m02=m10=m12=m20=m21=0.; }

  void write(std::ostream&) const;
  void read(std::istream&);
};

/// a quaternion (double[4])
struct Quaternion {
  double w, x, y, z;
  bool isZero;

  Quaternion() {}
  Quaternion(int zero) { CHECK_EQ(zero, 0, "this is only for initialization with zero"); setZero(); }
  Quaternion(double w, double x, double y, double z) { set(w, x, y, z); }
  Quaternion(const arr& q) { CHECK_EQ(q.N, 4, "");  set(q.p); }
  Quaternion(const Quaternion& q) { set(q.w, q.x, q.y, q.z); }
  double* p() { return &w; }

  double& operator()(uint i) { CHECK(i<4, "out of range"); return (&w)[i]; }
  void set(double w, double x, double y, double z);
  void set(const arr& q);
  void set(const std::vector<double>& x) { CHECK_EQ(x.size(), 4, "");  set(x.data()); }
  void set(const double* p);
  void setZero();
  Quaternion& setRandom();
  void setDeg(double degree, double axis0, double axis1, double axis2);
  void setDeg(double degree, const Vector& axis);
  void setRad(double radians, double axis0, double axis1, double axis2);
  void setRad(double radians, const Vector& axis);
  void setRad(double angle);
  void setRadX(double radians);
  void setRadY(double radians);
  void setRadZ(double radians);
  Quaternion& setRpy(double r, double p, double y);
  void setVec(Vector w);
  void setMatrix(double* m);
  void setMatrix(const arr& R) { CHECK_EQ(R.N, 9, ""); setMatrix(R.p); }
  void setDiff(const Vector& from, const Vector& to);
  void setInterpolate(double t, const Quaternion& a, const Quaternion b);
  void add(const Quaternion b, double w_b=1., double w_this=1.);
  Quaternion& invert();
  void flipSign();
  void uniqueSign();
  void normalize();
  void multiply(double f);
  void alignWith(const Vector& v);

  void addX(double radians);
  void addY(double radians);
  Quaternion& addZ(double radians);
  void append(const Quaternion& q);

  double diffZero() const;
  double sqrDiffZero() const;
  void checkZero() const;
  double sqrDiff(const Quaternion& q2) const;
  double normalization() const;
  bool isNormalized() const;
  double getDeg() const;
  double getRad() const;
  void getDeg(double& degree, Vector& axis) const;
  void getRad(double& angle, Vector& axis) const;
  Vector getVec() const;
  Vector getX() const;
  Vector getY() const;
  Vector getZ() const;
  Matrix getMatrix() const;
  arr    getArr() const;
  arr    getArr4d() const { return arr(&w, 4, false); }
  double* getMatrix(double* m) const;
  double* getMatrixOde(double* m) const; //in Ode foramt: 3x4 memory storae
  double* getMatrixGL(double* m) const;  //in OpenGL format: transposed 4x4 memory storage
  double getRoll_X() const;
  double getPitch_Y() const;
  double getYaw_Z() const;
  arr getEulerRPY() const;
  void applyOnPointArray(arr& pts) const;

  arr getJacobian() const;
  arr getMatrixJacobian() const;

  arr getQuaternionMultiplicationMatrix() const; //turns a RHS(!) quat multiplication into a LHS(!) matrix multiplication

  void writeNice(std::ostream& os) const;
  void write(std::ostream& os) const;
  void read(std::istream& is);
};

/// a transformation in 3D (position, orientation)
struct Transformation {
  Vector pos;     ///< position (translation)
  Quaternion rot; ///< orientation

  Transformation() {}
  Transformation(int zero) { CHECK_EQ(zero, 0, "this is only for initialization with zero"); setZero(); }
  Transformation(const Vector _pos, const Quaternion _rot) : pos(_pos), rot(_rot) {}
  Transformation(const Transformation& t) : pos(t.pos), rot(t.rot) {}
  Transformation(const char* init) { setText(init); }
  Transformation(const arr& t) { set(t); }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wclass-memaccess"
  void operator=(const Transformation& f) { memcpy(this, &f, sizeof(Transformation)); }
#pragma GCC diagnostic pop
  bool operator!() const;

  Transformation& setZero();
  Transformation& setText(const char* txt);
  void set(const double* p);
  void set(const arr& t);
  Transformation& setRandom();
  void setInverse(const Transformation& f);
  void setDifference(const Transformation& from, const Transformation& to);
  void setInterpolate(double t, const Transformation& a, const Transformation b);
  void setAffineMatrix(const double* m);

  bool isZero() const;
  double diffZero() const;
  void checkNan() const;

  Transformation& addRelativeTranslation(const Vector& t);
  Transformation& addRelativeTranslation(double x, double y, double z);
  void addRelativeRotation(const Quaternion&);
  void addRelativeRotationDeg(double degree, double x, double y, double z);
  void addRelativeRotationRad(double rad, double x, double y, double z);
  void addRelativeRotationQuat(double w, double x, double y, double z);

  void appendTransformation(const Transformation& f);     // this = this * f
  void appendInvTransformation(const Transformation& f);     // this = this * f^{-1}

  double* getAffineMatrix(double* m) const;         // 4x4 matrix with 3x3=rotation and right-column=translation
  arr getAffineMatrix() const;                      // 4x4 matrix with 3x3=rotation and right-column=translation
  double* getInverseAffineMatrix(double* m) const;  // 4x4 matrix with 3x3=R^{-1}   and bottom-row=R^{-1}*translation
  arr getInverseAffineMatrix() const;
  double* getAffineMatrixGL(double* m) const;       // in OpenGL format (transposed memory storage!!)
  double* getInverseAffineMatrixGL(double* m) const;// in OpenGL format (transposed memory storage!!)
  arr getArr7d() const;
  arr getWrenchTransform() const;

  void applyOnPoint(arr& pt) const;
  arr& applyOnPointArray(arr& pts) const;

  void write(std::ostream& os) const;
  void read(std::istream& is);
};

/// includes linear & angular velocities
struct DynamicTransformation : Transformation {
  Vector vel;     ///< linear velocity
  Vector angvel;  ///< angular velocity
  bool zeroVels;    ///< velocities are identically zero

  DynamicTransformation() {}
  DynamicTransformation(int zero) { CHECK_EQ(zero, 0, "this is only for initialization with zero"); setZero(); }
  DynamicTransformation(const DynamicTransformation& t) : Transformation(t), vel(t.vel), angvel(t.angvel), zeroVels(t.zeroVels) {}
  DynamicTransformation(const char* init);

  DynamicTransformation& setZero();
  DynamicTransformation& setText(const char* txt);
  void setRandom();
  void setInverse(const DynamicTransformation& f);
  void setDifference(const DynamicTransformation& from, const DynamicTransformation& to);
  void setAffineMatrix(const double* m);

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
  float focalLength;
  float whRatio;
  float zNear, zFar;

  Camera();

  void setZero();
  void setHeightAngle(float a);
  void setHeightAbs(float h);
  void setZRange(float znear, float zfar);
  void setWHRatio(float ratio);
  void setFocalLength(float f);
  void setPosition(float x, float y, float z);
  void setOffset(float x, float y, float z);
  void setKinect();
  void setDefault();

  void focus(float x, float y, float z, bool makeUpright=false);
  void watchDirection(const Vector& d);
  void upright(const Vector& up=Vector(0, 0, 1));

  //-- projection matrix stuff
  arr getT_IC() const;
  arr getT_CW() const;
  void glSetProjectionMatrix() const;
  arr getGLProjectionMatrix(bool includeCameraPose=true) const;
  arr getProjectionMatrix() const;
  arr getInverseProjectionMatrix() const;
  double glConvertToTrueDepth(double d) const;
  double glConvertToLinearDepth(double d) const;
  void project2PixelsAndTrueDepth(arr& x, double width, double height) const;
  void unproject_fromPixelsAndTrueDepth(arr& x, double width, double height) const;
  void unproject_fromPixelsAndGLDepth(arr& x, uint width, uint height) const;

  arr getFxycxy(double width, double height);
  arr getIntrinsicMatrix(double width, double height) const;

  //retired
  void setCameraProjectionMatrix(const arr& P); //P is in standard convention -> computes fixedProjectionMatrix in OpenGL convention from this

  void read(Graph& ats);
  void report(std::ostream& os=std::cout);
};

//===========================================================================
//
// operators
//

// C-style
void mult(Vector& a, const Quaternion& b, const Vector& c, bool add); //a += b*c (for add=true)
double sqrDistance(const rai::Vector& a, const rai::Vector& b);
// quaternion methods
double quat_scalarProduct(const rai::Quaternion& a, const rai::Quaternion& b);
double quat_sqrDistance(const rai::Quaternion& a, const rai::Quaternion& b);
// differentiable operations:
void quat_concat(arr& y, arr& Ja, arr& Jb, const arr& A, const arr& B);
void quat_normalize(arr& y, arr& J, const arr& a);
void quat_getVec(arr& y, arr& J, const arr& A);
/// return the difference of two orientations as a 3D-rotation-vector,
/// optionally also return the 'Jacobians' w.r.t. q1 and q2, but in terms
/// of a 'cross-product-matrix'
void quat_diffVector(arr& y, arr& Ja, arr& Jb, const arr& a, const arr& b);

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
Quaternion operator*=(Quaternion&, double);
bool       operator==(const Quaternion&, const Quaternion&);
bool       operator!=(const Quaternion&, const Quaternion&);
Quaternion operator-(const Quaternion&, const Quaternion&);

// TRANSFORMATION
Transformation operator-(const Transformation&);
Transformation operator*(const Transformation& b, const Transformation& c);
Transformation operator/(const Transformation& to, const Transformation& from);
bool           operator==(const Transformation&, const Transformation&);
bool           operator!=(const Transformation&, const Transformation&);

// MIXED
Vector operator*(const Quaternion& b, const Vector& c);
Vector operator/(const Vector& c, const Quaternion& b);
Vector operator*(const Transformation& b, const Vector& c);
Vector operator/(const Vector& c, const Transformation& b);

std::istream& operator>>(std::istream&, Vector&);
std::istream& operator>>(std::istream&, Matrix&);
std::istream& operator>>(std::istream&, Quaternion&);
std::istream& operator>>(std::istream&, Transformation&);
std::ostream& operator<<(std::ostream&, const Vector&);
std::ostream& operator<<(std::ostream&, const Matrix&);
std::ostream& operator<<(std::ostream&, const Quaternion&);
std::ostream& operator<<(std::ostream&, const Transformation&);

} //END of namespace

//===========================================================================
//
// conversions to arr
//

inline arr conv_vec2arr(const rai::Vector& v) {      return arr(&v.x, 3, false); }
inline arr conv_quat2arr(const rai::Quaternion& q) { return arr(&q.w, 4, false); }
inline arr conv_mat2arr(const rai::Matrix& m) {      return arr(&m.m00, 9, false).reshape(3, 3); }

//===========================================================================
//
// constants
//

extern const rai::Vector Vector_x;
extern const rai::Vector Vector_y;
extern const rai::Vector Vector_z;
extern const rai::Transformation Transformation_Id;
extern const rai::Quaternion Quaternion_Id;
extern const rai::Quaternion Quaternion_x;
extern const rai::Quaternion Quaternion_y;
extern const rai::Quaternion Quaternion_z;
extern rai::Vector& NoVector;
extern rai::Transformation& NoTransformation;
