/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "defines.h"

#include "array.h"

//#include <iosfwd>
#include <iostream>
#include <stdint.h>
#include <string.h>
#include <functional>
#include <memory>
#include <vector>

using std::unique_ptr;
using std::shared_ptr;
using std::make_unique;
using std::make_shared;

typedef unsigned char byte;
typedef unsigned int uint;

//fwd declarations
struct Serializable {
  virtual uint serial_size() = 0;
  virtual uint serial_encode(char* data, uint data_size) = 0;
  virtual uint serial_decode(char* data, uint data_size) = 0;
};

namespace rai {

//struct FileToken;

// OLD, TODO: hide -> array.cpp
extern bool useLapack;
extern const bool lapackSupported;

// default write formatting
extern const char* arrayElemsep;
extern const char* arrayLinesep;
extern const char* arrayBrackets;

// default sorting methods
//bool lower(const double& a, const double& b) { return a<b; }
//bool lowerEqual(const double& a, const double& b) { return a<=b; }
//bool greater(const double& a, const double& b) { return a>b; }
//bool greaterEqual(const double& a, const double& b) { return a>=b; }

} //namespace

//===========================================================================
//
// Array class
//

namespace rai {

#if 0
/** Simple array container to store arbitrary-dimensional arrays (tensors).
  Can buffer more memory than necessary for faster
  resize; enables non-const reference of subarrays; enables fast
  memove for elementary types; implements many standard
  array/matrix/tensor operations. Interfacing with ordinary C-buffers is simple.
  Please see also the reference for the \ref array.h
  header, which contains lots of functions that can be applied on
  Arrays. */
struct ArrayDouble : public Array<double> {

  //c.tors
  ArrayDouble() : Array<double>() {}
  ArrayDouble(const ArrayDouble& a) { operator=(a); } //copy constructor
  ArrayDouble(const Array<double>& a) { operator=(a); } //copy constructor
  ArrayDouble(ArrayDouble&& a) : Array<double>((Array<double>&&)a) { if(a.jac) jac = std::move(a.jac); }  //move constructor
  explicit ArrayDouble(uint D0) { resize(D0); }
  explicit ArrayDouble(uint D0, uint D1) { resize(D0, D1); }
  explicit ArrayDouble(uint D0, uint D1, uint D2) { resize(D0, D1, D2); }
  explicit ArrayDouble(const double* p, uint size, bool byReference) { if(byReference) referTo(p, size); else setCarray(p, size); }
  explicit ArrayDouble(const std::vector<double>& a, bool byReference) { if(byReference) referTo(&a.front(), a.size()); else setCarray(&a.front(), a.size()); }
  ArrayDouble(std::initializer_list<double> values) { operator=(values); }
  ArrayDouble(std::initializer_list<uint> dim, std::initializer_list<double> values) { operator=(values); reshape(dim); }

  //assignments
  ArrayDouble& operator=(std::initializer_list<double> values);
  ArrayDouble& operator=(const double& v);
  ArrayDouble& operator=(const ArrayDouble& a);
  ArrayDouble& operator=(const Array<double>& a);
  ArrayDouble& operator=(const std::vector<double>& values) { setCarray(&values.front(), values.size()); return *this; }

  //conversion
  std::vector<double> vec() const { return std::vector<double>(p, p+N); }
  ArrayDouble& noconst() { return *this; }

  /// @name access by reference (direct memory access)
  ArrayDouble ref() const { ArrayDouble x; x.referTo(*this); return x; }
  using Array<double>::elem; //adopt all elem(..) methods
  double& elem(int i, int j); //access that also handles sparse matrices
  using Array<double>::operator(); //adopt all double& operator()(..) methods
  ArrayDouble operator()(std::pair<int, int> I) const { ArrayDouble z; z.referToRange(*this, I.first, I.second+1); return z; }
  ArrayDouble operator()(int i, std::pair<int, int> J) const { ArrayDouble z; z.referToRange(*this, i, J.first, J.second); return z; }
  ArrayDouble operator()(int i, int j, std::initializer_list<int> K) const;
  ArrayDouble operator[](int i) const { ArrayDouble z; z.referToDim(*this, i); return z; }

  /// @name access by copy (overloaded because of return value)
  ArrayDouble copy() const { return ArrayDouble(*this); }

  ArrayDouble sub(int i, int I) const;
  ArrayDouble sub(int i, int I, int j, int J) const;
  ArrayDouble sub(int i, int I, int j, int J, int k, int K) const;
  ArrayDouble sub(int i, int I, Array<uint> cols) const;
  ArrayDouble sub(Array<uint> elems) const;
  ArrayDouble row(uint row_index) const { return sub(row_index, row_index, 0, d1 - 1); }
  ArrayDouble rows(uint start_row, uint end_row) const { return sub(start_row, end_row - 1, 0, d1 - 1); }
  ArrayDouble col(uint col_index) const {  ArrayDouble x = sub(0, d0 - 1, col_index, col_index); x.reshape(d0); return x; }
  ArrayDouble cols(uint start_col, uint end_col) const { return sub(0, d0 - 1, start_col, end_col - 1); }

  //overloaded to handle Jacobians
  void setMatrixBlock(const ArrayDouble& B, uint lo0, uint lo1);
  void setVectorBlock(const ArrayDouble& B, uint lo);
  void setBlockVector(const ArrayDouble& a, const ArrayDouble& b);
  void setBlockMatrix(const ArrayDouble& A, const ArrayDouble& B);

  /// @name special matrices
  double sparsity();
  SparseMatrix& sparse();
  const SparseMatrix& sparse() const;
  SparseVector& sparseVec();
  const SparseVector& sparseVec() const;
  RowShifted& rowShifted();
  const RowShifted& rowShifted() const;

  /// @name attached Jacobian
  std::unique_ptr<ArrayDouble> jac=0; ///< optional pointer to Jacobian, to enable autodiff
  void J_setId();
  ArrayDouble& J();
  ArrayDouble noJ() const;
  ArrayDouble J_reset();

  void write(std::ostream& os=stdCout(), const char* ELEMSEP=nullptr, const char* LINESEP=nullptr, const char* BRACKETS=nullptr, bool dimTag=false, bool binary=false) const;
};
#endif

}

//===========================================================================
///
/// @name alternative iterators
/// @{

namespace rai {

//struct ArrayIterationReverse_It {
//  double* p;
//  double& operator*() { return *p; } //access to value by user
//  void operator++() { p--; }
//  friend bool operator!=(const ArrayIterationReverse_It& i, const ArrayIterationReverse_It& j) { return i.p!=j.p; }
//};

//struct ArrayIterationReverse {
//  arr& x;
//  ArrayIterationReverse(arr& x):x(x) {}
//  ArrayIterationReverse_It begin() { return {x.p+x.N-1}; }
//  ArrayIterationReverse_It end() { return {x.p-1}; }
//};

//===========================================================================
/// @}
/// @name basic Array operators
/// @{

arr operator~(const arr& y); //op_transpose
arr operator-(const arr& y); //op_negative
arr operator^(const arr& y, const arr& z); //op_outerProduct
arr operator%(const arr& y, const arr& z); //op_indexWiseProduct
arr operator*(const arr& y, const arr& z); //op_innerProduct
arr operator*(const arr& y, double z);
arr operator*(double y, const arr& z);
arr operator/(int mustBeOne, const arr& z_tobeinverted);
arr operator/(const arr& y, double z);
arr operator/(const arr& y, const arr& z); //element-wise devision

arr& operator<<(arr& x, const double& y); //append
arr& operator<<(arr& x, const arr& y); //append

bool operator<(const arr& v, const arr& w);
std::istream& operator>>(std::istream& is, arr& x);
std::ostream& operator<<(std::ostream& os, const arr& x);

//element-wise update operators
#define UpdateOperator( op )        \
  arr& operator op (arr& x, const arr& y); \
  arr& operator op (arr& x, double y ); \
  arr& operator op (arr&& x, const arr& y); \
  arr& operator op (arr&& x, double y );
//UpdateOperator(|=)
//UpdateOperator(^=)
//UpdateOperator(&=)
//UpdateOperator(%=)
UpdateOperator(+=)
UpdateOperator(-=)
UpdateOperator(*=)
UpdateOperator(/=)
#undef UpdateOperator

//element-wise operators
#define BinaryOperator( op, updateOp)         \
  arr operator op(const arr& y, const arr& z); \
  arr operator op(double y, const arr& z);  \
  arr operator op(const arr& y, double z)
BinaryOperator(+, +=);
BinaryOperator(-, -=);
//BinaryOperator(% , *=);
//BinaryOperator(/ , /=);
#undef BinaryOperator

/// @} //name
} //namespace

//===========================================================================
/// @name basic Array functions
/// @{

#define UnaryFunction( func )           \
  arr func (const arr& y)
UnaryFunction(acos);
UnaryFunction(asin);
UnaryFunction(atan);
UnaryFunction(cos);
UnaryFunction(sin);
UnaryFunction(tan);
UnaryFunction(cosh);
UnaryFunction(sinh);
UnaryFunction(tanh);
UnaryFunction(acosh);
UnaryFunction(asinh);
UnaryFunction(atanh);
UnaryFunction(exp);
UnaryFunction(log);
UnaryFunction(log10);
UnaryFunction(sqrt);
UnaryFunction(cbrt);
UnaryFunction(ceil);
UnaryFunction(fabs);
UnaryFunction(floor);
UnaryFunction(sigm);
UnaryFunction(sign);
#undef UnaryFunction

#define BinaryFunction( func )            \
  arr func(const arr& y, const arr& z); \
  arr func(const arr& y, double z); \
  arr func(double y, const arr& z)
BinaryFunction(atan2);
BinaryFunction(pow);
BinaryFunction(fmod);
#undef BinaryFunction

//===========================================================================
/// @}
/// @name standard types
/// @{

typedef rai::Array<arr> arrA;
typedef rai::Array<arr*> arrL;

//===========================================================================
/// @}
/// @name constant non-arrays
/// @{

arr& getNoArr();
#define NoArr getNoArr()

//===========================================================================
/// @}
/// @name basic function types
/// @{

/// a generic vector-valued function \f$f:~x\mapsto y\in\mathbb{R}^d\f$, where return value may have Jacobian attached
typedef std::function<arr(const arr& x)> fct;
typedef std::function<arr(const arr& x)> VectorFunction;

/// a scalar function \f$f:~x\mapsto y\in\mathbb{R}\f$ with optional gradient and hessian
typedef std::function<double(arr& g, arr& H, const arr& x)> ScalarFunction;

/// a kernel function
struct KernelFunction {
  virtual double k(const arr& x1, const arr& x2, arr& g1=NoArr, arr& Hx1=NoArr) = 0;
  virtual ~KernelFunction() {}
};

//===========================================================================
/// @}
/// @name Octave/Matlab functions to generate special arrays
/// @{

/// return identity matrix
inline arr eye(uint d0, uint d1) { arr z(d0, d1);  z.setId();  return z; }
/// return identity matrix
inline arr eye(uint n) { return eye(n, n); }
/// return the ith standard basis vector (ith column of the Id matrix)
inline arr eyeVec(uint n, uint i) { arr z(n); z.setZero(); z(i)=1.; return z; }

/// return array of ones
inline arr ones(const uintA& d) {  arr z;  z.resize(d);  z=1.;  return z;  }
/// return VECTOR of ones
inline arr ones(uint n) { return ones(uintA{n}); }
/// return matrix of ones
inline arr ones(uint d0, uint d1) { return ones(uintA{d0, d1}); }

/// return array of zeros
inline arr zeros(const uintA& d) {  arr z;  z.resize(d);  z.setZero();  return z; }
/// return VECTOR of zeros
inline arr zeros(uint n) { return zeros(uintA{n}); }
/// return matrix of zeros
inline arr zeros(uint d0, uint d1) { return zeros(uintA{d0, d1}); }
/// return tensor of zeros
inline arr zeros(uint d0, uint d1, uint d2) { return zeros(uintA{d0, d1, d2}); }

/// return array of c's
inline arr consts(const double& c, const uintA& d)  {  arr z;  z.resize(d);  z.setConst(c);  return z; }
/// return VECTOR of c's
inline arr consts(const double& c, uint n) { return consts(c, uintA{n}); }
/// return matrix of c's
inline arr consts(const double& c, uint d0, uint d1) { return consts(c, uintA{d0, d1}); }
/// return tensor of c's
inline arr consts(const double& c, uint d0, uint d1, uint d2) { return consts(c, uintA{d0, d1, d2}); }

/// return array with uniform random numbers in [0, 1]
arr rand(const uintA& d);
/// return array with uniform random numbers in [0, 1]
inline arr rand(uint n) { return rand(uintA{n}); }
/// return array with uniform random numbers in [0, 1]
inline arr rand(uint d0, uint d1) { return rand(uintA{d0, d1}); }

inline arr rand(const arr& lo, const arr& up) { return lo + (up-lo)%rand(uintA{lo.N}); }

/// return tensor of c's
inline const double& random(const arr& range) { return range.rndElem(); }

/// return array with normal (Gaussian) random numbers
arr randn(const uintA& d);
/// return array with normal (Gaussian) random numbers
inline arr randn(uint n) { return randn(uintA{n}); }
/// return array with normal (Gaussian) random numbers
inline arr randn(uint d0, uint d1) { return randn(uintA{d0, d1}); }

/// return a grid with different lo/hi/steps in each dimension
arr grid(const arr& lo, const arr& hi, const uintA& steps);
/// return a 1D-grid
inline arr range(double lo, double hi, uint steps) { return rai::grid(1, lo, hi, steps).reshape(-1); }
//inline uintA range(uint n) { uintA r;  r.setStraightPerm(n);  return r; }

arr repmat(const arr& A, uint m, uint n);

//inline uintA randperm(uint n) {  uintA z;  z.setRandomPerm(n);  return z; }
inline arr linspace(double base, double limit, uint n) {  return rai::grid(1, base, limit, n).reshape(-1); }
arr logspace(double base, double limit, uint n);

void normalizeWithJac(arr& y, arr& J, double eps=0.);
void op_normalize(arr& y, double eps=0.);

//===========================================================================
/// @}
/// @name non-template functions //? needs most cleaning
/// @{

arr diag(double d, uint n);
void makeSymmetric(arr& A);
void transpose(arr& A);
uintA sampleMultinomial_SUS(const arr& p, uint n);
uint sampleMultinomial(const arr& p);
arr bootstrap(const arr& x);
void addDiag(arr& A, double d);

namespace rai {
/// use this to turn on Lapack routines [default true if RAI_LAPACK is defined]
extern bool useLapack;
}

uint svd(arr& U, arr& d, arr& V, const arr& A, bool sort2Dpoints=true);
void svd(arr& U, arr& V, const arr& A);
void pca(arr& Y, arr& v, arr& W, const arr& X, uint npc = 0);

arr  oneover(const arr& A); //element-wise reciprocal (devision, 1./A)

void mldivide(arr& X, const arr& A, const arr& b);

uint inverse(arr& Ainv, const arr& A);
arr  inverse(const arr& A);
uint inverse_SVD(arr& inverse, const arr& A);
void inverse_LU(arr& Xinv, const arr& X);
void inverse_SymPosDef(arr& Ainv, const arr& A);
inline arr inverse_SymPosDef(const arr& A) { arr Ainv; inverse_SymPosDef(Ainv, A); return Ainv; }
arr pseudoInverse(const arr& A, const arr& Winv=NoArr, double robustnessEps=1e-10);
void gaussFromData(arr& a, arr& A, const arr& X);
void rotationFromAtoB(arr& R, const arr& a, const arr& v);

double determinant(const arr& A);
double cofactor(const arr& A, uint i, uint j);

uintA getIndexTuple(uint i, const uintA& d);  //? that also exists inside of array!
void lognormScale(arr& P, double& logP, bool force=true);

void gnuplot(const arr& X, bool pauseMouse=false, bool persist=false, const char* PDFfile=nullptr);

void write_ppm(const byteA& img, const char* file_name, bool swap_rows=true);
void read_ppm(byteA& img, const char* file_name, bool swap_rows=true);
void add_alpha_channel(byteA& img, byte alpha);
void remove_alpha_channel(byteA& img);
void make_grey(byteA& img);
void make_RGB(byteA& img);
void make_RGB2BGRA(byteA& img);
void swap_RGB_BGR(byteA& img);
void flip_image(byteA& img);
void flip_image(floatA& img);
void image_halfResolution(byteA& img);
arr reshapeColor(const arr& col, int d0=-1);

void scanArrFile(const char* name);

arr finiteDifferenceGradient(const ScalarFunction& f, const arr& x, arr& Janalytic=NoArr, double eps=1e-8);
arr finiteDifferenceJacobian(const VectorFunction& f, const arr& _x, arr& Janalytic=NoArr, double eps=1e-8);
bool checkGradient(const ScalarFunction& f, const arr& x, double tolerance, bool verbose=false);
bool checkHessian(const ScalarFunction& f, const arr& x, double tolerance, bool verbose=false);
bool checkJacobian(const VectorFunction& f, const arr& x, double tolerance, bool verbose=false, const StringA& featureNames= {});
void boundClip(arr& y, const arr& bounds);
bool boundCheck(const arr& x, const arr& bounds, double eps=1e-3, bool verbose=true);

double NNinv(const arr& a, const arr& b, const arr& Cinv);
double logNNprec(const arr& a, const arr& b, double prec);
double logNNinv(const arr& a, const arr& b, const arr& Cinv);
double NN(const arr& a, const arr& b, const arr& C);
double logNN(const arr& a, const arr& b, const arr& C);

/// non-normalized!! Gaussian function (f(0)=1)
double NNNNinv(const arr& a, const arr& b, const arr& Cinv);
double NNNN(const arr& a, const arr& b, const arr& C);
double NNzeroinv(const arr& x, const arr& Cinv);
/// gradient of a Gaussian
double dNNinv(const arr& x, const arr& a, const arr& Ainv, arr& grad);
/// gradient of a non-normalized Gaussian
double dNNNNinv(const arr& x, const arr& a, const arr& Ainv, arr& grad);
double NNsdv(const arr& a, const arr& b, double sdv);
double NNzerosdv(const arr& x, double sdv);

rai::String singleString(const StringA& strs);

//===========================================================================
/// @}
/// @name template functions
/// @{

arr getDiag(const arr& y);
inline arr diag(const arr& x) {  arr y;  y.setDiag(x);  return y;  }
arr skew(const arr& x);
arr inverse2d(const arr& A);
arr replicate(const arr& A, uint d0);

void checkNan(const arr& x);

double entropy(const arr& v);
double normalizeDist(arr& v);
void makeConditional(arr& P);
void checkNormalization(arr& v, double tol=1e-10);
void eliminate(arr& x, const arr& y, uint d);
void eliminate(arr& x, const arr& y, uint d, uint e);
void eliminatePartial(arr& x, const arr& y, uint d);

double sqrDistance(const arr& v, const arr& w);
double maxDiff(const arr& v, const arr& w, uint* maxi=0);
double maxRelDiff(const arr& v, const arr& w, double tol);
//double sqrDistance(const arr& v, const arr& w, const array<bool>& mask);
double sqrDistance(const arr& g, const arr& v, const arr& w);
double euclideanDistance(const arr& v, const arr& w);
double metricDistance(const arr& g, const arr& v, const arr& w);

//min max
double min(const arr& x);
double max(const arr& x);
arr max(const arr& v, uint d);
arr min(const arr& v, uint d);
uint argmin(const arr& x);
uint argmax(const arr& x);
void argmax(uint& i, uint& j, const arr& x);
void argmax(uint& i, uint& j, uint& k, const arr& x);
double minDiag(const arr& v);
double absMax(const arr& x);
double absMin(const arr& x);

double sum(const arr& v);
arr sum(const arr& v, uint d);
double sumOfAbs(const arr& v);
double sumOfPos(const arr& v);
double sumOfSqr(const arr& v);
double length(const arr& v);
double product(const arr& v);

double trace(const arr& v);
double var(const arr& v);
arr mean(const arr& v);
arr covar(const arr& X);
arr stdDev(const arr& v);
void clip(const arr& x, double lo, double hi);

void op_transpose(arr& x, const arr& y);
void op_negative(arr& x, const arr& y);

void op_innerProduct(arr& x, const arr& y, const arr& z);
void op_outerProduct(arr& x, const arr& y, const arr& z);
void op_indexWiseProduct(arr& x, const arr& y, const arr& z);
void op_crossProduct(arr& x, const arr& y, const arr& z); //only for 3 x 3 or (3,n) x 3
inline arr crossProduct(const arr& y, const arr& z) { arr x; op_crossProduct(x, y, z); return x; }
double scalarProduct(const arr& v, const arr& w);
double scalarProduct(const arr& g, const arr& v, const arr& w);

arr elemWiseMin(const arr& v, const arr& w);
arr elemWiseMax(const arr& v, const arr& w);
arr elemWisemax(const arr& x, const double& y);
arr elemWisemax(const double& x, const arr& y);
arr elemWiseHinge(const arr& x);

void writeConsecutiveConstant(std::ostream& os, const arr& x);

//===========================================================================
/// @}
/// @name arrays interpreted as a set
/// @{

void rndInteger(arr& a, int low=0, int high=1, bool add=false);
void rndUniform(arr& a, double low=0., double high=1., bool add=false);
void rndNegLogUniform(arr& a, double low=0., double high=1., bool add=false);
void rndGauss(arr& a, double stdDev=1., bool add=false);
//void rndGauss(arr& a, bool add=false);
//arr& rndGauss(double stdDev, uint dim);
uint softMax(const arr& a, arr& soft, double beta);
inline arr sqr(const arr& y) { arr x; x.resizeAs(y); for(uint i=0; i<x.N; i++) x.p[i]=y.p[i]*y.p[i]; return x; }

//===========================================================================
/// @}
/// @name tensor functions
/// @{

void tensorCondNormalize(arr& X, int left);
void tensorCondMax(arr& X, uint left);
void tensorCondSoftMax(arr& X, uint left, double beta);
void tensorCond11Rule(arr& X, uint left, double rate);
void tensorCheckCondNormalization(const arr& X, uint left, double tol=1e-10);
void tensorCheckCondNormalization_with_logP(const arr& X, uint left, double logP, double tol=1e-10);

void tensorEquation(arr& X, const arr& A, const uintA& pickA, const arr& B, const uintA& pickB, uint sum=0);
void tensorMarginal(arr& Y, const arr& X, const uintA& Yid);
void tensorMaxMarginal(arr& Y, const arr& X, const uintA& Yid);
void tensorMarginal_old(arr& y, const arr& x, const uintA& xd, const uintA& ids);
void tensorMultiply(arr& X, const arr& Y, const uintA& Yid);
void tensorAdd(arr& X, const arr& Y, const uintA& Yid);
void tensorMultiply_old(arr& x, const arr& y, const uintA& d, const uintA& ids);
void tensorDivide(arr& X, const arr& Y, const uintA& Yid);

//===========================================================================
/// @}
/// @name low-level lapack interfaces
/// @{

void blas_Mv(arr& y, const arr& A, const arr& x);
void blas_MM(arr& X, const arr& A, const arr& B);
void blas_MsymMsym(arr& X, const arr& A, const arr& B);
void blas_A_At(arr& X, const arr& A);
void blas_At_A(arr& X, const arr& A);

void lapack_cholesky(arr& C, const arr& A);
uint lapack_SVD(arr& U, arr& d, arr& Vt, const arr& A);
void lapack_mldivide(arr& X, const arr& A, const arr& B);
void lapack_LU(arr& LU, const arr& A);
void lapack_RQ(arr& R, arr& Q, const arr& A);
void lapack_EigenDecomp(const arr& symmA, arr& Evals, arr& Evecs);
arr  lapack_kSmallestEigenValues_sym(const arr& A, uint k);
bool lapack_isPositiveSemiDefinite(const arr& symmA);
void lapack_inverseSymPosDef(arr& Ainv, const arr& A);
void lapack_choleskySymPosDef(arr& Achol, const arr& A);
double lapack_determinantSymPosDef(const arr& A);
inline arr lapack_inverseSymPosDef(const arr& A) { arr Ainv; lapack_inverseSymPosDef(Ainv, A); return Ainv; }
arr lapack_Ainv_b_sym(const arr& A, const arr& b);
void lapack_min_Ax_b(arr& x, const arr& A, const arr& b);
arr lapack_Ainv_b_symPosDef_givenCholesky(const arr& U, const arr& b);
arr lapack_Ainv_b_triangular(const arr& L, const arr& b);
arr eigen_Ainv_b(const arr& A, const arr& b);

//===========================================================================
/// @}
/// @name special matrices & packings
/// @{

namespace rai {

inline bool isSparse(const arr& X)       { return X.special && (X.special->type==SpecialArray::sparseMatrixST || X.special->type==SpecialArray::sparseVectorST); }
inline bool isSpecial(const arr& X)      { return X.special && X.special->type!=SpecialArray::ST_none; }
inline bool isNoArr(const arr& X)        { return X.special && X.special->type==SpecialArray::ST_NoArr; }
inline bool isEmptyShape(const arr& X)   { return X.special && X.special->type==SpecialArray::ST_EmptyShape; }
inline bool isRowShifted(const arr& X)   { return X.special && X.special->type==SpecialArray::RowShiftedST; }
inline bool isSparseMatrix(const arr& X) { return X.special && X.special->type==SpecialArray::sparseMatrixST; }
inline bool isSparseVector(const arr& X) { return X.special && X.special->type==SpecialArray::sparseVectorST; }
void special_copy(arr& x, const arr& a);
void special_write(ostream& os, const arr& x);

struct RowShifted : SpecialArray {
  arr& Z;           ///< references the array itself
  uint rowSize;     ///< the real width (the packed width is Z.d1; the height is Z.d0)
  uintA rowShift;   ///< amount of shift of each row (rowShift.N==Z.d0)
  uintA rowLen;     ///< number of non-zeros in the row
  uintA colPatches; ///< column-patch: (nd=2,d0=real_d1,d1=2) range of non-zeros in a COLUMN; starts with 'a', ends with 'b'-1
  bool symmetric;   ///< flag: if true, this stores a symmetric (banded) matrix: only the upper triangle

  RowShifted(arr& X);
  RowShifted(arr& X, RowShifted& aux);
  //access
  double elem(uint i, uint j) const; //access with natural coordinates
  double& elemNew(uint i, uint j); //access with natural coordinates
  double& entry(uint i, uint j) const; //access with memory coordinates
  arr memRef() const { arr x; x.referTo(Z.p, Z.N); x.reshape(Z.d0, rowSize); return x; }
  //manipulations
  void resize(uint d0, uint d1, uint _rowSize);
  void resizeCopy(uint d0, uint d1, uint n);
  void reshape(uint d0, uint d1);
  void reshift(); //shift all cols to start with non-zeros
  void computeColPatches(bool assumeMonotonic);
  void insRow(uint i) {
    uint real_d1 = Z.d1;
    Z.d1 = rowSize;
    Z.insRows(i, 1);
    Z.d1 = real_d1;
    rowShift.insert(i, 0);
    rowLen.insert(i, 0);
    colPatches.clear();
    symmetric=false;
  }

  //computations
  arr At_A();
  arr A_At();
  arr At_x(const arr& x);
  arr A_x(const arr& x);
  arr At();
  arr A_B(const arr& B) const;
  arr B_A(const arr& B) const;
  void rowWiseMult(const arr& a);

  void add(const arr& B, uint lo0=0, uint lo1=0, double coeff=1.);

  void write(std::ostream& os) const;

  arr unpack() const;

  void checkConsistency() const;
};
inline std::ostream& operator<<(std::ostream& os, const RowShifted& x) { x.write(os); return os; }

struct SparseVector: SpecialArray {
  arr& Z;      ///< references the array itself
  intA elems;  ///< for every non-zero (in memory order), the index
  SparseVector(arr& _Z);
  SparseVector(arr& _Z, const SparseVector& s);
  void resize(uint d0, uint n);
  double& entry(uint i, uint k);
  double& addEntry(int i);
  void setFromDense(const arr& x);
  arr unsparse();
};

struct SparseMatrix : SpecialArray {
  arr& Z;      ///< references the array itself, which linearly stores numbers
  intA elems;  ///< for every non-zero (in memory order), the (row,col) index tuple
  uintAA cols; ///< for every column, for every non-zero the (row,memory) index tuple
  uintAA rows; ///< for every row   , for every non-zero the (column,memory) index tuple

  SparseMatrix(arr& _Z);
  SparseMatrix(arr& _Z, const SparseMatrix& s);
  //access
  double& entry(uint i, uint j, uint k);
  double& elem(uint i, uint j);
  double& addEntry(int i, int j);
  arr getSparseRow(uint i) const;
  arr memRef() const { arr x; x.referTo(Z.p, Z.N); return x; }
  //construction
  void setFromDense(const arr& X);
  void setFromTriplets(const arr& T, uint d0, uint d1);
  void setupRowsCols();
  //manipulations
  SparseMatrix& resize(uint d0, uint d1, uint n);
  void resizeCopy(uint d0, uint d1, uint n);
  void reshape(uint d0, uint d1);
  void rowShift(int shift); //shift all rows to the right
  void colShift(int shift); //shift all cols downward
  //computations
  arr At_x(const arr& x, bool transpose=true) const;
  arr At_A() const;
  arr A_At() const;
  arr A_B(const arr& B) const;
  arr B_A(const arr& B) const;
  void transpose();
  void rowWiseMult(const arr& a);
  void add(const SparseMatrix& a, uint lo0=0, uint lo1=0, double coeff=1.);
  void add(const arr& B, uint lo0=0, uint lo1=0, double coeff=1.);
  void multRow(uint i, double a);
  arr unsparse();
  arr getTriplets() const;
  void checkConsistency() const;
};

arr unpack(const arr& X);
arr comp_At_A(const arr& A);
arr comp_A_At(const arr& A);
arr comp_At_x(const arr& A, const arr& x);
arr comp_At(const arr& A);
arr comp_A_x(const arr& A, const arr& x);
arr makeRowSparse(const arr& X);

#define UpdateOperator( op ) \
  void operator op (SparseMatrix& x, const SparseMatrix& y); \
  void operator op (SparseMatrix& x, double y ); \
  void operator op (RowShifted& x, const RowShifted& y); \
  void operator op (RowShifted& x, double y );
UpdateOperator(|=)
UpdateOperator(^=)
UpdateOperator(&=)
UpdateOperator(+=)
UpdateOperator(-=)
UpdateOperator(*=)
UpdateOperator(/=)
UpdateOperator(%=)
#undef UpdateOperator

}//namespace rai

//===========================================================================
//
// conv with Eigen
//

#ifdef RAI_EIGEN

#include <Eigen/Dense>

arr conv_eigen2arr(const Eigen::MatrixXd& in);
Eigen::MatrixXd conv_arr2eigen(const arr& in);

#endif //RAI_EIGEN

//===========================================================================
//
// implementations
//

//#include "array.ipp"
