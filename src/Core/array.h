/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    ---------------------------------------------------------------  */

/// @file
/// @ingroup group_array
/// @addtogroup group_array
/// @{

#ifndef MT_array_h
#define MT_array_h

#include <iostream>
#include <stdint.h>
#include <string.h>

#define FOR1D(x, i)   for(i=0;i<x.N;i++)
#define FOR1D_DOWN(x, i)   for(i=x.N;i--;)
#define FOR2D(x, i, j) for(i=0;i<x.d0;i++) for(j=0;j<x.d1;j++)
#define FOR3D(x, i, j, k) for(i=0;i<x.d0;i++) for(j=0;j<x.d1;j++) for(k=0;k<x.d2;k++)

//-- don't require previously defined iterators
#define for_list(Type, it, X)     Type *it=NULL; for(uint it##_COUNT=0;   it##_COUNT<X.N && ((it=X(it##_COUNT)) || true); it##_COUNT++)
#define for_list_rev(Type, it, X) Type *it=NULL; for(uint it##_COUNT=X.N; it##_COUNT--   && ((it=X(it##_COUNT)) || true); )

#define ARR ARRAY<double> ///< write ARR(1., 4., 5., 7.) to generate a double-Array
#define TUP ARRAY<uint> ///< write TUP(1, 2, 3) to generate a uint-Array

typedef unsigned char byte;
typedef unsigned int uint;

//-- global memory information and options
namespace MT {
extern bool useLapack;
extern const bool lapackSupported;
extern uint64_t globalMemoryTotal, globalMemoryBound;
extern bool globalMemoryStrict;
extern const char* arrayElemsep;
extern const char* arrayLinesep;
extern const char* arrayBrackets;
struct FileToken;
} //namespace

//===========================================================================
//
// Array class
//

namespace MT {

/// @addtogroup group_array
/// @{

/** Simple array container to store arbitrary-dimensional arrays (tensors).
  Can buffer more memory than necessary for faster
  resize; enables non-const reference of subarrays; enables fast
  memove for elementary types; implements many standard
  array/matrix/tensor operations. Please see the fully public attributes at the
  bottom of this page -- everthing is meant to be perfectly
  transparent. Interfacing with ordinary C-buffers is simple.
  Please see also the reference for the \ref array.h
  header, which contains lots of functions that can be applied on
  Arrays. */
template<class T> struct Array {
  typedef bool (*ElemCompare)(const T& a, const T& b);
  
  T *p;     ///< the pointer on the linear memory allocated
  uint N;   ///< number of elements
  uint nd;  ///< number of dimensions
  uint d0,d1,d2;  ///< 0th, 1st, 2nd dim
  uint *d;  ///< pointer to dimensions (for nd<=3 points to d0)
  uint M;   ///< size of actually allocated memory (may be greater than N)
  bool reference; ///< true if this refers to some external memory
  
  static int  sizeT;   ///< constant for each type T: stores the sizeof(T)
  static char memMove; ///< constant for each type T: decides whether memmove can be used instead of individual copies

  //-- special: arrays can be sparse/packed/etc and augmented with aux data to support this
  enum SpecialType { noneST, hasCarrayST, sparseST, diagST, RowShiftedPackedMatrixST, CpointerST };
  SpecialType special;
  void *aux; ///< arbitrary auxiliary data, depends on special
  
  /// @name constructors
  Array();
  Array(const Array<T>& a);                 //copy constructor
  Array(const Array<T>& a, uint i);         //reference constructor
  Array(const Array<T>& a, uint i, uint j); //reference constructor
  Array(const Array<T>& a, uint i, uint j, uint k); //reference constructor
  explicit Array(uint D0);
  explicit Array(uint D0, uint D1);
  explicit Array(uint D0, uint D1, uint D2);
  explicit Array(const T* p, uint size);    //reference!
  Array(std::initializer_list<T> list);
  Array(MT::FileToken&); //read from a file
  ~Array();
  
  Array<T>& operator=(const T& v);
  Array<T>& operator=(const Array<T>& a);

  /// @name iterators
  typedef T* iterator;
  typedef const T* const_iterator;
  iterator begin() { return p; }
  iterator end() { return p+N; }
  const_iterator begin() const { return p; }
  const_iterator end() const { return p+N; }

  /// @name resizing
  Array<T>& resize(uint D0);
  Array<T>& resize(uint D0, uint D1);
  Array<T>& resize(uint D0, uint D1, uint D2);
  Array<T>& resize(uint ND, uint *dim);
  Array<T>& resize(const Array<uint> &dim);
  Array<T>& reshape(uint D0);
  Array<T>& reshape(uint D0, uint D1);
  Array<T>& reshape(uint D0, uint D1, uint D2);
  Array<T>& reshape(uint ND, uint *dim);
  Array<T>& reshape(const Array<uint> &dim);
  Array<T>& resizeCopy(uint D0);
  Array<T>& resizeCopy(uint D0, uint D1);
  Array<T>& resizeCopy(uint D0, uint D1, uint D2);
  Array<T>& resizeCopy(uint ND, uint *dim);
  Array<T>& resizeCopy(const Array<uint> &dim);
  Array<T>& resizeAs(const Array<T>& a);
  Array<T>& reshapeAs(const Array<T>& a);
  Array<T>& resizeCopyAs(const Array<T>& a);
  Array<T>& flatten();
  Array<T>& dereference();

  /// @name initializing/assigning entries
  void clear();
  void setZero(byte zero=0);
  void setUni(const T& scalar, int d=-1);
  void setId(int d=-1);
  void setDiag(const T& scalar, int d=-1);
  void setDiag(const Array<T>& vector);
  void setBlockMatrix(const Array<T>& A, const Array<T>& B, const Array<T>& C, const Array<T>& D);
  void setBlockMatrix(const Array<T>& A, const Array<T>& B);
  void setBlockVector(const Array<T>& a, const Array<T>& b);
  void setMatrixBlock(const Array<T>& B, uint lo0, uint lo1);
  void setVectorBlock(const Array<T>& B, uint lo);
  void setStraightPerm(int n=-1);
  void setReversePerm(int n=-1);
  void setRandomPerm(int n=-1);
  void setCarray(const T *buffer, uint D0);
  void setCarray(const T **buffer, uint D0, uint D1);
  void referTo(const T *buffer, uint n);
  void referTo(const Array<T>& a);
  void referToSubRange(const Array<T>& a, int i, int I);
  void referToSubDim(const Array<T>& a, uint dim);
  void referToSubDim(const Array<T>& a, uint i, uint j);
  void referToSubDim(const Array<T>& a, uint i, uint j, uint k);
  void takeOver(Array<T>& a);  //a becomes a reference to its previously owned memory!
  void swap(Array<T>& a);      //the two arrays swap their contents!
  void setGrid(uint dim, T lo, T hi, uint steps);
  
  /// @name access by reference (direct memory access)
  T& elem(uint i) const;
  T& scalar() const;
  T& last() const;
  T& rndElem() const;
  T& operator()(uint i) const;
  T& operator()(uint i, uint j) const;
  T& operator()(uint i, uint j, uint k) const;
  T& operator()(const Array<uint> &I) const;
  Array<T> operator[](uint i) const;     // calls referToSubDim(*this, i)
  Array<T> subDim(uint i, uint j) const; // calls referToSubDim(*this, i, j)
  Array<T> subDim(uint i, uint j, uint k) const; // calls referToSubDim(*this, i, j, k)
  Array<T> subRange(int i, int I) const; // calls referToSubRange(*this, i, I)
  Array<T>& operator()();
  T** getCarray(Array<T*>& Cpointers) const;
  
  /// @name access by copy
  uint dim(uint k) const;
  Array<uint> getDim() const;
  Array<T> sub(int i, int I) const;
  Array<T> sub(int i, int I, int j, int J) const;
  Array<T> sub(int i, int I, int j, int J, int k, int K) const;
  Array<T> sub(int i, int I, Array<uint> cols) const;
  Array<T> row(uint row_index) const;
  Array<T> rows(uint start_row, uint end_row) const;
  Array<T> col(uint col_index) const;
  Array<T> cols(uint start_col, uint end_col) const;

  void getMatrixBlock(Array<T>& B, uint lo0, uint lo1) const;
  void getVectorBlock(Array<T>& B, uint lo) const;
  void copyInto(T *buffer) const;
  void copyInto2D(T **buffer) const;
  T& min() const;
  T& max() const;
  void minmax(T& minVal, T& maxVal) const;
  uint minIndex() const;
  uint maxIndex() const;
  void maxIndeces(uint& m1, uint& m2) const; //best and 2nd best
  void maxIndex(uint& i, uint& j) const;
  void maxIndex(uint& i, uint& j, uint& k) const;
  int findValue(const T& x) const;
  void findValues(MT::Array<uint>& indices, const T& x) const;
  bool contains(const T& x) const { return findValue(x)!=-1; }
  bool containsDoubles() const;
  uint getMemsize() const;
  void getIndexTuple(Array<uint> &I, uint i) const;
  
  /// @name appending etc
  T& append();
  T& append(const T& x);
  void append(const Array<T>& x);
  void append(const T *p, uint n);
  void prepend(const T& x){ insert(0,x); }
  void replicate(uint copies);
  void insert(uint i, const T& x);
  void replace(uint i, uint n, const Array<T>& x);
  void remove(uint i, uint n=1);
  void removePerm(uint i);          //more efficient for sets, works also for non-memMove arrays
  void removeValue(const T& x);
  bool removeValueSafe(const T& x); //? same as if((i=findValue(x))!=-1) remove[Perm](i);
  void removeAllValues(const T& x);
  void delRows(uint i, uint k=1);
  void delColumns(uint i, uint k=1);
  void insRows(uint i, uint k=1);
  void insColumns(uint i, uint k=1);
  void resizeDim(uint k, uint dk);
  void setAppend(const T& x); //? same as if(findValue(x)==-1) append(x)
  void setAppend(const Array<T>& x);
  T popFirst();
  T popLast();
  
  /// @name sorting and permuting this array
  void sort(ElemCompare comp);
  bool isSorted(ElemCompare comp) const;
  uint rankInSorted(const T& x, ElemCompare comp) const;
  int findValueInSorted(const T& x, ElemCompare comp) const;
  uint insertInSorted(const T& x, ElemCompare comp);
  uint setAppendInSorted(const T& x, ElemCompare comp);
  void removeValueInSorted(const T& x, ElemCompare comp);
  void reverse();
  void reverseRows();
  void permute(uint i, uint j);
  void permute(const Array<uint>& permutation);
  void permuteInv(const Array<uint>& permutation);
  void permuteRows(const Array<uint>& permutation);
  void permuteRandomly();
  void shift(int offset, bool wrapAround=true);
  
  /// @name sparse matrices [TODO: move outside, use 'special']
  double sparsity();
  void makeSparse();
  
  /// @name I/O
  void write(std::ostream& os=std::cout, const char *ELEMSEP=NULL, const char *LINESEP=NULL, const char *BRACKETS=NULL, bool dimTag=false, bool binary=false) const;
  void read(std::istream& is);
  void writeTagged(std::ostream& os, const char* tag, bool binary=false) const;
  bool readTagged(std::istream& is, const char *tag);
  void writeTagged(const char* filename, const char* tag, bool binary=false) const;
  bool readTagged(const char* filename, const char *tag);
  void writeDim(std::ostream& os=std::cout) const;
  void readDim(std::istream& is);
  void writeRaw(std::ostream& os) const;
  void readRaw(std::istream& is);
  void writeWithIndex(std::ostream& os=std::cout) const;
  const Array<T>& ioraw() const;
  const char* prt(); //gdb pretty print
  
  /// @name kind of private
  void resizeMEM(uint n, bool copy);
  void freeMEM();
  void resetD();
  void init();
};


//===========================================================================
/// @name basic Array operators
/// @{
template<class T> Array<T> operator~(const Array<T>& y); //transpose
template<class T> Array<T> operator-(const Array<T>& y); //negative
template<class T> Array<T> operator^(const Array<T>& y, const Array<T>& z); //outer product
template<class T> Array<T> operator%(const Array<T>& y, const Array<T>& z); //index-wise product
template<class T> Array<T> operator*(const Array<T>& y, const Array<T>& z); //inner product
template<class T> Array<T> operator*(const Array<T>& y, T z);
template<class T> Array<T> operator*(T y, const Array<T>& z);
template<class T> bool operator==(const Array<T>& v, const Array<T>& w);
template<class T> bool operator==(const Array<T>& v, const T *w);
template<class T> bool operator!=(const Array<T>& v, const Array<T>& w);
template<class T> bool operator<(const Array<T>& v, const Array<T>& w);
template<class T> std::istream& operator>>(std::istream& is, Array<T>& x);
template<class T> std::ostream& operator<<(std::ostream& os, const Array<T>& x);
template<class T> Array<T>& operator<<(Array<T>& x, const char* str);

//element-wise update operators
#ifndef SWIG
#define UpdateOperator( op )        \
  template<class T> Array<T>& operator op (Array<T>& x, const Array<T>& y); \
  template<class T> Array<T>& operator op ( Array<T>& x, T y )
UpdateOperator(|=);
UpdateOperator(^=);
UpdateOperator(&=);
UpdateOperator(+=);
UpdateOperator(-=);
UpdateOperator(*=);
UpdateOperator(/=);
UpdateOperator(%=);
#undef UpdateOperator
#endif

//element-wise operators
#define BinaryOperator( op, updateOp)         \
  template<class T> Array<T> operator op(const Array<T>& y, const Array<T>& z); \
  template<class T> Array<T> operator op(T y, const Array<T>& z);  \
  template<class T> Array<T> operator op(const Array<T>& y, T z)
BinaryOperator(+ , +=);
BinaryOperator(- , -=);
//BinaryOperator(% , *=);
BinaryOperator(/ , /=);
#undef BinaryOperator

/// @} //name
/// @} //group
} //namespace


//===========================================================================
/// @name standard types
/// @{

typedef MT::Array<double> arr;
typedef MT::Array<float>  arrf;
typedef MT::Array<double> doubleA;
typedef MT::Array<float>  floatA;
typedef MT::Array<uint>   uintA;
typedef MT::Array<int>    intA;
typedef MT::Array<char>   charA;
typedef MT::Array<byte>   byteA;
typedef MT::Array<bool>   boolA;
typedef MT::Array<uint16_t>   uint16A;
typedef MT::Array<uint32_t>   uint32A;
typedef MT::Array<const char*>  CstrList;
typedef MT::Array<arr*>   arrL;
typedef MT::Array<arr>    arrA;

namespace MT { struct String; }
typedef MT::Array<MT::String> StringA;
typedef MT::Array<MT::String*> StringL;

//===========================================================================
/// @}
/// @name constant arrays
/// @{

extern arr& NoArr; //this is a pointer to NULL!!!! I use it for optional arguments
extern uintA& NoUintA; //this is a pointer to NULL!!!! I use it for optional arguments

//===========================================================================
/// @}
/// @name function types
/// @{

/// a scalar function \f$f:~x\mapsto y\in\mathbb{R}\f$ with optional gradient and hessian
struct ScalarFunction {
  virtual double fs(arr& g, arr& H, const arr& x) = 0;
  virtual ~ScalarFunction(){}
};

/// a vector function \f$f:~x\mapsto y\in\mathbb{R}^d\f$ with optional Jacobian
struct VectorFunction {
  virtual void fv(arr& y, arr& J, const arr& x) = 0; ///< returning a vector y and (optionally, if NoArr) Jacobian J for x
  virtual ~VectorFunction(){}
};

/// a kernel function
struct KernelFunction {
  virtual double k(const arr& x1, const arr& x2, arr& g1=NoArr, arr& g2=NoArr) = 0;
  virtual ~KernelFunction(){}
};



//===========================================================================
template<class T> MT::Array<T> ARRAY() {                                    MT::Array<T> z(0); return z; }
template<class T> MT::Array<T> ARRAY(const T& i) {                                    MT::Array<T> z(1); z(0)=i; return z; }
template<class T> MT::Array<T> ARRAY(const T& i, const T& j) {                               MT::Array<T> z(2); z(0)=i; z(1)=j; return z; }
template<class T> MT::Array<T> ARRAY(const T& i, const T& j, const T& k) {                          MT::Array<T> z(3); z(0)=i; z(1)=j; z(2)=k; return z; }
template<class T> MT::Array<T> ARRAY(const T& i, const T& j, const T& k, const T& l) {                     MT::Array<T> z(4); z(0)=i; z(1)=j; z(2)=k; z(3)=l; return z; }
template<class T> MT::Array<T> ARRAY(const T& i, const T& j, const T& k, const T& l, const T& m) {                MT::Array<T> z(5); z(0)=i; z(1)=j; z(2)=k; z(3)=l; z(4)=m; return z; }
template<class T> MT::Array<T> ARRAY(const T& i, const T& j, const T& k, const T& l, const T& m, const T& n) {           MT::Array<T> z(6); z(0)=i; z(1)=j; z(2)=k; z(3)=l; z(4)=m; z(5)=n; return z; }
template<class T> MT::Array<T> ARRAY(const T& i, const T& j, const T& k, const T& l, const T& m, const T& n, const T& o) {      MT::Array<T> z(7); z(0)=i; z(1)=j; z(2)=k; z(3)=l; z(4)=m; z(5)=n; z(6)=o; return z; }
template<class T> MT::Array<T> ARRAY(const T& i, const T& j, const T& k, const T& l, const T& m, const T& n, const T& o, const T& p) { MT::Array<T> z(8); z(0)=i; z(1)=j; z(2)=k; z(3)=l; z(4)=m; z(5)=n; z(6)=o; z(7)=p; return z; }
template<class T> MT::Array<T> ARRAY(const T& i, const T& j, const T& k, const T& l, const T& m, const T& n, const T& o, const T& p, const T& q) { MT::Array<T> z(9); z(0)=i; z(1)=j; z(2)=k; z(3)=l; z(4)=m; z(5)=n; z(6)=o; z(7)=p; z(8)=q; return z; }

template<class T> MT::Array<T*> LIST() {                                    MT::Array<T*> z(0); return z; }
template<class T> MT::Array<T*> LIST(const T& i) {                                    MT::Array<T*> z(1); z(0)=(T*)&i; return z; }
template<class T> MT::Array<T*> LIST(const T& i, const T& j) {                               MT::Array<T*> z(2); z(0)=(T*)&i; z(1)=(T*)&j; return z; }
template<class T> MT::Array<T*> LIST(const T& i, const T& j, const T& k) {                          MT::Array<T*> z(3); z(0)=(T*)&i; z(1)=(T*)&j; z(2)=(T*)&k; return z; }
template<class T> MT::Array<T*> LIST(const T& i, const T& j, const T& k, const T& l) {                     MT::Array<T*> z(4); z(0)=(T*)&i; z(1)=(T*)&j; z(2)=(T*)&k; z(3)=(T*)&l; return z; }
template<class T> MT::Array<T*> LIST(const T& i, const T& j, const T& k, const T& l, const T& m) {                MT::Array<T*> z(5); z(0)=(T*)&i; z(1)=(T*)&j; z(2)=(T*)&k; z(3)=(T*)&l; z(4)=(T*)&m; return z; }
template<class T> MT::Array<T*> LIST(const T& i, const T& j, const T& k, const T& l, const T& m, const T& n) {           MT::Array<T*> z(6); z(0)=(T*)&i; z(1)=(T*)&j; z(2)=(T*)&k; z(3)=(T*)&l; z(4)=(T*)&m; z(5)=(T*)&n; return z; }
template<class T> MT::Array<T*> LIST(const T& i, const T& j, const T& k, const T& l, const T& m, const T& n, const T& o) {      MT::Array<T*> z(7); z(0)=(T*)&i; z(1)=(T*)&j; z(2)=(T*)&k; z(3)=(T*)&l; z(4)=(T*)&m; z(5)=(T*)&n; z(6)=(T*)&o; return z; }
template<class T> MT::Array<T*> LIST(const T& i, const T& j, const T& k, const T& l, const T& m, const T& n, const T& o, const T& p) { MT::Array<T*> z(8); z(0)=(T*)&i; z(1)=(T*)&j; z(2)=(T*)&k; z(3)=(T*)&l; z(4)=(T*)&m; z(5)=(T*)&n; z(6)=(T*)&o; z(7)=(T*)&p; return z; }

MT::Array<MT::String> STRINGS();
MT::Array<MT::String> STRINGS(const char* s0);
MT::Array<MT::String> STRINGS(const char* s0, const char* s1);
MT::Array<MT::String> STRINGS(const char* s0, const char* s1, const char* s2);

#define STRINGS_0()           (ARRAY<MT::String>())
#define STRINGS_1(s0)         (ARRAY<MT::String>(STRING(s0)))
#define STRINGS_2(s0, s1)     (ARRAY<MT::String>(STRING(s0),STRING(s1)))
#define STRINGS_3(s0, s1, s2) (ARRAY<MT::String>(STRING(s0),STRING(s1),STRING(s2)))


//===========================================================================
/// @}
/// @name Octave/Matlab functions to generate special arrays
/// @{

/// return identity matrix
inline arr eye(uint d0, uint d1) { arr z;  z.resize(d0, d1);  z.setId();  return z; }
/// return identity matrix
inline arr eye(uint n) { return eye(n, n); }

/// return matrix of ones
inline arr ones(const uintA& d) {  arr z;  z.resize(d);  z=1.;  return z;  }
/// return VECTOR of ones
inline arr ones(uint n) { return ones(TUP(n)); }
/// return matrix of ones
inline arr ones(uint d0, uint d1) { return ones(TUP(d0, d1)); }

/// return matrix of zeros
inline arr zeros(const uintA& d) {  arr z;  z.resize(d);  z.setZero();  return z; }
/// return VECTOR of zeros
inline arr zeros(uint n) { return zeros(TUP(n)); }
/// return matrix of zeros
inline arr zeros(uint d0, uint d1) { return zeros(TUP(d0, d1)); }
/// return tensor of zeros
inline arr zeros(uint d0, uint d1, uint d2) { return zeros(TUP(d0, d1, d2)); }

arr repmat(const arr& A, uint m, uint n);

/// return array with random numbers in [0, 1]
arr rand(const uintA& d);
/// return array with random numbers in [0, 1]
inline arr rand(uint n) { return rand(TUP(n, n)); }
/// return array with random numbers in [0, 1]
inline arr rand(uint d0, uint d1) { return rand(TUP(d0, d1)); }

/// return array with normal (Gaussian) random numbers
arr randn(const uintA& d);
/// return array with normal (Gaussian) random numbers
inline arr randn(uint n) { return randn(TUP(n, n)); }
/// return array with normal (Gaussian) random numbers
inline arr randn(uint d0, uint d1) { return randn(TUP(d0, d1)); }

inline double max(const arr& x) { return x.max(); }
inline double min(const arr& x) { return x.min(); }
inline uint argmax(const arr& x) { return x.maxIndex(); }
inline uint argmin(const arr& x) { return x.minIndex(); }

inline uintA randperm(uint n) {  uintA z;  z.setRandomPerm(n);  return z; }
inline arr linspace(double base, double limit, uint n) {  arr z;  z.setGrid(1, base, limit, n);  return z;  }
arr logspace(double base, double limit, uint n);


//===========================================================================
/// @}
/// @name non-template functions //? needs most cleaning
/// @{

arr diag(double d, uint n);
void makeSymmetric(arr& A);
void transpose(arr& A);
void SUS(const arr& p, uint n, uintA& s);
uint SUS(const arr& p);

namespace MT {
/// use this to turn on Lapack routines [default true if MT_LAPACK is defined]
extern bool useLapack;
}

uint svd(arr& U, arr& d, arr& V, const arr& A, bool sort=true);
void svd(arr& U, arr& V, const arr& A);
void pca(arr &Y, arr &v, arr &W, const arr &X, uint npc = 0);

void mldivide(arr& X, const arr& A, const arr& b);

uint inverse(arr& Ainv, const arr& A);
arr  inverse(const arr& A);
uint inverse_SVD(arr& inverse, const arr& A);
void inverse_LU(arr& Xinv, const arr& X);
void inverse_SymPosDef(arr& Ainv, const arr& A);
inline arr inverse_SymPosDef(const arr& A){ arr Ainv; inverse_SymPosDef(Ainv, A); return Ainv; }
arr pseudoInverse(const arr& A, const arr& Winv=NoArr, double robustnessEps=1e-10);
void gaussFromData(arr& a, arr& A, const arr& X);
void rotationFromAtoB(arr& R, const arr& a, const arr& v);

double determinant(const arr& A);
double cofactor(const arr& A, uint i, uint j);

//void getIndexTuple(uintA &I, uint i, const uintA &d);  //? that also exists inside of array!
void lognormScale(arr& P, double& logP, bool force=true);

void gnuplot(const arr& X);
//these are obsolete, use catCol instead
//void write(const arr& X, const char *filename, const char *ELEMSEP=" ", const char *LINESEP="\n ", const char *BRACKETS="  ", bool dimTag=false, bool binary=false);
//void write(std::ostream& os, const arrL& X, const char *ELEMSEP=" ", const char *LINESEP="\n ", const char *BRACKETS="  ", bool dimTag=false, bool binary=false);
void write(const arrL& X, const char *filename, const char *ELEMSEP=" ", const char *LINESEP="\n ", const char *BRACKETS="  ", bool dimTag=false, bool binary=false);


void write_ppm(const byteA &img, const char *file_name, bool swap_rows=false);
void read_ppm(byteA &img, const char *file_name, bool swap_rows=false);
void add_alpha_channel(byteA &img, byte alpha);
void make_grey(byteA &img);
void make_RGB(byteA &img);
void flip_image(byteA &img);

void scanArrFile(const char* name);

bool checkGradient(ScalarFunction &f, const arr& x, double tolerance);
bool checkHessian(ScalarFunction &f, const arr& x, double tolerance);
bool checkJacobian(VectorFunction &f, const arr& x, double tolerance);

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


//===========================================================================
/// @}
/// @name template functions
/// @{

//NOTES:
// -- past-tense names do not modify the array, but return variants
// -- more methods should return an array instead of have a returned parameter...

template<class T> MT::Array<T> vectorShaped(const MT::Array<T>& x) {  MT::Array<T> y;  y.referTo(x);  y.reshape(y.N);  return y;  }
template<class T> void transpose(MT::Array<T>& x, const MT::Array<T>& y);
template<class T> void negative(MT::Array<T>& x, const MT::Array<T>& y);
template<class T> MT::Array<T> getDiag(const MT::Array<T>& y);
template<class T> MT::Array<T> diag(const MT::Array<T>& x) {  MT::Array<T> y;  y.setDiag(x);  return y;  }
template<class T> MT::Array<T> skew(const MT::Array<T>& x);
template<class T> void inverse2d(MT::Array<T>& Ainv, const MT::Array<T>& A);
template<class T> MT::Array<T> replicate(const MT::Array<T>& A, uint d0);

template<class T> uintA size(const MT::Array<T>& x) { return x.getDim(); }
template<class T> void checkNan(const MT::Array<T>& x);

template<class T> T entropy(const MT::Array<T>& v);
template<class T> T normalizeDist(MT::Array<T>& v);
template<class T> void makeConditional(MT::Array<T>& P);
template<class T> void checkNormalization(MT::Array<T>& v, double tol);
template<class T> void checkNormalization(MT::Array<T>& v) { checkNormalization(v, 1e-10); }
template<class T> void eliminate(MT::Array<T>& x, const MT::Array<T>& y, uint d);
template<class T> void eliminate(MT::Array<T>& x, const MT::Array<T>& y, uint d, uint e);
template<class T> void eliminatePartial(MT::Array<T>& x, const MT::Array<T>& y, uint d);

#ifndef SWIG
template<class T> T sqrDistance(const MT::Array<T>& v, const MT::Array<T>& w);
template<class T> T maxDiff(const MT::Array<T>& v, const MT::Array<T>& w, uint *maxi=0);
template<class T> T maxRelDiff(const MT::Array<T>& v, const MT::Array<T>& w, T tol);
//template<class T> T sqrDistance(const MT::Array<T>& v, const MT::Array<T>& w, const MT::Array<bool>& mask);
template<class T> T sqrDistance(const MT::Array<T>& g, const MT::Array<T>& v, const MT::Array<T>& w);
template<class T> T euclideanDistance(const MT::Array<T>& v, const MT::Array<T>& w);
template<class T> T metricDistance(const MT::Array<T>& g, const MT::Array<T>& v, const MT::Array<T>& w);

template<class T> T sum(const MT::Array<T>& v);
template<class T> T scalar(const MT::Array<T>& v);
template<class T> MT::Array<T> sum(const MT::Array<T>& v, uint d);
template<class T> T sumOfAbs(const MT::Array<T>& v);
template<class T> T sumOfSqr(const MT::Array<T>& v);
template<class T> T length(const MT::Array<T>& v);
template<class T> T product(const MT::Array<T>& v);

template<class T> T trace(const MT::Array<T>& v);
template<class T> T var(const MT::Array<T>& v);
template<class T> T minDiag(const MT::Array<T>& v);
template<class T> T absMax(const MT::Array<T>& x);
template<class T> T absMin(const MT::Array<T>& x);

template<class T> void innerProduct(MT::Array<T>& x, const MT::Array<T>& y, const MT::Array<T>& z);
template<class T> void outerProduct(MT::Array<T>& x, const MT::Array<T>& y, const MT::Array<T>& z);
template<class T> void indexWiseProduct(MT::Array<T>& x, const MT::Array<T>& y, const MT::Array<T>& z);
template<class T> T scalarProduct(const MT::Array<T>& v, const MT::Array<T>& w);
template<class T> T scalarProduct(const MT::Array<T>& g, const MT::Array<T>& v, const MT::Array<T>& w);
template<class T> MT::Array<T> diagProduct(const MT::Array<T>& v, const MT::Array<T>& w);

template<class T> MT::Array<T> elemWiseMin(const MT::Array<T>& v, const MT::Array<T>& w);
template<class T> MT::Array<T> elemWiseMax(const MT::Array<T>& v, const MT::Array<T>& w);
template<class T> MT::Array<T> elemWisemax(const MT::Array<T>& x,const T& y);
template<class T> MT::Array<T> elemWisemax(const T& x,const MT::Array<T>& y);


//===========================================================================
/// @}
/// @name concatenating arrays together
/// @{

template<class T> MT::Array<T> cat(const MT::Array<T>& y, const MT::Array<T>& z) { MT::Array<T> x; x.append(y); x.append(z); return x; }
template<class T> MT::Array<T> cat(const MT::Array<T>& y, const MT::Array<T>& z, const MT::Array<T>& w) { MT::Array<T> x; x.append(y); x.append(z); x.append(w); return x; }
template<class T> MT::Array<T> cat(const MT::Array<T>& a, const MT::Array<T>& b, const MT::Array<T>& c, const MT::Array<T>& d) { MT::Array<T> x; x.append(a); x.append(b); x.append(c); x.append(d); return x; }
template<class T> MT::Array<T> cat(const MT::Array<T>& a, const MT::Array<T>& b, const MT::Array<T>& c, const MT::Array<T>& d, const MT::Array<T>& e) { MT::Array<T> x; x.append(a); x.append(b); x.append(c); x.append(d); x.append(e); return x; }
template<class T> MT::Array<T> catCol(const MT::Array<MT::Array<T>*>& X);
template<class T> MT::Array<T> catCol(const MT::Array<T>& a, const MT::Array<T>& b){ return catCol(LIST<MT::Array<T> >(a,b)); }
template<class T> MT::Array<T> catCol(const MT::Array<T>& a, const MT::Array<T>& b, const MT::Array<T>& c){ return catCol(LIST<MT::Array<T> >(a,b,c)); }
template<class T> MT::Array<T> catCol(const MT::Array<T>& a, const MT::Array<T>& b, const MT::Array<T>& c, const MT::Array<T>& d){ return catCol(LIST<MT::Array<T> >(a,b,c,d)); }


//===========================================================================
/// @}
/// @name arrays interpreted as a set
/// @{

template<class T> void setUnion(MT::Array<T>& x, const MT::Array<T>& y, const MT::Array<T>& z);
template<class T> void setSection(MT::Array<T>& x, const MT::Array<T>& y, const MT::Array<T>& z);
template<class T> MT::Array<T> setUnion(const MT::Array<T>& y, const MT::Array<T>& z) { MT::Array<T> x; setUnion(x, y, z); return x; }
template<class T> MT::Array<T> setSection(const MT::Array<T>& y, const MT::Array<T>& z) { MT::Array<T> x; setSection(x, y, z); return x; }
template<class T> void setMinus(MT::Array<T>& x, const MT::Array<T>& y);
template<class T> uint numberSharedElements(const MT::Array<T>& x, const MT::Array<T>& y);
template<class T> void rndInteger(MT::Array<T>& a, int low=0, int high=1, bool add=false);
template<class T> void rndUniform(MT::Array<T>& a, double low=0., double high=1., bool add=false);
template<class T> void rndNegLogUniform(MT::Array<T>& a, double low=0., double high=1., bool add=false);
template<class T> void rndGauss(MT::Array<T>& a, double stdDev=1., bool add=false);
//template<class T> void rndGauss(MT::Array<T>& a, bool add=false);
//template<class T> MT::Array<T>& rndGauss(double stdDev, uint dim);
template<class T> uint softMax(const MT::Array<T>& a, arr& soft, double beta);
template<class T> MT::Array<T> sqr(const MT::Array<T>& y) { MT::Array<T> x; x.resizeAs(y); for(uint i=0; i<x.N; i++) x.elem(i)=y.elem(i)*y.elem(i); return x; }


//===========================================================================
/// @}
/// @name tensor functions
/// @{

template<class T> void tensorCondNormalize(MT::Array<T> &X, int left);
template<class T> void tensorCondMax(MT::Array<T> &X, uint left);
template<class T> void tensorCondSoftMax(MT::Array<T> &X, uint left, double beta);
template<class T> void tensorCond11Rule(MT::Array<T>& X, uint left, double rate);
template<class T> void tensorCheckCondNormalization(const MT::Array<T> &X, uint left, double tol=1e-10);
template<class T> void tensorCheckCondNormalization_with_logP(const MT::Array<T> &X, uint left, double logP, double tol=1e-10);

template<class T> void tensorEquation(MT::Array<T> &X, const MT::Array<T> &A, const uintA &pickA, const MT::Array<T> &B, const uintA &pickB, uint sum=0);
template<class T> void tensorPermutation(MT::Array<T> &Y, const MT::Array<T> &X, const uintA &Yid);
template<class T> void tensorMarginal(MT::Array<T> &Y, const MT::Array<T> &X, const uintA &Yid);
template<class T> void tensorMaxMarginal(MT::Array<T> &Y, const MT::Array<T> &X, const uintA &Yid);
template<class T> void tensorMarginal_old(MT::Array<T> &y, const MT::Array<T> &x, const uintA &xd, const uintA &ids);
template<class T> void tensorMultiply(MT::Array<T> &X, const MT::Array<T> &Y, const uintA &Yid);
template<class T> void tensorAdd(MT::Array<T> &X, const MT::Array<T> &Y, const uintA &Yid);
template<class T> void tensorMultiply_old(MT::Array<T> &x, const MT::Array<T> &y, const uintA &d, const uintA &ids);
template<class T> void tensorDivide(MT::Array<T> &X, const MT::Array<T> &Y, const uintA &Yid);
template<class T> void tensorAdd(MT::Array<T> &X, const MT::Array<T> &Y, const uintA &Yid);


//===========================================================================
/// @}
/// @name basic Array functions
/// @{

#ifndef SWIG
#define UnaryFunction( func )           \
  template<class T> MT::Array<T> func (const MT::Array<T>& y)
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
#undef UnaryFunction

#define BinaryFunction( func )            \
  template<class T> MT::Array<T> func(const MT::Array<T>& y, const MT::Array<T>& z); \
  template<class T> MT::Array<T> func(const MT::Array<T>& y, T z); \
  template<class T> MT::Array<T> func(T y, const MT::Array<T>& z)
BinaryFunction(atan2);
BinaryFunction(pow);
BinaryFunction(fmod);
#undef BinaryFunction

#endif //SWIG


//===========================================================================
/// @}
/// @name double template functions
/// @{

#ifndef SWIG
template<class T, class S> void resizeAs(MT::Array<T>& x, const MT::Array<S>& a) {
  x.nd=a.nd; x.d0=a.d0; x.d1=a.d1; x.d2=a.d2;
  x.resetD();
  if(x.nd>3) { x.d=new uint[x.nd];  memmove(x.d, a.d, x.nd*sizeof(uint)); }
  x.resizeMEM(a.N, false);
}
template<class T, class S> void resizeCopyAs(MT::Array<T>& x, const MT::Array<S>& a);
template<class T, class S> void reshapeAs(MT::Array<T>& x, const MT::Array<S>& a);
template<class T, class S> void copy(MT::Array<T>& x, const MT::Array<S>& a) {
  resizeAs(x, a);
  for(uint i=0; i<x.N; i++) x.elem(i)=(T)a.elem(i);
}
/// check whether this and \c a have same dimensions
template<class T, class S>
bool samedim(const MT::Array<T>& a, const MT::Array<S>& b) {
  return (b.nd==a.nd && b.d0==a.d0 && b.d1==a.d1 && b.d2==a.d2);
}
#endif //SWIG


//===========================================================================
/// @}
/// @name lapack interfaces
/// @{

void blas_Mv(arr& y, const arr& A, const arr& x);
void blas_MM(arr& X, const arr& A, const arr& B);
void blas_MsymMsym(arr& X, const arr& A, const arr& B);
void blas_A_At(arr& X, const arr& A);
void blas_At_A(arr& X, const arr& A);
void lapack_cholesky(arr& C, const arr& A);
uint lapack_SVD(arr& U, arr& d, arr& Vt, const arr& A);
void lapack_mldivide(arr& X, const arr& A, const arr& b);
void lapack_LU(arr& LU, const arr& A);
void lapack_RQ(arr& R, arr& Q, const arr& A);
void lapack_EigenDecomp(const arr& symmA, arr& Evals, arr& Evecs);
bool lapack_isPositiveSemiDefinite(const arr& symmA);
void lapack_inverseSymPosDef(arr& Ainv, const arr& A);
double lapack_determinantSymPosDef(const arr& A);
inline arr lapack_inverseSymPosDef(const arr& A){ arr Ainv; lapack_inverseSymPosDef(Ainv, A); return Ainv; }
arr lapack_Ainv_b_sym(const arr& A, const arr& b);
void lapack_min_Ax_b(arr& x,const arr& A, const arr& b);


//===========================================================================
/// @}
/// @name special agumentations
/// @{

struct RowShiftedPackedMatrix {
  arr& Z;           ///< references the array itself
  uint real_d1;     ///< the real width (the packed width is Z.d1; the height is Z.d0)
  uintA rowShift;   ///< amount of shift of each row (rowShift.N==Z.d0)
  uintA colPatches; ///< column-patch: (nd=2,d0=real_d1,d1=2) range of non-zeros in a COLUMN; starts with 'a', ends with 'b'-1
  bool symmetric;   ///< flag: if true, this stores a symmetric (banded) matrix: only the upper triangle
  arr *nextInSum;
  
  RowShiftedPackedMatrix(arr& X);
  RowShiftedPackedMatrix(arr& X, RowShiftedPackedMatrix &aux);
  ~RowShiftedPackedMatrix();
  double acc(uint i, uint j);
  void computeColPatches(bool assumeMonotonic);
  arr At_A();
  arr A_At();
  arr At_x(const arr& x);
  arr A_x(const arr& x);
};

inline RowShiftedPackedMatrix& castRowShiftedPackedMatrix(arr& X) {
  ///CHECK(X.special==X.RowShiftedPackedMatrixST,"can't cast like this!");
  if(X.special!=X.RowShiftedPackedMatrixST) throw("can't cast like this!");
  return *((RowShiftedPackedMatrix*)X.aux);
}

arr unpack(const arr& Z); //returns an unpacked matrix in case this is packed
arr packRowShifted(const arr& X);
RowShiftedPackedMatrix *auxRowShifted(arr& Z, uint d0, uint pack_d1, uint real_d1);
arr comp_At_A(arr& A);
arr comp_A_At(arr& A);
arr comp_At_x(arr& A, const arr& x);
arr comp_A_x(arr& A, const arr& x);


//===========================================================================
/// @}
/// @name lists
/// @{

/*  TODO: realize list simpler: let the Array class have a 'listMode' flag. When this flag is true, the read, write, resize, find etc routines
will simply be behave differently */

template<class T> void listWrite(const MT::Array<T*>& L, std::ostream& os, const char *ELEMSEP=" ", const char *delim=NULL);
template<class T> void listWriteNames(const MT::Array<T*>& L, std::ostream& os);
template<class T> void listRead(MT::Array<T*>& L, std::istream& is, const char *delim=NULL);
template<class T> void listCopy(MT::Array<T*>& L, const MT::Array<T*>& M);  //copy a list by calling the copy constructor for each element
template<class T> void listClone(MT::Array<T*>& L, const MT::Array<T*>& M); //copy a list by calling the 'newClone' method of each element (works for virtual types)
template<class T> void listDelete(MT::Array<T*>& L);
template<class T> void listReindex(MT::Array<T*>& L);
template<class T> T* listFindValue(const MT::Array<T*>& L, const T& x);
template<class T> T* listFindByName(const MT::Array<T*>& L, const char* name); //each element needs a 'name' (usually MT::String)
template<class T> T* listFindByType(const MT::Array<T*>& L, const char* type); //each element needs a 'type' (usually MT::String)
template<class T, class LowerOperator> void listSort(MT::Array<T*>& L, LowerOperator lowerop);

//TODO obsolete?
template<class T> MT::Array<T*> getList(const MT::Array<T>& A) {
  MT::Array<T*> L;
  resizeAs(L, A);
  for(uint i=0; i<A.N; i++) L.elem(i) = &A.elem(i);
  return L;
}
template<class T> T* new_elem(MT::Array<T*>& L) { T *e=new T; e->index=L.N; L.append(e); return e; }


//===========================================================================
/// @}
/// @name graphs
/// @{

void graphRandomUndirected(uintA& E, uint n, double connectivity);
void graphRandomFixedDegree(uintA& E, uint N, uint degree);
void graphRandomTree(uintA& E, uint N, uint roots=1);

template<class vert, class edge> edge* graphGetEdge(vert *from, vert *to);
template<class vert, class edge> void graphMakeLists(MT::Array<vert*>& V, MT::Array<edge*>& E);
template<class vert, class edge> void graphRandomUndirected(MT::Array<vert*>& V, MT::Array<edge*>& E, uint N, double connectivity);
template<class vert, class edge> void graphRandomFixedDegree(MT::Array<vert*>& V, MT::Array<edge*>& E, uint N, uint degree);
template<class vert, class edge> void graphRandomLinear(MT::Array<vert*>& V, MT::Array<edge*>& E, uint N);
template<class vert, class edge> void graphConnectUndirected(MT::Array<vert*>& V, MT::Array<edge*>& E);
template<class vert, class edge> void graphLayered(MT::Array<vert*>& V, MT::Array<edge*>& E, const uintA& layers, bool interConnections);
template<class vert, class edge> edge *newEdge(vert *a, vert *b, MT::Array<edge*>& E);
template<class edge> edge *newEdge(uint a , uint b, MT::Array<edge*>& E);
template<class vert, class edge> edge *del_edge(edge *e, MT::Array<vert*>& V, MT::Array<edge*>& E, bool remakeLists);
template<class vert, class edge> void graphWriteDirected(std::ostream& os, const MT::Array<vert*>& V, const MT::Array<edge*>& E);
template<class vert, class edge> void graphWriteUndirected(std::ostream& os, const MT::Array<vert*>& V, const MT::Array<edge*>& E);
template<class vert, class edge> bool graphTopsort(MT::Array<vert*>& V, MT::Array<edge*>& E);
template<class vert, class edge> void graphDelete(MT::Array<vert*>& V, MT::Array<edge*>& E);

/// @}


//===========================================================================
// implementations
//

void linkArray();

//#if defined MT_IMPLEMENT_TEMPLATES | defined MT_IMPLEMENTATION
#  include "array_t.h"
//#endif

#endif
#endif

// (note: http://www.informit.com/articles/article.aspx?p=31783&seqNum=2)

/// @} //end group
