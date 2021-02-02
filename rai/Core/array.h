/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <iostream>
#include <stdint.h>
#include <string.h>
#include <functional>
#include <memory>
#include <vector>

//-- don't require previously defined iterators
#define for_list(Type, it, X)     Type *it=nullptr; for(uint it##_COUNT=0;   it##_COUNT<X.N && ((it=X(it##_COUNT)) || true); it##_COUNT++)
#define for_list_rev(Type, it, X) Type *it=nullptr; for(uint it##_COUNT=X.N; it##_COUNT--   && ((it=X(it##_COUNT)) || true); )

#define ARR ARRAY<double> ///< write ARR(1., 4., 5., 7.) to generate a double-Array
#define TUP ARRAY<uint> ///< write TUP(1, 2, 3) to generate a uint-Array

typedef unsigned char byte;
typedef unsigned int uint;

//fwd declarations
struct Serializable {
  virtual uint serial_size() = 0;
  virtual uint serial_encode(char* data, uint data_size) = 0;
  virtual uint serial_decode(char* data, uint data_size) = 0;
};

struct SpecialArray;

namespace rai {

struct FileToken;
struct SparseVector;
struct SparseMatrix;
struct RowShifted;

// OLD, TODO: hide -> array.cpp
extern bool useLapack;
extern const bool lapackSupported;
extern uint64_t globalMemoryTotal, globalMemoryBound;
extern bool globalMemoryStrict;

// default write formatting
extern const char* arrayElemsep;
extern const char* arrayLinesep;
extern const char* arrayBrackets;

// default sorting methods
template<class T> bool lower(const T& a, const T& b) { return a<b; }
template<class T> bool lowerEqual(const T& a, const T& b) { return a<=b; }
template<class T> bool greater(const T& a, const T& b) { return a>b; }
template<class T> bool greaterEqual(const T& a, const T& b) { return a>=b; }

} //namespace

//===========================================================================
//
// Array class
//

namespace rai {

template<class T> struct ArrayIterationEnumerated;

/** Simple array container to store arbitrary-dimensional arrays (tensors).
  Can buffer more memory than necessary for faster
  resize; enables non-const reference of subarrays; enables fast
  memove for elementary types; implements many standard
  array/matrix/tensor operations. Interfacing with ordinary C-buffers is simple.
  Please see also the reference for the \ref array.h
  header, which contains lots of functions that can be applied on
  Arrays. */
template<class T> struct Array : /*std::vector<T>,*/ Serializable {
  T* p;     ///< the pointer on the linear memory allocated
  uint N;   ///< number of elements
  uint nd;  ///< number of dimensions
  uint d0, d1, d2; ///< 0th, 1st, 2nd dim
  uint* d;  ///< pointer to dimensions (for nd<=3 points to d0)
  bool isReference; ///< true if this refers to some external memory
  uint M;   ///< memory allocated (>=N)

  static int  sizeT;   ///< constant for each type T: stores the sizeof(T)
  static char memMove; ///< constant for each type T: decides whether memmove can be used instead of individual copies

  //-- special: arrays can be sparse/packed/etc and augmented with aux data to support this
  SpecialArray* special; ///< auxiliary data, e.g. if this is a sparse matrics, depends on special type

  typedef std::vector<T> vec_type;
  typedef std::function<bool(const T& a, const T& b)> ElemCompare;

  /// @name constructors
  Array();
  Array(const Array<T>& a);                 //copy constructor
  Array(Array<T>&& a);                      //move constructor
  explicit Array(uint D0);
  explicit Array(uint D0, uint D1);
  explicit Array(uint D0, uint D1, uint D2);
  explicit Array(const T* p, uint size, bool byReference);      //reference!
  explicit Array(const std::vector<T>& a, bool byReference);   //reference?
  Array(std::initializer_list<T> values);
  Array(std::initializer_list<uint> dim, std::initializer_list<T> values);
  explicit Array(SpecialArray* _special); //only used to define NoArrays
  virtual ~Array();
  bool operator!() const; ///< check if NoArr

  Array<T>& operator=(std::initializer_list<T> values);
  Array<T>& operator=(const T& v);
  Array<T>& operator=(const Array<T>& a);
  Array<T>& operator=(const std::vector<T>& values);

  /// @name iterators
  typename vec_type::iterator begin() { return typename vec_type::iterator(p); }
  typename vec_type::const_iterator begin() const { return typename vec_type::const_iterator(p); }
  typename vec_type::iterator end() { return typename vec_type::iterator(p+N); }
  typename vec_type::const_iterator end() const { return typename vec_type::const_iterator(p+N); }
  ArrayIterationEnumerated<T> enumerated() { return ArrayIterationEnumerated<T>(*this); }
  //TODO: more: rows iterator, reverse iterator

  /// @name resizing
  Array<T>& resize(uint D0);
  Array<T>& resize(uint D0, uint D1);
  Array<T>& resize(uint D0, uint D1, uint D2);
  Array<T>& resize(uint ND, uint* dim);
  Array<T>& resize(const Array<uint>& dim);
  Array<T>& reshape(int D0);
  Array<T>& reshape(int D0, int D1);
  Array<T>& reshape(int D0, int D1, int D2);
  Array<T>& reshape(uint ND, uint* dim);
  Array<T>& reshape(const Array<uint>& dim);
  Array<T>& reshape(std::initializer_list<uint> dim);
  Array<T>& resizeCopy(uint D0);
  Array<T>& resizeCopy(uint D0, uint D1);
  Array<T>& resizeCopy(uint D0, uint D1, uint D2);
  Array<T>& resizeCopy(uint ND, uint* dim);
  Array<T>& resizeCopy(const Array<uint>& dim);
  Array<T>& resizeAs(const Array<T>& a);
  Array<T>& reshapeAs(const Array<T>& a);
  Array<T>& resizeCopyAs(const Array<T>& a);
  Array<T>& reshapeFlat();
  Array<T>& dereference();

  /// @name dimensionality access
  uint dim(uint k) const;
  Array<uint> dim() const;

  /// @name initializing/assigning entries
  rai::Array<T>& clear();
  void setZero(byte zero=0);
  void setUni(const T& scalar, int d=-1);
  void setId(int d=-1);
  void setDiag(const T& scalar, int d=-1);
  void setDiag(const Array<T>& vector);
  void setVectorBlock(const Array<T>& B, uint lo);
  void setMatrixBlock(const Array<T>& B, uint lo0, uint lo1);
  //TODO setTensorBlock(const Array<T>& B, const Array<uint>& lo);
  void setBlockMatrix(const Array<T>& A, const Array<T>& B, const Array<T>& C, const Array<T>& D);
  void setBlockMatrix(const Array<T>& A, const Array<T>& B);
  void setBlockVector(const Array<T>& a, const Array<T>& b);
  void setStraightPerm(int n=-1);
  void setReversePerm(int n=-1);
  void setRandomPerm(int n=-1);
  void setCarray(const T* buffer, uint D0);
  void setCarray(const T** buffer, uint D0, uint D1);
  void referTo(const T* buffer, uint n);
  void referTo(const Array<T>& a);
  void referToRange(const Array<T>& a, int i_lo, int i_up);
  void referToRange(const Array<T>& a, int i, int j_lo, int j_up);
  void referToRange(const Array<T>& a, int i, int j, int k_lo, int k_up);
  void referToDim(const Array<T>& a, int i);
  void referToDim(const Array<T>& a, uint i, uint j);
  void referToDim(const Array<T>& a, uint i, uint j, uint k);
  void takeOver(Array<T>& a);  //a becomes a reference to its previously owned memory!
  void swap(Array<T>& a);      //the two arrays swap their contents!
  void setGrid(uint dim, T lo, T hi, uint steps);

  /// @name access by reference (direct memory access)
  Array<T> ref() const; //a reference on this
  T& elem(int i) const;
  T& elem(int i, int j); //access that also handles sparse matrices
//  T& elem(const Array<int> &I) const;
  T& elem(const Array<uint>& I) const;
  T& scalar() const;
  T& first() const;
  T& last(int i=-1) const;
  T& rndElem() const;
  //reference to single elements
  T& operator()(int i) const;
  T& operator()(int i, int j) const;
  T& operator()(int i, int j, int k) const;
  //reference to sub ranges
  Array<T> operator()(std::pair<int, int> I) const;
  Array<T> operator()(int i, std::pair<int, int> J) const;
  Array<T> operator()(int i, int j, std::initializer_list<int> K) const;
  Array<T> operator[](int i) const;     // calls referToDim(*this, i)

  Array<T>& operator()() { return *this; } //TODO: make this the scalar reference!
  T** getCarray(Array<T*>& Cpointers) const;
  Array<T*> getCarray() const;

  /// @name access by copy
  Array<T> copy() const;
  Array<T> sub(int i, int I) const;
  Array<T> sub(int i, int I, int j, int J) const;
  Array<T> sub(int i, int I, int j, int J, int k, int K) const;
  Array<T> sub(int i, int I, Array<uint> cols) const;
  Array<T> sub(Array<uint> elems) const;
  Array<T> row(uint row_index) const;
  Array<T> rows(uint start_row, uint end_row) const;
  Array<T> col(uint col_index) const;
  Array<T> cols(uint start_col, uint end_col) const;

  void getMatrixBlock(Array<T>& B, uint lo0, uint lo1) const; // -> return array
  void getVectorBlock(Array<T>& B, uint lo) const;
  void copyInto(T* buffer) const;
  void copyInto2D(T** buffer) const;
  T& min() const;
  T& max() const;
  void minmax(T& minVal, T& maxVal) const;
  uint argmin() const;
  uint argmax() const;
  void maxIndeces(uint& m1, uint& m2) const; //best and 2nd best -> remove
  void argmax(uint& i, uint& j) const; //-> remove, or return uintA
  void argmax(uint& i, uint& j, uint& k) const; //-> remove
  int findValue(const T& x) const;
  void findValues(rai::Array<uint>& indices, const T& x) const;
  bool contains(const T& x) const { return findValue(x)!=-1; }
  bool contains(const Array<T>& X) const { for(const T& x:X) if(findValue(x)==-1) return false; return true; }
  bool containsDoubles() const;
  uint getMemsize() const; // -> remove
  void getIndexTuple(Array<uint>& I, uint i) const; // -> remove?
  std::vector<T> vec() const{ return std::vector<T>(begin(), end()); }

  /// @name appending etc
  T& append();
  T& append(const T& x);
  void append(const T& x, uint multiple);
  void append(const Array<T>& x);
  void append(const T* p, uint n);
  Array<T>& prepend(const T& x) { insert(0, x); return *this; }
  void prepend(const Array<T>& x) { insert(0, x); }
  void replicate(uint copies);
  void insert(uint i, const T& x);
  void insert(uint i, const Array<T>& x);
  void replace(uint i, uint n, const Array<T>& x);
  void remove(int i, uint n=1);
  void removePerm(uint i);          //more efficient for sets, works also for non-memMove arrays
  bool removeValue(const T& x, bool errorIfMissing=true);
  void removeAllValues(const T& x);
  void delRows(int i, uint k=1);
  void delColumns(int i, uint k=1);
  void insRows(int i, uint k=1);
  void insColumns(int i, uint k=1);
  void resizeDim(uint k, uint dk);
  void setAppend(const T& x); //? same as if(findValue(x)==-1) append(x)
  void setAppend(const Array<T>& x);
  T popFirst();
  T popLast();
  void removeLast();

  /// @name sorting and permuting this array
  T median_nonConst(); //this modifies the array!
  T nthElement_nonConst(uint n); //this modifies the array!
  Array<T>& sort(ElemCompare comp=lowerEqual<T>);
  bool isSorted(ElemCompare comp=lowerEqual<T>) const;
  uint rankInSorted(const T& x, ElemCompare comp=lowerEqual<T>, bool rankAfterIfEqual=false) const;
  int findValueInSorted(const T& x, ElemCompare comp=lowerEqual<T>) const;
  bool containsInSorted(const T& x, ElemCompare comp=lowerEqual<T>) const { return findValueInSorted(x)!=-1; }
  bool containsInSorted(const Array<T>& X, ElemCompare comp=lowerEqual<T>) const { for(const T& x:X) if(findValue(x)==-1) return false; return true; }
  uint insertInSorted(const T& x, ElemCompare comp=lowerEqual<T>, bool insertAfterIfEqual=false);
  uint setAppendInSorted(const T& x, ElemCompare comp=lowerEqual<T>);
  void removeValueInSorted(const T& x, ElemCompare comp=lowerEqual<T>);
  Array<T>& removeDoublesInSorted();
  void reverse();
  void reverseRows();
  void permute(uint i, uint j);
  void permute(const Array<uint>& permutation);
  void permuteInv(const Array<uint>& permutation);
  void permuteRows(const Array<uint>& permutation);
  void permuteRowsInv(const Array<uint>& permutation);
  void permuteRandomly();
  void shift(int offset, bool wrapAround=true);

  /// @name special matrices
  double sparsity();
  SparseMatrix& sparse();
  const SparseMatrix& sparse() const;
  SparseVector& sparseVec();
  const SparseVector& sparseVec() const;
  RowShifted& rowShifted();
  const RowShifted& rowShifted() const;
  bool isSparse() const;
  void setNoArr();

  /// @name I/O
  void write(std::ostream& os=std::cout, const char* ELEMSEP=nullptr, const char* LINESEP=nullptr, const char* BRACKETS=nullptr, bool dimTag=false, bool binary=false) const;
  void read(std::istream& is);
  void writeTagged(std::ostream& os, const char* tag, bool binary=false) const;
  bool readTagged(std::istream& is, const char* tag);
  void writeTagged(const char* filename, const char* tag, bool binary=false) const;
  bool readTagged(const char* filename, const char* tag);
  void writeDim(std::ostream& os=std::cout) const;
  void readDim(std::istream& is);
  void writeRaw(std::ostream& os) const;
  void readRaw(std::istream& is);
  void writeWithIndex(std::ostream& os=std::cout) const;
  const Array<T>& ioraw() const;
  const char* prt(); //gdb pretty print

  /// @name kind of private
  void resizeMEM(uint n, bool copy, int Mforce=-1);
  void reserveMEM(uint Mforce) { resizeMEM(N, true, Mforce); if(!nd) nd=1; }
  void freeMEM();
  void resetD();

  /// @name serialization
  uint serial_size();
  uint serial_encode(char* data, uint data_size);
  uint serial_decode(char* data, uint data_size);
};

//===========================================================================
///
/// @name alternative iterators
/// @{

template<class T> struct ArrayItEnumerated {
  T* p;
  uint i;
  T& operator()() { return *p; } //access to value by user
  void operator++() { p++; i++; }
  ArrayItEnumerated<T>& operator*() { return *this; } //in for(auto& it:array.enumerated())  it is assigned to *iterator
};

template<class T> struct ArrayIterationEnumerated {
  Array<T>& x;
  ArrayIterationEnumerated(Array<T>& x):x(x) {}
  ArrayItEnumerated<T> begin() { return {x.p, 0}; }
  ArrayItEnumerated<T> end() { return {x.p+x.N, x.N}; }
  //  const_iterator begin() const { return p; }
  //  const_iterator end() const { return p+N; }
};

template<class T> bool operator!=(const ArrayItEnumerated<T>& i, const ArrayItEnumerated<T>& j) { return i.p!=j.p; }

//===========================================================================
/// @}
/// @name basic Array operators
/// @{

template<class T> Array<T> operator~(const Array<T>& y); //transpose
template<class T> Array<T> operator-(const Array<T>& y); //negative
template<class T> Array<T> operator^(const Array<T>& y, const Array<T>& z); //outer product
template<class T> Array<T> operator%(const Array<T>& y, const Array<T>& z); //index/element-wise product
template<class T> Array<T> operator*(const Array<T>& y, const Array<T>& z); //inner product
template<class T> Array<T> operator*(const Array<T>& y, T z);
template<class T> Array<T> operator*(T y, const Array<T>& z);
template<class T> Array<T> operator/(int mustBeOne, const Array<T>& z_tobeinverted);
template<class T> Array<T> operator/(const Array<T>& y, T z);
template<class T> Array<T> operator/(const Array<T>& y, const Array<T>& z); //element-wise devision
template<class T> Array<T> operator|(const Array<T>& A, const Array<T>& B); //A^-1 B
template<class T> Array<T> operator, (const Array<T>& y, const Array<T>& z); //concat

template<class T> Array<T>& operator<<(Array<T>& x, const T& y); //append
template<class T> Array<T>& operator<<(Array<T>& x, const Array<T>& y); //append

template<class T> bool operator==(const Array<T>& v, const Array<T>& w); //equal in size and all elements
template<class T> Array<byte> operator==(const Array<T>& v, const T& w); //element-wise equal
template<class T> bool operator!=(const Array<T>& v, const Array<T>& w);
template<class T> bool operator<(const Array<T>& v, const Array<T>& w);
template<class T> std::istream& operator>>(std::istream& is, Array<T>& x);
//template<class T> std::ostream& operator<<(std::ostream& os, const Array<T>& x);

//element-wise update operators
#ifndef SWIG
#define UpdateOperator( op )        \
  template<class T> Array<T>& operator op (Array<T>& x, const Array<T>& y); \
  template<class T> Array<T>& operator op (Array<T>& x, T y ); \
  template<class T> void operator op (Array<T>&& x, const Array<T>& y); \
  template<class T> void operator op (Array<T>&& x, T y );
UpdateOperator(|=)
UpdateOperator(^=)
UpdateOperator(&=)
UpdateOperator(+=)
UpdateOperator(-=)
UpdateOperator(*=)
UpdateOperator(/=)
UpdateOperator(%=)
#undef UpdateOperator
#endif

//element-wise operators
#define BinaryOperator( op, updateOp)         \
  template<class T> Array<T> operator op(const Array<T>& y, const Array<T>& z); \
  template<class T> Array<T> operator op(T y, const Array<T>& z);  \
  template<class T> Array<T> operator op(const Array<T>& y, T z)
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

#ifndef SWIG
#define UnaryFunction( func )           \
  template<class T> rai::Array<T> func (const rai::Array<T>& y)
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
  template<class T> rai::Array<T> func(const rai::Array<T>& y, const rai::Array<T>& z); \
  template<class T> rai::Array<T> func(const rai::Array<T>& y, T z); \
  template<class T> rai::Array<T> func(T y, const rai::Array<T>& z)
BinaryFunction(atan2);
BinaryFunction(pow);
BinaryFunction(fmod);
#undef BinaryFunction

#endif //SWIG

//===========================================================================
/// @}
/// @name standard types
/// @{

typedef rai::Array<double> arr;
typedef rai::Array<double> doubleA;
typedef rai::Array<float>  floatA;
typedef rai::Array<uint>   uintA;
typedef rai::Array<int>    intA;
typedef rai::Array<char>   charA;
typedef rai::Array<byte>   byteA;
typedef rai::Array<byte>   boolA;
typedef rai::Array<uint16_t>   uint16A;
typedef rai::Array<uint32_t>   uint32A;
typedef rai::Array<const char*>  CstrList;
typedef rai::Array<arr*>   arrL;
typedef rai::Array<arr>    arrA;
typedef rai::Array<intA>   intAA;
typedef rai::Array<uintA>  uintAA;

namespace rai { struct String; }
typedef rai::Array<rai::String> StringA;
typedef rai::Array<StringA> StringAA;
typedef rai::Array<rai::String*> StringL;

//===========================================================================
/// @}
/// @name constant non-arrays
/// @{

extern arr& NoArr; //this is a pointer to nullptr!!!! I use it for optional arguments
extern arrA& NoArrA; //this is a pointer to nullptr!!!! I use it for optional arguments
extern uintA& NoUintA; //this is a pointer to nullptr!!!! I use it for optional arguments
extern byteA& NoByteA; //this is a pointer to nullptr!!!! I use it for optional arguments
extern intAA& NoIntAA; //this is a pointer to nullptr!!!! I use it for optional arguments
extern uintAA& NoUintAA; //this is a pointer to nullptr!!!! I use it for optional arguments
extern uint16A& NoUint16A; //this is a pointer to nullptr!!!! I use it for optional arguments
extern StringA& NoStringA; //this is a pointer to nullptr!!!! I use it for optional arguments

//===========================================================================
/// @}
/// @name basic function types
/// @{

/// a scalar function \f$f:~x\mapsto y\in\mathbb{R}\f$ with optional gradient and hessian
//struct ScalarFunction {
//  virtual double fs(arr& g, arr& H, const arr& x) = 0;
//  virtual ~ScalarFunction(){}
//};

typedef std::function<double(arr& g, arr& H, const arr& x)> ScalarFunction;

/// a vector function \f$f:~x\mapsto y\in\mathbb{R}^d\f$ with optional Jacobian
//struct VectorFunction {
//  virtual void fv(arr& y, arr& J, const arr& x) = 0; ///< returning a vector y and (optionally, if NoArr) Jacobian J for x
//  virtual ~VectorFunction(){}
//};
typedef std::function<void(arr& y, arr& J, const arr& x)> VectorFunction;

/// a kernel function
struct KernelFunction {
  virtual double k(const arr& x1, const arr& x2, arr& g1=NoArr, arr& Hx1=NoArr) = 0;
  virtual ~KernelFunction() {}
};

//===========================================================================

template<class T> rai::Array<T> ARRAY() {                                    rai::Array<T> z(0); return z; }
template<class T> rai::Array<T> ARRAY(const T& i) {                                    rai::Array<T> z(1); z(0)=i; return z; }
template<class T> rai::Array<T> ARRAY(const T& i, const T& j) {                               rai::Array<T> z(2); z(0)=i; z(1)=j; return z; }
template<class T> rai::Array<T> ARRAY(const T& i, const T& j, const T& k) {                          rai::Array<T> z(3); z(0)=i; z(1)=j; z(2)=k; return z; }
template<class T> rai::Array<T> ARRAY(const T& i, const T& j, const T& k, const T& l) {                     rai::Array<T> z(4); z(0)=i; z(1)=j; z(2)=k; z(3)=l; return z; }
template<class T> rai::Array<T> ARRAY(const T& i, const T& j, const T& k, const T& l, const T& m) {                rai::Array<T> z(5); z(0)=i; z(1)=j; z(2)=k; z(3)=l; z(4)=m; return z; }
template<class T> rai::Array<T> ARRAY(const T& i, const T& j, const T& k, const T& l, const T& m, const T& n) {           rai::Array<T> z(6); z(0)=i; z(1)=j; z(2)=k; z(3)=l; z(4)=m; z(5)=n; return z; }
template<class T> rai::Array<T> ARRAY(const T& i, const T& j, const T& k, const T& l, const T& m, const T& n, const T& o) {      rai::Array<T> z(7); z(0)=i; z(1)=j; z(2)=k; z(3)=l; z(4)=m; z(5)=n; z(6)=o; return z; }
template<class T> rai::Array<T> ARRAY(const T& i, const T& j, const T& k, const T& l, const T& m, const T& n, const T& o, const T& p) { rai::Array<T> z(8); z(0)=i; z(1)=j; z(2)=k; z(3)=l; z(4)=m; z(5)=n; z(6)=o; z(7)=p; return z; }
template<class T> rai::Array<T> ARRAY(const T& i, const T& j, const T& k, const T& l, const T& m, const T& n, const T& o, const T& p, const T& q) { rai::Array<T> z(9); z(0)=i; z(1)=j; z(2)=k; z(3)=l; z(4)=m; z(5)=n; z(6)=o; z(7)=p; z(8)=q; return z; }

template<class T> rai::Array<T*> LIST() {                                    rai::Array<T*> z(0); return z; }
template<class T> rai::Array<T*> LIST(const T& i) {                                    rai::Array<T*> z(1); z(0)=(T*)&i; return z; }
template<class T> rai::Array<T*> LIST(const T& i, const T& j) {                               rai::Array<T*> z(2); z(0)=(T*)&i; z(1)=(T*)&j; return z; }
template<class T> rai::Array<T*> LIST(const T& i, const T& j, const T& k) {                          rai::Array<T*> z(3); z(0)=(T*)&i; z(1)=(T*)&j; z(2)=(T*)&k; return z; }
template<class T> rai::Array<T*> LIST(const T& i, const T& j, const T& k, const T& l) {                     rai::Array<T*> z(4); z(0)=(T*)&i; z(1)=(T*)&j; z(2)=(T*)&k; z(3)=(T*)&l; return z; }
template<class T> rai::Array<T*> LIST(const T& i, const T& j, const T& k, const T& l, const T& m) {                rai::Array<T*> z(5); z(0)=(T*)&i; z(1)=(T*)&j; z(2)=(T*)&k; z(3)=(T*)&l; z(4)=(T*)&m; return z; }
template<class T> rai::Array<T*> LIST(const T& i, const T& j, const T& k, const T& l, const T& m, const T& n) {           rai::Array<T*> z(6); z(0)=(T*)&i; z(1)=(T*)&j; z(2)=(T*)&k; z(3)=(T*)&l; z(4)=(T*)&m; z(5)=(T*)&n; return z; }
template<class T> rai::Array<T*> LIST(const T& i, const T& j, const T& k, const T& l, const T& m, const T& n, const T& o) {      rai::Array<T*> z(7); z(0)=(T*)&i; z(1)=(T*)&j; z(2)=(T*)&k; z(3)=(T*)&l; z(4)=(T*)&m; z(5)=(T*)&n; z(6)=(T*)&o; return z; }
template<class T> rai::Array<T*> LIST(const T& i, const T& j, const T& k, const T& l, const T& m, const T& n, const T& o, const T& p) { rai::Array<T*> z(8); z(0)=(T*)&i; z(1)=(T*)&j; z(2)=(T*)&k; z(3)=(T*)&l; z(4)=(T*)&m; z(5)=(T*)&n; z(6)=(T*)&o; z(7)=(T*)&p; return z; }

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
inline arr ones(uint n) { return ones(TUP(n)); }
/// return matrix of ones
inline arr ones(uint d0, uint d1) { return ones(TUP(d0, d1)); }

/// return array of zeros
inline arr zeros(const uintA& d) {  arr z;  z.resize(d);  z.setZero();  return z; }
/// return VECTOR of zeros
inline arr zeros(uint n) { return zeros(TUP(n)); }
/// return matrix of zeros
inline arr zeros(uint d0, uint d1) { return zeros(TUP(d0, d1)); }
/// return tensor of zeros
inline arr zeros(uint d0, uint d1, uint d2) { return zeros(TUP(d0, d1, d2)); }

/// return array of c's
template<class T> rai::Array<T> consts(const T& c, const uintA& d)  {  rai::Array<T> z;  z.resize(d);  z.setUni(c);  return z; }
/// return VECTOR of c's
template<class T> rai::Array<T> consts(const T& c, uint n) { return consts(c, TUP(n)); }
/// return matrix of c's
template<class T> rai::Array<T> consts(const T& c, uint d0, uint d1) { return consts(c, TUP(d0, d1)); }
/// return tensor of c's
template<class T> rai::Array<T> consts(const T& c, uint d0, uint d1, uint d2) { return consts(c, TUP(d0, d1, d2)); }

/// return array with uniform random numbers in [0, 1]
arr rand(const uintA& d);
/// return array with uniform random numbers in [0, 1]
inline arr rand(uint n) { return rand(TUP(n)); }
/// return array with uniform random numbers in [0, 1]
inline arr rand(uint d0, uint d1) { return rand(TUP(d0, d1)); }

/// return array with normal (Gaussian) random numbers
arr randn(const uintA& d);
/// return array with normal (Gaussian) random numbers
inline arr randn(uint n) { return randn(TUP(n)); }
/// return array with normal (Gaussian) random numbers
inline arr randn(uint d0, uint d1) { return randn(TUP(d0, d1)); }

/// return a grid with different lo/hi/steps in each dimension
arr grid(const arr& lo, const arr& hi, const uintA& steps);
/// return a grid (1D: range) split in 'steps' steps
inline arr grid(uint dim, double lo, double hi, uint steps) { arr g;  g.setGrid(dim, lo, hi, steps);  return g; }

arr repmat(const arr& A, uint m, uint n);

//inline double max(const arr& x) { return x.max(); }
//inline double min(const arr& x) { return x.min(); }
inline uint argmax(const arr& x) { return x.argmax(); }
inline uint argmin(const arr& x) { return x.argmin(); }

inline uintA randperm(uint n) {  uintA z;  z.setRandomPerm(n);  return z; }
inline arr linspace(double base, double limit, uint n) {  arr z;  z.setGrid(1, base, limit, n);  return z;  }
arr logspace(double base, double limit, uint n);

void normalizeWithJac(arr& y, arr& J);

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
//these are obsolete, use catCol instead
void write(const arrL& X, const char* filename, const char* ELEMSEP=" ", const char* LINESEP="\n ", const char* BRACKETS="  ", bool dimTag=false, bool binary=false);

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

void scanArrFile(const char* name);

arr finiteDifferenceGradient(const ScalarFunction& f, const arr& x, arr& Janalytic=NoArr);
arr finiteDifferenceJacobian(const VectorFunction& f, const arr& _x, arr& Janalytic=NoArr);
bool checkGradient(const ScalarFunction& f, const arr& x, double tolerance, bool verbose=false);
bool checkHessian(const ScalarFunction& f, const arr& x, double tolerance, bool verbose=false);
bool checkJacobian(const VectorFunction& f, const arr& x, double tolerance, bool verbose=false);

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

//NOTES:
// -- past-tense names do not modify the array, but return variants
// -- more methods should return an array instead of have a returned parameter...

template<class T> rai::Array<T> vectorShaped(const rai::Array<T>& x) {  rai::Array<T> y;  y.referTo(x);  y.reshape(y.N);  return y;  }
template<class T> void transpose(rai::Array<T>& x, const rai::Array<T>& y);
template<class T> void negative(rai::Array<T>& x, const rai::Array<T>& y);
template<class T> rai::Array<T> getDiag(const rai::Array<T>& y);
template<class T> rai::Array<T> diag(const rai::Array<T>& x) {  rai::Array<T> y;  y.setDiag(x);  return y;  }
template<class T> rai::Array<T> skew(const rai::Array<T>& x);
template<class T> void inverse2d(rai::Array<T>& Ainv, const rai::Array<T>& A);
template<class T> rai::Array<T> replicate(const rai::Array<T>& A, uint d0);
template<class T> rai::Array<T> integral(const rai::Array<T>& x);

template<class T> uintA size(const rai::Array<T>& x) { return x.dim(); } //TODO: remove
template<class T> void checkNan(const rai::Array<T>& x);
template<class T> void sort(rai::Array<T>& x);

template<class T> T entropy(const rai::Array<T>& v);
template<class T> T normalizeDist(rai::Array<T>& v);
template<class T> void makeConditional(rai::Array<T>& P);
template<class T> void checkNormalization(rai::Array<T>& v, double tol);
template<class T> void checkNormalization(rai::Array<T>& v) { checkNormalization(v, 1e-10); }
template<class T> void eliminate(rai::Array<T>& x, const rai::Array<T>& y, uint d);
template<class T> void eliminate(rai::Array<T>& x, const rai::Array<T>& y, uint d, uint e);
template<class T> void eliminatePartial(rai::Array<T>& x, const rai::Array<T>& y, uint d);

#ifndef SWIG
template<class T> T sqrDistance(const rai::Array<T>& v, const rai::Array<T>& w);
template<class T> T maxDiff(const rai::Array<T>& v, const rai::Array<T>& w, uint* maxi=0);
template<class T> T maxRelDiff(const rai::Array<T>& v, const rai::Array<T>& w, T tol);
//template<class T> T sqrDistance(const rai::Array<T>& v, const rai::Array<T>& w, const rai::Array<bool>& mask);
template<class T> T sqrDistance(const rai::Array<T>& g, const rai::Array<T>& v, const rai::Array<T>& w);
template<class T> T euclideanDistance(const rai::Array<T>& v, const rai::Array<T>& w);
template<class T> T metricDistance(const rai::Array<T>& g, const rai::Array<T>& v, const rai::Array<T>& w);

template<class T> T sum(const rai::Array<T>& v);
template<class T> T scalar(const rai::Array<T>& v);
template<class T> rai::Array<T> sum(const rai::Array<T>& v, uint d);
template<class T> T sumOfAbs(const rai::Array<T>& v);
template<class T> T sumOfSqr(const rai::Array<T>& v);
template<class T> T length(const rai::Array<T>& v);
template<class T> T product(const rai::Array<T>& v);
template<class T> T max(const rai::Array<T>& v);
template<class T> rai::Array<T> max(const rai::Array<T>& v, uint d);

template<class T> T trace(const rai::Array<T>& v);
template<class T> T var(const rai::Array<T>& v);
template<class T> rai::Array<T> mean(const rai::Array<T>& v);
template<class T> rai::Array<T> stdDev(const rai::Array<T>& v);
template<class T> T minDiag(const rai::Array<T>& v);
template<class T> T absMax(const rai::Array<T>& x);
template<class T> T absMin(const rai::Array<T>& x);
template<class T> void clip(const rai::Array<T>& x, T lo, T hi);

template<class T> void innerProduct(rai::Array<T>& x, const rai::Array<T>& y, const rai::Array<T>& z);
template<class T> void outerProduct(rai::Array<T>& x, const rai::Array<T>& y, const rai::Array<T>& z);
template<class T> void indexWiseProduct(rai::Array<T>& x, const rai::Array<T>& y, const rai::Array<T>& z);
template<class T> rai::Array<T> crossProduct(const rai::Array<T>& y, const rai::Array<T>& z); //only for 3 x 3 or (3,n) x 3
template<class T> T scalarProduct(const rai::Array<T>& v, const rai::Array<T>& w);
template<class T> T scalarProduct(const rai::Array<T>& g, const rai::Array<T>& v, const rai::Array<T>& w);
template<class T> rai::Array<T> diagProduct(const rai::Array<T>& v, const rai::Array<T>& w);

template<class T> rai::Array<T> elemWiseMin(const rai::Array<T>& v, const rai::Array<T>& w);
template<class T> rai::Array<T> elemWiseMax(const rai::Array<T>& v, const rai::Array<T>& w);
template<class T> rai::Array<T> elemWisemax(const rai::Array<T>& x, const T& y);
template<class T> rai::Array<T> elemWisemax(const T& x, const rai::Array<T>& y);
template<class T> rai::Array<T> elemWiseHinge(const rai::Array<T>& x);

template<class T> void writeConsecutiveConstant(std::ostream& os, const rai::Array<T>& x);

//===========================================================================
/// @}
/// @name concatenating arrays together
/// @{

template<class T> rai::Array<T> cat(const rai::Array<T>& y, const rai::Array<T>& z) { rai::Array<T> x(y); x.append(z); return x; }
template<class T> rai::Array<T> cat(const rai::Array<T>& y, const rai::Array<T>& z, const rai::Array<T>& w) { rai::Array<T> x; x.append(y); x.append(z); x.append(w); return x; }
template<class T> rai::Array<T> cat(const rai::Array<T>& a, const rai::Array<T>& b, const rai::Array<T>& c, const rai::Array<T>& d) { rai::Array<T> x; x.append(a); x.append(b); x.append(c); x.append(d); return x; }
template<class T> rai::Array<T> cat(const rai::Array<T>& a, const rai::Array<T>& b, const rai::Array<T>& c, const rai::Array<T>& d, const rai::Array<T>& e) { rai::Array<T> x; x.append(a); x.append(b); x.append(c); x.append(d); x.append(e); return x; }
template<class T> rai::Array<T> cat(const rai::Array<rai::Array<T>>& X) {  rai::Array<T> x; for(const rai::Array<T>& z: X) x.append(z); return x; }
template<class T> rai::Array<T> catCol(const rai::Array<rai::Array<T>*>& X);
template<class T> rai::Array<T> catCol(const rai::Array<rai::Array<T>>& X);
template<class T> rai::Array<T> catCol(const rai::Array<T>& a, const rai::Array<T>& b) { return catCol(LIST<rai::Array<T>>(a, b)); }
template<class T> rai::Array<T> catCol(const rai::Array<T>& a, const rai::Array<T>& b, const rai::Array<T>& c) { return catCol(LIST<rai::Array<T>>(a, b, c)); }
template<class T> rai::Array<T> catCol(const rai::Array<T>& a, const rai::Array<T>& b, const rai::Array<T>& c, const rai::Array<T>& d) { return catCol(LIST<rai::Array<T>>(a, b, c, d)); }
template<class T> rai::Array<T> catCol(const rai::Array<T>& a, const rai::Array<T>& b, const rai::Array<T>& c, const rai::Array<T>& d, const rai::Array<T>& e) { return catCol(LIST<rai::Array<T>>(a, b, c, d, e)); }

//===========================================================================
/// @}
/// @name arrays interpreted as a set
/// @{

template<class T> void setUnion(rai::Array<T>& x, const rai::Array<T>& y, const rai::Array<T>& z);
template<class T> void setSection(rai::Array<T>& x, const rai::Array<T>& y, const rai::Array<T>& z);
template<class T> rai::Array<T> setUnion(const rai::Array<T>& y, const rai::Array<T>& z) { rai::Array<T> x; setUnion(x, y, z); return x; }
template<class T> rai::Array<T> setSection(const rai::Array<T>& y, const rai::Array<T>& z) { rai::Array<T> x; setSection(x, y, z); return x; }
template<class T> rai::Array<T> setSectionSorted(const rai::Array<T>& x, const rai::Array<T>& y,
    bool (*comp)(const T& a, const T& b));
template<class T> void setMinus(rai::Array<T>& x, const rai::Array<T>& y);
template<class T> void setMinusSorted(rai::Array<T>& x, const rai::Array<T>& y,
                                      bool (*comp)(const T& a, const T& b)=rai::lowerEqual<T>);
template<class T> uint numberSharedElements(const rai::Array<T>& x, const rai::Array<T>& y);
template<class T> void rndInteger(rai::Array<T>& a, int low=0, int high=1, bool add=false);
template<class T> void rndUniform(rai::Array<T>& a, double low=0., double high=1., bool add=false);
template<class T> void rndNegLogUniform(rai::Array<T>& a, double low=0., double high=1., bool add=false);
template<class T> void rndGauss(rai::Array<T>& a, double stdDev=1., bool add=false);
//template<class T> void rndGauss(rai::Array<T>& a, bool add=false);
//template<class T> rai::Array<T>& rndGauss(double stdDev, uint dim);
template<class T> uint softMax(const rai::Array<T>& a, arr& soft, double beta);
template<class T> rai::Array<T> sqr(const rai::Array<T>& y) { rai::Array<T> x; x.resizeAs(y); for(uint i=0; i<x.N; i++) x.elem(i)=y.elem(i)*y.elem(i); return x; }

//===========================================================================
/// @}
/// @name tensor functions
/// @{

template<class T> void tensorCondNormalize(rai::Array<T>& X, int left);
template<class T> void tensorCondMax(rai::Array<T>& X, uint left);
template<class T> void tensorCondSoftMax(rai::Array<T>& X, uint left, double beta);
template<class T> void tensorCond11Rule(rai::Array<T>& X, uint left, double rate);
template<class T> void tensorCheckCondNormalization(const rai::Array<T>& X, uint left, double tol=1e-10);
template<class T> void tensorCheckCondNormalization_with_logP(const rai::Array<T>& X, uint left, double logP, double tol=1e-10);

template<class T> void tensorEquation(rai::Array<T>& X, const rai::Array<T>& A, const uintA& pickA, const rai::Array<T>& B, const uintA& pickB, uint sum=0);
template<class T> void tensorPermutation(rai::Array<T>& Y, const rai::Array<T>& X, const uintA& Yid);
template<class T> void tensorMarginal(rai::Array<T>& Y, const rai::Array<T>& X, const uintA& Yid);
template<class T> void tensorMaxMarginal(rai::Array<T>& Y, const rai::Array<T>& X, const uintA& Yid);
template<class T> void tensorMarginal_old(rai::Array<T>& y, const rai::Array<T>& x, const uintA& xd, const uintA& ids);
template<class T> void tensorMultiply(rai::Array<T>& X, const rai::Array<T>& Y, const uintA& Yid);
template<class T> void tensorAdd(rai::Array<T>& X, const rai::Array<T>& Y, const uintA& Yid);
template<class T> void tensorMultiply_old(rai::Array<T>& x, const rai::Array<T>& y, const uintA& d, const uintA& ids);
template<class T> void tensorDivide(rai::Array<T>& X, const rai::Array<T>& Y, const uintA& Yid);

//===========================================================================
/// @}
/// @name twice template functions
/// @{

#ifndef SWIG
template<class T, class S> void resizeAs(rai::Array<T>& x, const rai::Array<S>& a) {
  x.nd=a.nd; x.d0=a.d0; x.d1=a.d1; x.d2=a.d2;
  x.resetD();
  if(x.nd>3) { x.d=new uint[x.nd];  memmove(x.d, a.d, x.nd*sizeof(uint)); }
  x.resizeMEM(a.N, false);
}
template<class T, class S> void resizeCopyAs(rai::Array<T>& x, const rai::Array<S>& a);
template<class T, class S> void reshapeAs(rai::Array<T>& x, const rai::Array<S>& a);
template<class T, class S> void copy(rai::Array<T>& x, const rai::Array<S>& a) {
  resizeAs(x, a);
  for(uint i=0; i<x.N; i++) x.elem(i)=(T)a.elem(i);
}
template<class T, class S> rai::Array<T> convert(const rai::Array<S>& a) {
  rai::Array<T> x;
  copy<T, S>(x, a);
  return x;
}
/// check whether this and \c a have same dimensions
template<class T, class S>
bool samedim(const rai::Array<T>& a, const rai::Array<S>& b) {
  return (b.nd==a.nd && b.d0==a.d0 && b.d1==a.d1 && b.d2==a.d2);
}
#endif //SWIG

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

struct SpecialArray {
  enum Type { ST_none, ST_NoArr, ST_EmptyShape, hasCarrayST, sparseVectorST, sparseMatrixST, diagST, RowShiftedST, CpointerST };
  Type type;
  SpecialArray(Type _type=ST_none) : type(_type) {}
  virtual ~SpecialArray() {}
};

namespace rai {

template<class T> bool isSpecial(const Array<T>& X)      { return X.special && X.special->type!=SpecialArray::ST_none; }
template<class T> bool isNoArr(const Array<T>& X)        { return X.special && X.special->type==SpecialArray::ST_NoArr; }
template<class T> bool isEmptyShape(const Array<T>& X)   { return X.special && X.special->type==SpecialArray::ST_EmptyShape; }
template<class T> bool isRowShifted(const Array<T>& X)   { return X.special && X.special->type==SpecialArray::RowShiftedST; }
template<class T> bool isSparseMatrix(const Array<T>& X) { return X.special && X.special->type==SpecialArray::sparseMatrixST; }
template<class T> bool isSparseVector(const Array<T>& X) { return X.special && X.special->type==SpecialArray::sparseVectorST; }
template<class T> bool Array<T>::isSparse() const { return special && (special->type==SpecialArray::sparseMatrixST || special->type==SpecialArray::sparseVectorST); }

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
  arr memRef() const{ arr x(Z.p, Z.N, true); x.reshape(Z.d0, rowSize); return x; }
  //manipulations
  void resize(uint d0, uint d1, uint _rowSize);
  void resizeCopy(uint d0, uint d1, uint n);
  void reshape(uint d0, uint d1);
  void reshift(); //shift all cols to start with non-zeros
  void computeColPatches(bool assumeMonotonic);
  void insRow(uint i){
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
inline std::ostream& operator<<(std::ostream& os, const RowShifted& x){ x.write(os); return os; }

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
  arr getSparseRow(uint i);
  arr memRef() const{ return arr(Z.p, Z.N, true); }
  //construction
  void setFromDense(const arr& X);
  void setupRowsCols();
  //manipulations
  void resize(uint d0, uint d1, uint n);
  void resizeCopy(uint d0, uint d1, uint n);
  void reshape(uint d0, uint d1);
  void rowShift(int shift); //shift all rows to the right
  void colShift(int shift); //shift all cols downward
  //computations
  arr At_x(const arr& x);
  arr At_A();
  arr A_B(const arr& B) const;
  arr B_A(const arr& B) const;
  void transpose();
  void rowWiseMult(const arr& a);
  void rowWiseMult(const floatA& a);
  void add(const SparseMatrix& a, uint lo0=0, uint lo1=0, double coeff=1.);
  void add(const arr& B, uint lo0=0, uint lo1=0, double coeff=1.);
  arr unsparse();
};

arr unpack(const arr& X);
arr comp_At_A(const arr& A);
arr comp_A_At(const arr& A);
arr comp_At_x(const arr& A, const arr& x);
arr comp_At(const arr& A);
arr comp_A_x(const arr& A, const arr& x);
arr makeRowSparse(const arr& X);

}//namespace rai

#define UpdateOperator( op ) \
  void operator op (rai::SparseMatrix& x, const rai::SparseMatrix& y); \
  void operator op (rai::SparseMatrix& x, double y ); \
  void operator op (rai::RowShifted& x, const rai::RowShifted& y); \
  void operator op (rai::RowShifted& x, double y );
UpdateOperator(|=)
UpdateOperator(^=)
UpdateOperator(&=)
UpdateOperator(+=)
UpdateOperator(-=)
UpdateOperator(*=)
UpdateOperator(/=)
UpdateOperator(%=)
#undef UpdateOperator

//===========================================================================
/// @}
/// @name lists -- TODO: make lists 'special'
/// @{

/*  TODO: realize list simpler: let the Array class have a 'listMode' flag. When this flag is true, the read, write, resize, find etc routines
will simply be behave differently */

template<class T> char listWrite(const rai::Array<std::shared_ptr<T>>& L, std::ostream& os=std::cout, const char* ELEMSEP=" ", const char* delim=nullptr);
template<class T> char listWrite(const rai::Array<T*>& L, std::ostream& os=std::cout, const char* ELEMSEP=" ", const char* delim=nullptr);
template<class T> void listWriteNames(const rai::Array<T*>& L, std::ostream& os);
template<class T> rai::String listString(const rai::Array<T*>& L);
template<class T> void listRead(rai::Array<T*>& L, std::istream& is, const char* delim=nullptr);
template<class T> void listCopy(rai::Array<T*>& L, const rai::Array<T*>& M);  //copy a list by calling the copy constructor for each element
template<class T> void listCopy(rai::Array<std::shared_ptr<T>>& L, const rai::Array<std::shared_ptr<T>>& M);  //copy a list by calling the copy constructor for each element
template<class T> void listClone(rai::Array<T*>& L, const rai::Array<T*>& M); //copy a list by calling the 'newClone' method of each element (works for virtual types)
template<class T> void listDelete(rai::Array<T*>& L);
template<class T> void listReindex(rai::Array<T*>& L);
template<class T> T* listFindValue(const rai::Array<T*>& L, const T& x);
template<class T> T* listFindByName(const rai::Array<T*>& L, const char* name); //each element needs a 'name' (usually rai::String)
template<class T> T* listFindByType(const rai::Array<T*>& L, const char* type); //each element needs a 'type' (usually rai::String)
template<class T, class LowerOperator> void listSort(rai::Array<T*>& L, LowerOperator lowerop);

//TODO obsolete?
template<class T> rai::Array<T*> getList(const rai::Array<T>& A) {
  rai::Array<T*> L;
  resizeAs(L, A);
  for(uint i=0; i<A.N; i++) L.elem(i) = &A.elem(i);
  return L;
}
template<class T> T* new_elem(rai::Array<T*>& L) { T* e=new T; e->index=L.N; L.append(e); return e; }

//===========================================================================
/// @}
/// @name graphs -- TODO: transfer to graph data structure
/// @{

void graphRandomUndirected(uintA& E, uint n, double connectivity);
void graphRandomFixedDegree(uintA& E, uint N, uint degree);
void graphRandomTree(uintA& E, uint N, uint roots=1);

template<class vert, class edge> edge* graphGetEdge(vert* from, vert* to);
template<class vert, class edge> void graphMakeLists(rai::Array<vert*>& V, rai::Array<edge*>& E);
template<class vert, class edge> void graphRandomUndirected(rai::Array<vert*>& V, rai::Array<edge*>& E, uint N, double connectivity);
template<class vert, class edge> void graphRandomFixedDegree(rai::Array<vert*>& V, rai::Array<edge*>& E, uint N, uint degree);
template<class vert, class edge> void graphRandomLinear(rai::Array<vert*>& V, rai::Array<edge*>& E, uint N);
template<class vert, class edge> void graphConnectUndirected(rai::Array<vert*>& V, rai::Array<edge*>& E);
template<class vert, class edge> void graphLayered(rai::Array<vert*>& V, rai::Array<edge*>& E, const uintA& layers, bool interConnections);
template<class vert, class edge> edge* newEdge(vert* a, vert* b, rai::Array<edge*>& E);
template<class edge> edge* newEdge(uint a, uint b, rai::Array<edge*>& E);
template<class vert, class edge> edge* del_edge(edge* e, rai::Array<vert*>& V, rai::Array<edge*>& E, bool remakeLists);
template<class vert, class edge> void graphWriteDirected(std::ostream& os, const rai::Array<vert*>& V, const rai::Array<edge*>& E);
template<class vert, class edge> void graphWriteUndirected(std::ostream& os, const rai::Array<vert*>& V, const rai::Array<edge*>& E);
template<class vert, class edge> bool graphTopsort(rai::Array<vert*>& V, rai::Array<edge*>& E);
template<class vert, class edge> void graphDelete(rai::Array<vert*>& V, rai::Array<edge*>& E);

/// @}

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
// implementations
//

void linkArray();

#include "array.ipp"

#endif //SWIG

// (note: http://www.informit.com/articles/article.aspx?p=31783&seqNum=2)
