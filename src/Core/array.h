/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "defines.h"
#include "stdint.h"

#include <initializer_list>
#include <tuple>
#include <iostream>
#include <memory>
#include <vector>

using std::endl;

typedef unsigned char byte;
typedef unsigned int uint;

namespace rai {

// default write formatting
extern const char* arrayElemsep;
extern const char* arrayLinesep;
extern const char* arrayBrackets;
extern int64_t globalMemoryTotal, globalMemoryBound;
extern bool globalMemoryStrict;

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

//fwd declarations
template<class T> struct ArrayModRaw;
template<class T> struct ArrayModList;
struct SpecialArray;
struct SparseVector;
struct SparseMatrix;
struct RowShifted;

/** Simple array container to store arbitrary-dimensional arrays (tensors).
  Can buffer more memory than necessary for faster
  resize; enables non-const reference of subarrays; enables fast
  memove for elementary types; implements many standard
  array/matrix/tensor operations. Interfacing with ordinary C-buffers is simple.
  Please see also the reference for the \ref array.h
  header, which contains lots of functions that can be applied on
  Arrays. */
template<class T> struct Array {
  T* p;     ///< the pointer on the linear memory allocated
  uint N;   ///< number of elements
  uint nd;  ///< number of dimensions
  uint d0, d1, d2; ///< 0th, 1st, 2nd dim
  uint* d;  ///< pointer to dimensions (for nd<=3 points to d0)
  bool isReference; ///< true if this refers to memory of another array
  uint M;   ///< memory allocated (>=N)
  SpecialArray* special=0; ///< auxiliary data, e.g. if this is a sparse matrics, depends on special type

  static int  sizeT;   ///< constant for each type T: stores the sizeof(T)
  static char memMove; ///< constant for each type T: decides whether memmove can be used instead of individual copies

  typedef bool(*ElemCompare)(const T& a, const T& b); //needs to be < or > (NOT <= !!!)

  /// @name constructors
  Array();
  Array(const Array<T>& a);                 //copy constructor
  Array(Array<T>&& a);                      //move constructor
  explicit Array(uint D0);
  explicit Array(uint D0, uint D1);
  explicit Array(uint D0, uint D1, uint D2);
  Array(std::initializer_list<T> values);
  Array(std::initializer_list<uint> dim, std::initializer_list<T> values);
  explicit Array(const T* p, uint size, bool byReference);
  virtual ~Array();

  /// @name assignments
  Array<T>& operator=(std::initializer_list<T> values);
  Array<T>& operator=(const T& v);
  Array<T>& operator=(const Array<T>& a);

  /// @name iterators
  struct iterator {
    using reference = T&;
    T* p;
    T& operator()() { return *p; } //access to value by user
    void operator++() { p++; }
    reference operator*() { return *p; } //in for(auto& it:array.enumerated())  it is assigned to *iterator
    friend bool operator!=(const iterator& i, const iterator& j) { return i.p!=j.p; }
    T& operator->() { return *p; }
  };
  struct const_iterator {
    using reference = const T&;
    const T* p;
    const T& operator()() { return *p; } //access to value by user
    void operator++() { p++; }
    reference operator*() { return *p; } //in for(auto& it:array.enumerated())  it is assigned to *iterator
    friend bool operator!=(const const_iterator& i, const const_iterator& j) { return i.p!=j.p; }
    const T& operator->() { return *p; }
  };

  iterator begin() { return iterator{p}; }
  const_iterator begin() const { return const_iterator{p}; }
  iterator end() { return iterator{p+N}; }
  const_iterator end() const { return const_iterator{p+N}; }

  /// @name resizing
  Array<T>& resize(uint D0);
  Array<T>& resize(uint D0, uint D1);
  Array<T>& resize(uint D0, uint D1, uint D2);
  Array<T>& resize(const Array<uint>& dim);
  Array<T>& resize(std::initializer_list<uint> dim);
  Array<T>& resize(uint ND, uint* dim);
  Array<T>& reshape(int D0);
  Array<T>& reshape(int D0, int D1);
  Array<T>& reshape(int D0, int D1, int D2);
  Array<T>& reshape(const Array<uint>& dim);
  Array<T>& reshape(std::initializer_list<uint> dim);
  Array<T>& reshape(uint ND, uint* dim);
  Array<T>& resizeCopy(uint D0);
  Array<T>& resizeCopy(uint D0, uint D1);
  Array<T>& resizeCopy(uint D0, uint D1, uint D2);
  Array<T>& resizeCopy(std::initializer_list<uint> dim);
  Array<T>& resizeCopy(const Array<uint>& dim);
  Array<T>& resizeCopy(uint ND, uint* dim);
  Array<T>& resizeAs(const Array<T>& a);
  Array<T>& reshapeAs(const Array<T>& a);
  Array<T>& resizeCopyAs(const Array<T>& a);
  Array<T>& dereference();

  /// @name dimensionality access
  uint dim(uint k) const;
  Array<uint> dim() const;

  /// @name initializing/assigning entries
  Array<T>& clear();
  Array<T>& setZero(byte zero=0);
  void setConst(const T& scalar, int d=-1);
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
  Array<T>& setCarray(const T* buffer, uint D0);
  Array<T>& setCarray(const T** buffer, uint D0, uint D1);
  Array<T>& referTo(const T* buffer, uint n);
  void referTo(const Array<T>& a);
  void referToRange(const Array<T>& a, std::pair<int, int> I);
  void referToRange(const Array<T>& a, int i,  std::pair<int, int> J);
  void referToRange(const Array<T>& a, int i, int j,  std::pair<int, int> K);
  void referToDim(const Array<T>& a, int i);
  void referToDim(const Array<T>& a, uint i, uint j);
  void referToDim(const Array<T>& a, uint i, uint j, uint k);
  void takeOver(Array<T>& a);  //a is cleared (earlier: becomes a reference to its previously owned memory)

  /// @name access by reference (direct memory access)
  Array<T> ref() const; //a reference on this
  Array<T>& noconst() { return *this; }
  T& elem() const;
  T& elem(int i) const;
  T& elem(int i, int j); //access that also handles sparse matrices
  T& elem(const Array<uint>& I) const;
  T& first() const { return elem(0); }
  T& last() const { return elem(-1); }
  T& scalar() const { return elem(); }
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

  /// @name access by copy
  Array<T> copy() const;
  Array<T> sub(std::pair<int, int> I) const;
  Array<T> sub(std::pair<int, int> I, std::pair<int, int> J) const;
  Array<T> sub(std::pair<int, int> I, std::pair<int, int> J, std::pair<int, int> K) const;
  Array<T> pick(std::pair<int, int> I, Array<uint> cols) const;
  Array<T> pick(Array<uint> elems) const;
  Array<T> row(uint row_index) const;
  Array<T> rows(uint start_row, uint end_row) const;
  Array<T> col(uint col_index) const;
  Array<T> cols(uint start_col, uint end_col) const;

  int findValue(const T& x) const;
  void findValues(Array<uint>& indices, const T& x) const;
  bool contains(const T& x) const { return findValue(x)!=-1; }
  bool contains(const Array<T>& X) const { for(const T& x:X) if(findValue(x)==-1) return false; return true; }

  /// @name appending etc
  T& append();
  Array<T>& append(const T& x);
  Array<T>& append(const T& x, uint multiple);
  Array<T>& append(const Array<T>& x, bool asRow=false);
  Array<T>& append(const T* p, uint n);
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
  Array<T>& insColumns(int i, uint k=1);
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

  /// @name special matrices -- only for double
  double sparsity();
  SparseMatrix& sparse();
  const SparseMatrix& sparse() const;
  SparseVector& sparseVec();
  const SparseVector& sparseVec() const;
  RowShifted& rowShifted();
  const RowShifted& rowShifted() const;

  /// @name attached Jacobian -- only for double
  std::unique_ptr<Array<double>> jac=0; ///< optional pointer to Jacobian, to enable autodiff
  void J_setId();
  Array<double>& J();
  Array<double> noJ() const;
  Array<double> J_reset();

  /// @name I/O
  void write(std::ostream& os=std::cout, const char* ELEMSEP=nullptr, const char* LINESEP=nullptr, const char* BRACKETS=nullptr, bool dimTag=false, bool binary=false) const;
  Array<T>& read(std::istream& is);
  void writeTagged(std::ostream& os, const char* tag, bool binary=false) const;
  bool readTagged(std::istream& is, const char* tag);
  void writeDim(std::ostream& os=std::cout) const;
  void readDim(std::istream& is);
  void writeJson(std::ostream& os=std::cout) const;
  void readJson(std::istream& is, bool skipType=false);
  void writeBase64(std::ostream& os) const;
  void readBase64(std::istream& is);

  /// modifiers
  ArrayModRaw<T> modRaw() const;
  ArrayModList<T> modList() const;

  // noArr special
  bool operator!() const;
  Array<T>& setNoArr();

//protected:
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
/// @}
/// @name Array operators
/// @{

template<class T> Array<T> operator, (const Array<T>& y, const Array<T>& z); //concat
template<class T> bool operator==(const Array<T>& v, const Array<T>& w); //equal in size and all elements
template<class T> Array<byte> operator==(const Array<T>& v, const T& w); //element-wise equal
template<class T> bool operator!=(const Array<T>& v, const Array<T>& w);
template<class T> std::istream& operator>>(std::istream& is, Array<T>& x);
template<class T> std::ostream& operator<<(std::ostream& os, const Array<T>& x);

template<class T> void operator+=(Array<T>& x, const Array<T>& y);
template<class T> void operator+=(Array<T>& x, const T& y);
template<class T> void operator-=(Array<T>& x, const Array<T>& y);
template<class T> void operator-=(Array<T>& x, const T& y);
template<class T> void operator*=(Array<T>& x, const T& y);

template<class T> Array<T> operator+(const Array<T>& y, const Array<T>& z) { Array<T> x(y); x+=z; return x; }
template<class T> Array<T> operator+(const Array<T>& y, T z) {                Array<T> x(y); x+=z; return x; }
template<class T> Array<T> operator-(const Array<T>& y, const Array<T>& z) { Array<T> x(y); x-=z; return x; }
template<class T> Array<T> operator-(const Array<T>& y, T z) {                Array<T> x(y); x-=z; return x; }

//IO modifiers
template <class T> struct ArrayModRaw {
  const Array<T>* x;
  ArrayModRaw(const Array<T>* x) : x(x) {}
  void write(std::ostream& os) const {
    x->write(os, " ", "\n", "  ");
  }
};
template <class T> ArrayModRaw<T> Array<T>::modRaw() const { return ArrayModRaw<T>(this); }
template <class T> std::ostream& operator<<(std::ostream& os, const ArrayModRaw<T>& x) { x.write(os); return os; }

template <class T> struct ArrayModList {
  const Array<T>* x;
  ArrayModList(const Array<T>* x) : x(x) {}
  void write(std::ostream& os) const {
    for(uint i=0; i<x->N; i++) { if(i) os <<' ';  if(x->elem(i)) os <<*x->elem(i); else os <<"<NULL>"; }
  }
};
template <class T> ArrayModList<T> Array<T>::modList() const { return ArrayModList<T>(this); }

template <class T> std::ostream& operator<<(std::ostream& os, const ArrayModList<T>& x) { x.write(os); return os; }

} //namespace

//===========================================================================
/// @}
/// @name standard types
/// @{

typedef rai::Array<double> arr;
typedef rai::Array<uint>   uintA;
typedef rai::Array<int>    intA;
typedef rai::Array<char>   charA;
typedef rai::Array<float>  floatA;
typedef rai::Array<byte>   byteA;
typedef rai::Array<byte>   boolA;
typedef rai::Array<int16_t>   int16A;
typedef rai::Array<uint16_t>   uint16A;
typedef rai::Array<uint32_t>   uint32A;
typedef rai::Array<intA>   intAA;
typedef rai::Array<uintA>  uintAA;

namespace rai { struct String; }
typedef rai::Array<rai::String> strA;
typedef rai::Array<rai::String> strAA;
typedef rai::Array<rai::String> StringA;
typedef rai::Array<StringA> StringAA;
typedef rai::Array<rai::String*> StringL;

//===========================================================================
//
// creators
//

namespace rai {

/// return array of c's
template<class T> Array<T> consts(const T& c, const uintA& d)  {  Array<T> z;  z.resize(d);  z.setConst(c);  return z; }
/// return VECTOR of c's
template<class T> Array<T> consts(const T& c, uint n) { return consts(c, uintA{n}); }
/// return matrix of c's
template<class T> Array<T> consts(const T& c, uint d0, uint d1) { return consts(c, uintA{d0, d1}); }
/// return tensor of c's
template<class T> Array<T> consts(const T& c, uint d0, uint d1, uint d2) { return consts(c, uintA{d0, d1, d2}); }

inline uintA range(uint n) { uintA r;  r.setStraightPerm(n);  return r; }
inline uintA randperm(uint n) {  uintA z;  z.setRandomPerm(n);  return z; }

template<class T> Array<T*> getCarray(const Array<T>& data) {
  CHECK_EQ(data.nd, 2, "only 2D array gives C-array of type T**");
  Array<T*> Cpointers(data.d0);
  for(uint i=0; i<data.d0; i++) Cpointers(i)=data.p+i*data.d1;
  return Cpointers;
}

template<class T> Array<Array<T>> getArrayArray(const Array<T>& data) {
  CHECK_EQ(data.nd, 2, "only 2D array gives C-array of type T**");
  Array<Array<T>> xx(data.d0);
  for(uint i=0; i<data.d0; i++) xx(i).referTo(data.p+i*data.d1, data.d1);
  return xx;
}

/** @brief return a `dim'-dimensional grid with `steps' intervals
  filling the range [lo, hi] in each dimension. Note: returned array is
  `flat', rather than grid-shaped. */
template<class T> Array<T> grid(uint dim, T lo, T hi, uint steps);

}

//===========================================================================
/// @}
/// @name concatenating arrays together
/// @{

namespace rai {
template<class T> Array<T> catCol(const rai::Array<rai::Array<T>*>& X);
template<class T> Array<T> catCol(std::initializer_list<rai::Array<T>> X);
template<class T> Array<T> catCol(const rai::Array<rai::Array<T>>& X);
template<class T> Array<T> catCol(const rai::Array<T>& a, const rai::Array<T>& b) { return catCol(rai::Array<rai::Array<T>*> {(rai::Array<T>*)&a, (rai::Array<T>*)&b}); }
template<class T> Array<T> catCol(const rai::Array<T>& a, const rai::Array<T>& b, const rai::Array<T>& c) { return catCol(rai::Array<rai::Array<T>*> {(rai::Array<T>*)&a, (rai::Array<T>*)&b, (rai::Array<T>*)&c}); }
template<class T> Array<T> catCol(const rai::Array<T>& a, const rai::Array<T>& b, const rai::Array<T>& c, const rai::Array<T>& d) { return catCol(rai::Array<rai::Array<T>*> {(rai::Array<T>*)&a, (rai::Array<T>*)&b, (rai::Array<T>*)&c, (rai::Array<T>*)&d}); }
template<class T> Array<T> catCol(const rai::Array<T>& a, const rai::Array<T>& b, const rai::Array<T>& c, const rai::Array<T>& d, const rai::Array<T>& e) { return catCol(rai::Array<rai::Array<T>*> {(rai::Array<T>*)&a, (rai::Array<T>*)&b, (rai::Array<T>*)&c, (rai::Array<T>*)&d, (rai::Array<T>*)&e}); }
}

//===========================================================================
/// @}
/// @name twice template functions
/// @{

namespace rai {
template<class T, class S> void resizeAs(Array<T>& x, const Array<S>& a) {
  x.nd=a.nd; x.d0=a.d0; x.d1=a.d1; x.d2=a.d2;
  x.resetD();
  if(x.nd>3) { x.d=new uint[x.nd];  memmove(x.d, a.d, x.nd*sizeof(uint)); }
  x.resizeMEM(a.N, false);
}
template<class T, class S> void resizeCopyAs(Array<T>& x, const Array<S>& a);
template<class T, class S> void reshapeAs(Array<T>& x, const Array<S>& a);
template<class T, class S> void copy(Array<T>& x, const Array<S>& a) {
  resizeAs(x, a);
  T* xp=x.p, *xstop = xp+x.N;;
  S* ap=a.p;
  for(; xp!=xstop; xp++, ap++) *xp = (T)*ap;
  //for(uint i=0; i<x.N; i++) x.elem(i)=(T)a.elem(i);
}
template<class T, class S> Array<T> convert(const Array<S>& a) {
  Array<T> x;
  copy<T, S>(x, a);
  return x;
}
/// check whether this and \c a have same dimensions
template<class T, class S>
bool samedim(const Array<T>& a, const Array<S>& b) {
  return (b.nd==a.nd && b.d0==a.d0 && b.d1==a.d1 && b.d2==a.d2);
}
}

//===========================================================================
//
/// @name set operations
//

namespace rai {

/// x becomes the section of y and z
template<class T> Array<T> setSection(const Array<T>& y, const Array<T>& z) {
  Array<T> x;
  x.reserveMEM(y.N<z.N?y.N:z.N);
  T* yp=y.p, *zp=z.p, *ystop=y.p+y.N, *zstop=z.p+z.N;
  for(yp=y.p; yp!=ystop; yp++) {
    for(zp=z.p; zp!=zstop; zp++) {
      if(*yp==*zp) {
        x.append(*yp); break;
      }
    }
  }
  return x;
}

/// x becomes the section of y and z
template<class T> Array<T> setUnion(const Array<T>& y, const Array<T>& z) {
  Array<T> x;
  uint i, j;
  if(&x!=&y) x=y;
  for(i=0; i<z.N; i++) {
    for(j=0; j<y.N; j++) if(z(i)==y(j)) break;
    if(j==y.N) x.append(z(i));
  }
  return x;
}

/// x becomes the section of y and z
template<class T> void setMinus(Array<T>& x, const Array<T>& y) {
  uint i, j;
  for(i=0; i<x.N;) {
    for(j=0; j<y.N; j++) {
      if(x(i)==y(j)) {
        x.remove(i);
        break;
      }
    }
    if(j==y.N) i++;
  }
}

/// x becomes the section of y and z
template<class T> Array<T> setSectionSorted(const Array<T>& x, const Array<T>& y, bool (*comp)(const T& a, const T& b)) {
  Array<T> R;
//  R.anticipateMEM(MIN(x.N,y.N));
  T* xp=x.p, *yp=y.p, *xstop=x.p+x.N, *ystop=y.p+y.N;
  while(xp!=xstop && yp!=ystop) {
    if(*xp==*yp) {
      R.append(*xp);
      xp++;
      yp++;
    } else {
      if(comp(*xp, *yp)) xp++;
      else yp++;
    }
  }
  return R;
}

template<class T> void setMinusSorted(Array<T>& x, const Array<T>& y, bool (*comp)(const T& a, const T& b)) {
#if 1
  int i=x.N-1, j=y.N-1;
  if(j<0) return;
  if(i<0) return;
  for(;;) {
    while(j>=0 && !comp(y.elem(j), x.elem(i))) j--;
    if(j<0) break;
    while(i>=0 && !comp(x.elem(i), y.elem(j))) i--;
    if(i<0) break;
    if(x.elem(i)==y.elem(j)) { x.remove(i); i--; }
    if(i<0) break;
  }
#else
  T* yp=y.p, *ystop=y.p+y.N;
  for(uint i=0; i<x.N;) {
    while(yp!=ystop && comp(*yp, x.elem(i))) yp++;
    if(yp==ystop) break;
    if(*yp==x.elem(i)) x.remove(i);
    else i++;
  }
#endif
}

/// share x and y at least one element?
template<class T> uint numberSharedElements(const Array<T>& x, const Array<T>& y) {
  Array<T> z = setSection(x, y);
  return z.N;
}

}

//===========================================================================
//
// special iterators

namespace rai {

template<class T>
struct ArrayItEnumerated {
  T* p;
  uint count;
  T& operator()() { return *p; } //access to value by user
  T& operator->() { return *p; }
  void operator++() { p++; count++; }
  ArrayItEnumerated& operator*() { return *this; } //in for(auto& it:array.enumerated())  it is assigned to *iterator
  friend bool operator!=(const ArrayItEnumerated& i, const ArrayItEnumerated& j) { return i.p!=j.p; }
};

template<class T>
struct ArrayIterationEnumerated {
  const Array<T>& x;
  ArrayIterationEnumerated(const Array<T>& x):x(x) {}
  ArrayItEnumerated<T> begin() { return {x.p, 0}; }
  ArrayItEnumerated<T> end() { return {x.p+x.N, x.N}; }
};

template<class T>
ArrayIterationEnumerated<T> enumerated(const Array<T>& x) { return ArrayIterationEnumerated<T>(x); }

template<class T>
struct ArrayItReverse {
  T* p;
  T& operator*() { return *p; } //access to value by user
  void operator++() { p--; }
  friend bool operator!=(const ArrayItReverse& i, const ArrayItReverse& j) { return i.p!=j.p; }
};

template<class T>
struct ArrayIterationReversed {
  const Array<T>& x;
  ArrayIterationReversed(const Array<T>& x):x(x) {}
  ArrayItReverse<T> begin() { return {x.p+x.N-1}; }
  ArrayItReverse<T> end() { return {x.p-1}; }
};

template<class T>
ArrayIterationReversed<T> reversed(const Array<T>& x) { return ArrayIterationReversed<T>(x); }

} //namespace

//===========================================================================

namespace rai {

template<class T> char listWrite(const rai::Array<T*>& L, std::ostream& os, const char* ELEMSEP=" ", const char* delim=0) {
  if(delim) os <<delim[0];
  for(uint i=0; i<L.N; i++) { if(i) os <<ELEMSEP;  if(L.elem(i)) os <<*L.elem(i); else os <<"<NULL>"; }
  if(delim) os <<delim[1] <<std::flush;
  return '#';
}

template<class T> void listDelete(rai::Array<T*>& L) {
  for(uint i=L.N; i--;) delete L.elem(i);
  L.clear();
}

template<class T> void listResizeCopy(rai::Array<T*>& L, uint N) {
  if(L.N<N) {
    uint n=L.N;
    L.resizeCopy(N);
    for(uint i=n; i<N; i++) L.elem(i)=new T();
  } else {
    for(uint i=N; i<L.N; i++) { delete L.elem(i); L.elem(i)=NULL; }
    L.resizeCopy(N);
  }
}

}

//===========================================================================
//
// arr specific
//

namespace rai {

struct SpecialArray {
  enum Type { ST_none, ST_NoArr, ST_EmptyShape, hasCarrayST, sparseVectorST, sparseMatrixST, diagST, RowShiftedST, CpointerSdouble };
  Type type;
  SpecialArray(Type _type=ST_none) : type(_type) {}
  SpecialArray(const SpecialArray&) = delete; //non-copyable
  virtual ~SpecialArray() {}
  SpecialArray& operator=(const SpecialArray&) = delete; //non-copyable
};

template<class T> bool Array<T>::operator!() const { return special && special->type==SpecialArray::ST_NoArr; }
template<class T> Array<T>& Array<T>::setNoArr() { special = new SpecialArray(SpecialArray::ST_NoArr); return *this; }

}

//===========================================================================
//
// numerical operations, also for non double arrays
//

namespace rai {
uint product(const uintA& x);
uint max(const uintA& x);
uint sum(const uintA& x);
float sum(const floatA& x);
template<class T> Array<T> integral(const Array<T>& x);
template<class T> Array<T> differencing(const Array<T>& x, uint width=1);

template<class T>
void tensorPermutation(Array<T>& Y, const Array<T>& X, const uintA& Yid);
}

//===========================================================================
//
// base 64 encoding
//

namespace rai {
int b64_codeLen(uint data_len);
uint b64_maxDataLen(uint code_len);
void b64_encode(char* code, int code_len, const char* data, int data_len);
void b64_decode(char* data, int data_len, const char* code, int code_len);
}

//===========================================================================
//
// conversions
//

template<class T> rai::Array<T> as_arr(const std::vector<T>& a, bool byReference) {
  return rai::Array<T>(&a.front(), a.size(), byReference);
}

template<class T> std::vector<T> as_vector(const rai::Array<T>& a) {
  return std::vector<T>(a.p, a.p+a.N);
}

//===========================================================================
//
// implementations
//

#include "array.ipp"
#include "arrayDouble.h"
