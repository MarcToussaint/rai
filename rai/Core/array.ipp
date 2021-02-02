/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "array.h"
#include "util.h"

#include <math.h>
#include <algorithm>
#include <sstream>

#define maxRank 30
/** @brief if flexiMem is true (which is default!) the resize method will
  (1) at the first call allocate the exact amount of memory, (2)
  at further calls of increasing memory allocate twice the memory
  needed or (3) at further calls of decreasing memory only free
  the memory if the new size is smaller than a fourth */
#define ARRAY_flexiMem true

//#ifdef RAI_MSVC
//#define RAI_NO_VEC_IMPL
//#endif

//===========================================================================
//
// Array class
//

template<class T> char rai::Array<T>::memMove=(char)-1;
template<class T> int rai::Array<T>::sizeT=-1;

/** @brief Simple array container to store arbitrary-dimensional arrays
  (tensors); can buffer more memory than necessary for faster
  resize; enables non-const reference of subarrays; enables fast
  memove for elementary types; implements many standard
  array/matrix/tensor operations. Please see the fully public attributes at the
  bottom of this page -- everthing is meant to be perfectly
  transparent. Interfacing with ordinary C-buffers is simple,
  e.g. via \c Array::referTo (Cbuffer, size) and \c Array::p and \c
  Array::pp. Please see also the reference for the \ref array.h
  header, which contains lots of functions that can be applied on
  Arrays. */

//***** constructors

/// standard constructor -- this becomes an empty array
template<class T> rai::Array<T>::Array()
  : /*std::vector<T>(),*/
    p(0),
    N(0),
    nd(0),
    d0(0), d1(0), d2(0),
    d(&d0),
    isReference(false),
    M(0),
    special(0) {
  if(sizeT==-1) sizeT=sizeof(T);
  if(memMove==(char)-1) {
    memMove=0;
    if(typeid(T)==typeid(bool) ||
        typeid(T)==typeid(char) ||
        typeid(T)==typeid(unsigned char) ||
        typeid(T)==typeid(int) ||
        typeid(T)==typeid(unsigned int) ||
        typeid(T)==typeid(short) ||
        typeid(T)==typeid(unsigned short) ||
        typeid(T)==typeid(long) ||
        typeid(T)==typeid(unsigned long) ||
        typeid(T)==typeid(float) ||
        typeid(T)==typeid(double)) memMove=1;
  }
}

/// copy constructor
template<class T> rai::Array<T>::Array(const rai::Array<T>& a) : Array() { operator=(a); }

/// copy constructor
template<class T> rai::Array<T>::Array(rai::Array<T>&& a)
  : /*std::vector<T>(std::move(a)),*/
    p(a.p),
    N(a.N),
    nd(a.nd),
    d0(a.d0), d1(a.d1), d2(a.d2),
    d(&d0),
    isReference(a.isReference),
    M(a.M),
    special(a.special) {
  CHECK_EQ(a.d, &a.d0, "");
  a.p=NULL;
  a.N=a.nd=a.d0=a.d1=a.d2=0;
  a.isReference=false;
  a.special=NULL;
}

/// constructor with resize
template<class T> rai::Array<T>::Array(uint i) : Array() { resize(i); }

/// constructor with resize
template<class T> rai::Array<T>::Array(uint i, uint j) : Array() { resize(i, j); }

/// constructor with resize
template<class T> rai::Array<T>::Array(uint i, uint j, uint k) : Array() { resize(i, j, k); }

/// this becomes a reference on the C-array \c p
template<class T> rai::Array<T>::Array(const T* p, uint size, bool byReference) : Array() { if(byReference) referTo(p, size); else setCarray(p, size); }

template<class T> rai::Array<T>::Array(const std::vector<T>& a, bool byReference) : Array() { if(byReference) referTo(&a.front(), a.size()); else setCarray(&a.front(), a.size()); }

/// initialization via {1., 2., 3., ...} lists..
template<class T> rai::Array<T>::Array(std::initializer_list<T> values) : Array() { operator=(values); }

/// initialization via {1., 2., 3., ...} lists, with certain dimensionality
template<class T> rai::Array<T>::Array(std::initializer_list<uint> dim, std::initializer_list<T> values) : Array() { operator=(values); reshape(dim); }

template<class T> rai::Array<T>::Array(SpecialArray* _special) : Array() { special=_special; }

template<class T> rai::Array<T>::~Array() { clear(); }

template<class T> bool rai::Array<T>::operator!() const {
  CHECK(((char*)this)+1!=(char*)1, "the zero pointer convention is deprecated!");
  return isNoArr<T>(*this);
}

//***** resize

/// frees all memory; this becomes an empty array
template<class T> rai::Array<T>&  rai::Array<T>::clear() {
  if(special) { delete special; special=NULL; }
  freeMEM();
  return *this;
}

/// resize 1D array, discard the previous contents
template<class T> rai::Array<T>& rai::Array<T>::resize(uint D0) { nd=1; d0=D0; resetD(); resizeMEM(d0, false); return *this; }

/// resize but copy the previous contents
template<class T> rai::Array<T>& rai::Array<T>::resizeCopy(uint D0) { nd=1; d0=D0; resetD(); resizeMEM(d0, true); return *this; }

/// reshape the dimensionality (e.g. from 2D to 1D); throw an error if this actually requires to resize the memory
template<class T> rai::Array<T>& rai::Array<T>::reshape(int D0) {
  if(D0<0) D0=N;
  CHECK_EQ((int)N, D0, "reshape must preserve total memory size");
  nd=1; d0=D0; d1=d2=0; resetD();
  return *this;
}

/// same for 2D ...
template<class T> rai::Array<T>& rai::Array<T>::resize(uint D0, uint D1) { nd=2; d0=D0; d1=D1; resetD(); resizeMEM(d0*d1, false); return *this; }

/// ...
template<class T> rai::Array<T>& rai::Array<T>::resizeCopy(uint D0, uint D1) { nd=2; d0=D0; d1=D1; resetD(); resizeMEM(d0*d1, true); return *this; }

/// ...
template<class T> rai::Array<T>& rai::Array<T>::reshape(int D0, int D1) {
  if(D0<0) D0=N/D1; else if(D1<0) D1=N/D0;
  CHECK_EQ((int)N, D0*D1, "reshape must preserve total memory size");
  nd=2; d0=D0; d1=D1; d2=0;
  resetD();
  return *this;
}

/// same for 3D ...
template<class T> rai::Array<T>& rai::Array<T>::resize(uint D0, uint D1, uint D2) { nd=3; d0=D0; d1=D1; d2=D2; resetD(); resizeMEM(d0*d1*d2, false); return *this; }

/// ...
template<class T> rai::Array<T>& rai::Array<T>::resizeCopy(uint D0, uint D1, uint D2) { nd=3; d0=D0; d1=D1; d2=D2; resetD(); resizeMEM(d0*d1*d2, true); return *this; }

/// ...
template<class T> rai::Array<T>& rai::Array<T>::reshape(int D0, int D1, int D2) {
  if(D0<0) D0=N/(D1*D2); else if(D1<0) D1=N/(D0*D2); else if(D2<0) D2=N/(D0*D1);
  CHECK_EQ((int)N, D0*D1*D2, "reshape must preserve total memory size");
  nd=3; d0=D0; d1=D1; d2=D2;
  resetD();
  return *this;
}

/// resize to multi-dimensional tensor
template<class T> rai::Array<T>& rai::Array<T>::resize(uint ND, uint* dim) {
  nd=ND; d0=d1=d2=0; resetD();
  uint j;
  for(j=0; j<nd && j<3; j++) {(&d0)[j]=dim[j]; }
  if(nd>3) { d=new uint[nd];  memmove(d, dim, nd*sizeof(uint)); }
  uint64_t S;
  for(S=1, j=0; j<nd; j++) S*=dim[j];
  if(S>=(1ull <<32)) HALT("Array #elements " <<(S>>30) <<"G is >= 2^32");
  resizeMEM((uint)S, false);
  return *this;
}

/// resize to multi-dimensional tensor
template<class T> rai::Array<T>& rai::Array<T>::resizeCopy(uint ND, uint* dim) {
  nd=ND; d0=d1=d2=0; resetD();
  uint j;
  for(j=0; j<nd && j<3; j++) {(&d0)[j]=dim[j]; }
  if(nd>3) { d=new uint[nd];  memmove(d, dim, nd*sizeof(uint)); }
  uint64_t S;
  for(S=1, j=0; j<nd; j++) S*=dim[j];
  if(S>=(1ull <<32)) HALT("Array #elements " <<(S>>30) <<"G is >= 2^32");
  resizeMEM((uint)S, true);
  return *this;
}

/// resize to multi-dimensional tensor
template<class T> rai::Array<T>& rai::Array<T>::reshape(uint ND, uint* dim) {
  nd=ND; d0=d1=d2=0; resetD();
  uint j, S;
  for(j=0; j<nd && j<3; j++) {(&d0)[j]=dim[j]; }
  if(nd>3) { d=new uint[nd];  memmove(d, dim, nd*sizeof(uint)); }
  for(S=(nd>0?1:0), j=0; j<nd; j++) S*=dim[j];
  CHECK_EQ(N, S, "reshape must preserve total memory size");
  return *this;
}

/// resize to multi-dimensional tensor
template<class T> rai::Array<T>& rai::Array<T>::resize(const Array<uint>& newD) { resize(newD.N, newD.p); return *this; }

/// resize to multi-dimensional tensor
template<class T> rai::Array<T>& rai::Array<T>::resizeCopy(const Array<uint>& newD) { resizeCopy(newD.N, newD.p); return *this; }

/// resize to multi-dimensional tensor
template<class T> rai::Array<T>& rai::Array<T>::reshape(const Array<uint>& newD) { reshape(newD.N, newD.p); return *this; }

template<class T> rai::Array<T>& rai::Array<T>::reshape(std::initializer_list<uint> dim) { reshape(dim.size(), (uint*)dim.begin()); return *this; }

template<class T> rai::Array<T>& rai::Array<T>::resizeAs(const rai::Array<T>& a) {
  CHECK(this!=&a, "never do this!!!");
  CHECK(!isReference || N==a.N, "resize of a reference (e.g. subarray) is not allowed! (only a resize without changing memory size)");
  nd=a.nd; d0=a.d0; d1=a.d1; d2=a.d2;
  resetD();
  if(nd>3) { d=new uint[nd];  memmove(d, a.d, nd*sizeof(uint)); }
  resizeMEM(a.N, false);
  return *this;
}

/// make it the same size as \c a and copy previous content
template<class T> rai::Array<T>& rai::Array<T>::resizeCopyAs(const rai::Array<T>& a) {
  CHECK(this!=&a, "never do this!!!");
  nd=a.nd; d0=a.d0; d1=a.d1; d2=a.d2; resetD();
  if(nd>3) { d=new uint[nd];  memmove(d, a.d, nd*sizeof(uint)); }
  resizeMEM(a.N, true);
  return *this;
}

template<class T> rai::Array<T>& rai::Array<T>::reshapeAs(const rai::Array<T>& a) {
  CHECK(this!=&a, "never do this!!!");
  CHECK_EQ(N, a.N, "reshape must preserve total memory size");
  nd=a.nd; d0=a.d0; d1=a.d1; d2=a.d2; resetD();
  if(nd>3) { d=new uint[nd];  memmove(d, a.d, nd*sizeof(uint)); }
  return *this;
}

template<class T> rai::Array<T>& rai::Array<T>::reshapeFlat() {
  reshape(N);
  return *this;
}

/// return the size of memory allocated in bytes
template<class T> uint rai::Array<T>::getMemsize() const {
#ifdef RAI_USE_STDVEC
  return vec_type::capacity()*sizeof(T);
#else
  return M*sizeT;
#endif
}

/// I becomes the index tuple for the absolute index i
template<class T> void rai::Array<T>::getIndexTuple(Array<uint>& I, uint i) const {
  uint j;
  CHECK(i<N, "out of range");
  I.resize(nd);
  I.setZero();
  for(j=nd; j--;) {
    I.p[j] = i % d[j];
    i -= I.p[j];
    i /= d[j];
  }
}

/// return the k-th dimensionality
template<class T> uint rai::Array<T>::dim(uint k) const {
  CHECK(k<nd, "dimensionality range check error: " <<k <<"!<" <<nd);
  if(!d && k<3) return (&d0)[k]; else return d[k];
}

//***** internal memory routines (probably not for external use)

#ifdef RAI_CLANG
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wdynamic-class-memaccess"
#endif

#if 0
/// allocate memory (maybe using \ref flexiMem)
template<class T> void rai::Array<T>::resizeMEM(uint n, bool copy, int Mforce) {
  if(n==N && Mforce<0) return; //no change
  CHECK(!reference, "resize of a reference (e.g. subarray) is not allowed! (only a resize without changing memory size)");
  if(n>N && n<=M) { N=n; return; } //memory is big enough, just increase N (copy does not matter)
  uint i;
  T* pold=p;
  uint Mold=M, Mnew;
  //determine a new M (number of allocated items)
  if(Mforce>=0) { //forced size
    Mnew = Mforce;
    CHECK_LE(n, Mnew, "Mforce is smaller than required!");
  } else { //automatic
    if(!ARRAY_flexiMem) {
      Mnew=n;
    } else {
      if(n>0 && Mold==0) {
        Mnew=n;      //first time: exact allocation
      } else if(n>Mold || 10+2*n<Mold/4) {
        Mnew=10+2*n; //big down-or-up-resize: allocate with some extra space
      } else {
        Mnew=Mold;   //small down-size: don't really resize memory
      }
    }
  }
  if(Mnew!=Mold) {  //if M changed, allocate the memory
    uint64_t memoryNew = ((uint64_t)Mnew)*sizeT;
#ifdef RAI_GLOBALMEM
    globalMemoryTotal -= ((uint64_t)Mold)*sizeT;
    globalMemoryTotal += memoryNew;
#endif
    if(Mnew) {
      if(globalMemoryTotal>globalMemoryBound) { //helpers to limit global memory (e.g. to avoid crashing a machine)
        if(globalMemoryStrict) { //undo changes then throw an error
          RAI_MSG("allocating " <<(memoryNew>>20) <<"MB (total=" <<(globalMemoryTotal>>20) <<"M, bound=" <<(globalMemoryBound>>20) <<"M)... (in THREADING applications: undefine the RAI_GLOBALMEM compile flag)");
          globalMemoryTotal += ((uint64_t)Mold)*sizeT;
          globalMemoryTotal -= memoryNew;
          p=pold; M=Mold;
          HALT("strict memory limit exceeded");
        } else { //just give a warning
          if(memoryNew>>20 || globalMemoryTotal-memoryNew<=globalMemoryBound) {
            RAI_MSG("allocating " <<(memoryNew>>20) <<"MB (total=" <<(globalMemoryTotal>>20) <<"M, bound=" <<(globalMemoryBound>>20) <<"M)... (in THREADING applications: undefine the RAI_GLOBALMEM compile flag)");
          }
        }
      }
      p=new T [Mnew];      //p=(T*)malloc(M*sizeT);
      if(!p) { p=pold; M=Mold; HALT("memory allocation failed! Wanted size = " <<Mnew*sizeT <<"bytes"); }
      M=Mnew;
      if(copy) {
        if(memMove==1) {
          memmove(p, pold, sizeT*(N<n?N:n));
          if(n>N) memset(p+N, 0, sizeT*(n-N));
        } else for(i=N<n?N:n; i--;) p[i]=pold[i];
      }
    } else {
      p=0;
    }
    CHECK((pold && Mold) || (!pold && !Mold), "");
    if(Mold) { delete[] pold; pold=NULL; } //if(Mold) free(pold);
  }
  N=n;
}

/// free all memory and reset all pointers and sizes
template<class T> void rai::Array<T>::freeMEM() {
#ifdef RAI_GLOBALMEM
  globalMemoryTotal -= M*sizeT;
#endif
  if(M) { delete[] p; p=NULL; }
  //if(M) free(p);
  //if(special) delete[] special;
  if(d && d!=&d0) { delete[] d; d=NULL; }
  p=NULL;
  M=N=nd=d0=d1=d2=0;
  d=&d0;
  //special=NULL;
  reference=false;
}

//--------- this allocator class doesn't work as expected
template<typename T>
class ArrayAllocator { //      allocate(allocator_type& __a, size_type __n)

 public:
  Array<T>& base;
  std::allocator<T> defaultAlloc;
  typedef std::size_t     size_type;
  typedef T*              pointer;
  typedef T               value_type;

  ArrayAllocator(Array<T>& _base) : base(_base) {}

  ArrayAllocator(const ArrayAllocator& other) throw() : base(other.base) {}

//  template<typename U>
//  ArrayAllocator(const ArrayAllocator<U>& other) throw() : memory_ptr(other.memory_ptr), memory_size(other.memory_size) {};

  template<typename U>
  ArrayAllocator& operator = (const ArrayAllocator<U>& other) { return *this; }
  ArrayAllocator<T>& operator = (const ArrayAllocator& other) { return *this; }
  ~ArrayAllocator() {}

  T* allocate(size_t n, const void* hint = 0) {
    if(!base.reference) {
      base.p = defaultAlloc.allocate(n, hint);
      base.N = base.size();
      base.M = base.capacity();
    }
    return base.p;
  }

  void deallocate(T* ptr, size_t n) {
    if(!base.reference) defaultAlloc.deallocate(ptr, n);
  }

  size_t max_size() const {
    if(base.reference) return base.M;
    return defaultAlloc.max_size();
  }
};

#else
/// allocate memory (maybe using \ref flexiMem)
template<class T> void rai::Array<T>::resizeMEM(uint n, bool copy, int Mforce) {
  if(n==N) return;
  CHECK(!isReference, "resize of a reference (e.g. subarray) is not allowed! (only a resize without changing memory size)");
  CHECK(!isNoArr(*this), "resize of NO-ARRAY is not allowed!");

  //determine a new M (number of allocated items)
  uint Mold=0, Mnew=0;
#ifdef RAI_USE_STDVEC
  Mold = vec_type::size();
#else
  Mold = M;
#endif
  if(Mforce>=0) { //forced size
    Mnew = Mforce;
    CHECK_LE(n, Mnew, "Mforce is smaller than required!");
  } else { //automatic
    if(!ARRAY_flexiMem) {
      Mnew=n;
    } else {
      if(n>0 && Mold==0) {
        Mnew=n;      //first time: exact allocation
      } else if(n>Mold || 10+2*n<Mold/4) {
        Mnew=10+2*n; //big down-or-up-resize: allocate with some extra space
      } else {
        Mnew=Mold;   //small down-size: don't really resize memory
      }
    }
  }

#ifdef RAI_USE_STDVEC
  if(Mnew!=Mold){ vec_type::resize(Mnew); }
//  vec_type::reserve(Mnew);
//  vec_type::resize(Mnew);
  p = vec_type::data();
#else
  CHECK_GE(Mnew, n, "");
  CHECK((p && M) || (!p && !M), "");
  if(Mnew!=Mold) {  //if M changed, allocate the memory
    if(Mnew) {
        if(memMove==1){
            if(p){
                p=(T*)realloc(p, Mnew*sizeT);
            } else {
                p=(T*)malloc(Mnew*sizeT);
//                memset(p, 0, Mnew*sizeT);
            }
            if(!p) { HALT("memory allocation failed! Wanted size = " <<Mnew*sizeT <<"bytes"); }
        }else{
            T* pold = p;
            p=new T [Mnew];
            if(!p) { HALT("memory allocation failed! Wanted size = " <<Mnew*sizeT <<"bytes"); }
            if(copy) for(uint i=N<n?N:n; i--;) p[i]=pold[i];
            if(pold) delete[] pold;
        }
        M=Mnew;
    } else {
      if(p) {
          if(memMove==1){
              free(p);
          }else{
              delete[] p;
          }
          p=0;
          M=0;
      }
    }
  }
#endif
  N = n;
  if(N) CHECK(p, "");
}

/// free all memory and reset all pointers and sizes
template<class T> void rai::Array<T>::freeMEM() {
#ifdef RAI_USE_STDVEC
  vec_type::clear();
#else
  if(M) {
      if(memMove==1){
          free(p);
      }else{
          delete[] p;
      }
      p=0;
      M=0;
  }
#endif
  if(d && d!=&d0) { delete[] d; d=NULL; }
  p=NULL;
  N=nd=d0=d1=d2=0;
  d=&d0;
  isReference=false;
}
#endif

///this was a reference; becomes a copy
template<class T> rai::Array<T>& rai::Array<T>::dereference() {
  CHECK(isReference, "can only dereference a reference!");
  NIY; //not for the new vector versoin..
  uint n=N;
  T* pold=p;
  isReference=false;
  N=0;
  p=NULL;
  resizeMEM(n, false);
  CHECK_EQ(memMove, 1, "only with memmove");
  memmove(p, pold, sizeT*N);
  return *this;
}

/// reset the dimensionality pointer d to point to &d0
template<class T> void rai::Array<T>::resetD() {
  if(d && d!=&d0) { delete[] d; d=NULL; }
  d=&d0;
}

//***** append, insert & remove

/// append an (uninitialized) element to the array and return its reference -- the array becomes 1D!
template<class T> T& rai::Array<T>::append() {
  if(nd==2 && d1==1)
    resizeCopy(d0+1, d1);
  else
    resizeCopy(N+1);
  return p[N-1];
}

/// append an element to the array -- the array becomes 1D!
template<class T> T& rai::Array<T>::append(const T& x) {
#if 0
  reshape(N);
  vec_type::push_back(x);
  p = vec_type::data();
  d0 = N = vec_type::size();
#else
  resizeCopy(N+1);
  p[N-1]=x;
#endif
  return p[N-1];
}

/// append an element to the array -- the array becomes 1D!
template<class T> void rai::Array<T>::append(const T& x, uint multiple) {
  uint i=N;
  resizeCopy(N+multiple);
  for(; i<N; i++) p[i]=x;
}

/// append another array to the array (by copying it) -- the array might become 1D!
template<class T> void rai::Array<T>::append(const rai::Array<T>& x) {
  uint oldN=N, xN=x.N, i;
  if(!xN) return;
  if(!nd)
    resizeAs(x);
  else if(nd==2 && x.nd==1 && d1==x.d0)
    resizeCopy(d0+1, d1);
  else if(nd==2 && x.nd==2 && d1==x.d1)
    resizeCopy(d0+x.d0, d1);
  else if(!N)
    resizeAs(x);
  else
    resizeCopy(N+xN);
  if(memMove==1) memmove(p+oldN, x.p, sizeT*xN);
  else for(i=0; i<xN; i++) p[oldN+i]=x.p[i];
}

/// append a C array to the array (by copying it) -- the array might become 1D!
template<class T> void rai::Array<T>::append(const T* q, uint n) {
  uint oldN=N, i;
  if(nd==2 && d1==n)
    resizeCopy(d0+1, d1);
  else
    resizeCopy(N+n);
  if(memMove==1) memmove(p+oldN, q, sizeT*n);
  else for(i=0; i<n; i++) p[n+i]=q[i];
}

/// append an element to the array if it is not included yet -- the array becomes 1D! [TL]
template<class T> void rai::Array<T>::setAppend(const T& x) {
  if(findValue(x) < 0)
    append(x);
}

/// append elements of another array to the array which are not included yet -- the array might become 1D! [TL]
template<class T> void rai::Array<T>::setAppend(const rai::Array<T>& x) {
  for(const T& i:x) setAppend(i);
}

/// remove and return the first element of the array (must have size>1)
template<class T> T rai::Array<T>::popFirst() { T x; x=elem(0); remove(0);   return x; }

/// remove and return the last element of the array (must have size>1)
template<class T> T rai::Array<T>::popLast() { T x=elem(N-1); resizeCopy(N-1); return x; }

/// remove and return the last element of the array (must have size>1)
template<class T> void rai::Array<T>::removeLast() { resizeCopy(N-1); }

/// reverse this array
template<class T> void rai::Array<T>::reverse() {
  rai::Array<T> L2;
  uint i;
  for(i=this->N; i--;) L2.append(this->elem(i));
  *this = L2;
}

/// reverse the rows of this array
template<class T> void rai::Array<T>::reverseRows() {
  CHECK_EQ(this->nd, 2, "Can only reverse rows of 2 dim arrays. nd=" << this->nd);
  rai::Array<T> L2;
  uint i;
  for(i=this->d0; i--;) L2.append(this->operator[](i));
  L2.reshape(this->d0, this->d1);
  *this = L2;
}

/// the array contains `copies' copies of the old one
template<class T> void rai::Array<T>::replicate(uint copies) {
  if(copies<2) return;
  uint i, oldN=N;
  resizeCopy(copies*N);
  if(memMove==1) {
    for(i=0; i<copies; i++) memmove(p+i*oldN, p, sizeT*oldN);
  } else {
    NIY;
  }
}

/// inserts x at the position i -- the array becomes 1D! [only with memMove!]
template<class T> void rai::Array<T>::insert(uint i, const T& x) {
  CHECK(memMove, "only with memMove");
  uint Nold=N;
  resizeCopy(Nold+1);
  if(i<Nold) memmove(p+i+1, p+i, sizeT*(Nold-i));
  p[i]=x;
}

template<class T> void rai::Array<T>::insert(uint i, const Array<T>& x) {
  uint xN=x.N;
  if(!xN) return;
  if(!nd || !N) {
    CHECK_EQ(i, 0, "");
    *this = x;
  } else if(nd==1) {
    CHECK_LE(i, N, "");
    uint oldN=N;
    resizeCopy(N+xN);
    if(i<oldN) memmove(p+i+xN, p+i, sizeT*(oldN-i));
    memmove(p+i, x.p, sizeT*xN);
  } else if(nd==2) {
    CHECK_LE(i, d0, "");
    uint oldN=d0;
    if(x.nd==1 && d1==x.d0) resizeCopy(d0+1, d1);
    else if(x.nd==2 && d1==x.d1) resizeCopy(d0+x.d0, d1);
    else HALT("");
    if(i<oldN) memmove(p+i*d1+xN, p+i*d1, sizeT*(oldN-i)*d1);
    memmove(p+i*d1, x.p, sizeT*xN);
  }
}

/// remove (delete) a subsequence of the array -- the array becomes 1D!  [only with memMove!]
template<class T> void rai::Array<T>::remove(int i, uint n) {
  if(i<0) i+=N;
  CHECK((uint)i<N, "");
  if((uint)i==N-n) { resizeCopy(N-n); return; }
  if(memMove) {
    if(N>i+n) memmove(p+i, p+i+n, sizeT*(N-i-n));
    resizeCopy(N-n);
  } else {
    //RAI_MSG("don't use this!");
    reshape(N);
    for(uint j=i+n; j<N; j++) p[j-n] = p[j];
    resizeCopy(N-n);
  }
}

/// remove some element by permuting the last element in its place -- the array becomes 1D!
template<class T> void rai::Array<T>::removePerm(uint i) {
  p[i]=p[N-1];
  resizeCopy(N-1);
}

/// remove (delete) a subsequence of the array -- the array becomes 1D!  [only with memMove!] (throws error if value does not exist)
template<class T> bool rai::Array<T>::removeValue(const T& x, bool errorIfMissing) {
  if(p[N-1]==x) { //special case: remove last value
    resizeCopy(N-1);
    return true;
  }
  uint i;
  for(i=0; i<N; i++) if(p[i]==x) break;
  if(errorIfMissing) {
    CHECK(i<N, "value to remove not found");
  } else {
    if(i==N) return false;
  }
  remove(i, 1);
  return true;
}

/// remove (delete) a subsequence of the array -- the array becomes 1D!  [only with memMove!]
template<class T> void rai::Array<T>::removeAllValues(const T& x) {
  CHECK(memMove, "only with memMove");
  uint i;
  for(i=0; i<N; i++) if(p[i]==x) { remove(i, 1); i--; }
}

/// replace n elements at pos i by the sequence x -- the array becomes 1D!  [only with memMove!]
template<class T> void rai::Array<T>::replace(uint i, uint n, const rai::Array<T>& x) {
  CHECK(memMove, "only with memMove");
  uint Nold=N;
  if(n==x.N) {
    memmove(p+i, x.p, sizeT*(x.N));
  } else if(n>x.N) {
    memmove(p+i+x.N, p+i+n, sizeT*(Nold-i-n));
    if(i+n<Nold) memmove(p+i, x.p, sizeT*(x.N));
    resizeCopy(Nold-n+x.N);
  } else {
    resizeCopy(Nold+x.N-n);
    if(i+n<Nold) memmove(p+i+x.N, p+i+n, sizeT*(Nold-i-n));
    memmove(p+i, x.p, sizeT*(x.N));
  }
}

/// deletes the i-th row [must be 2D]
template<class T> void rai::Array<T>::delRows(int i, uint k) {
  CHECK(memMove, "only with memMove");
  CHECK_EQ(nd, 2, "only for matricies");
  if(i<0) i+=d0;
  CHECK_GE(i, 0, "range check error");
  CHECK_LE(i+k, d0, "range check error");
  uint n=d1;
  if(i+k<d0) memmove(p+i*n, p+(i+k)*n, sizeT*(d0-i-k)*n);
  resizeCopy(d0-k, n);
}

/// inserts k rows at the i-th row [must be 2D]
template<class T> void rai::Array<T>::insRows(int i, uint k) {
  CHECK(memMove, "only with memMove");
  CHECK_EQ(nd, 2, "only for matricies");
  if(i<0) i+=d0;
  CHECK(i>=0 && i<(int)d0, "range error (" <<i <<">=" <<d0 <<")");
  int n=d0;
  resizeCopy(d0+k, d1);
  if(n>i) memmove(p+(i+k)*d1, p+i*d1, sizeT*d1*(n-i));
  if(k)   memset(p+ i   *d1, 0, sizeT*d1*k);
}

/// deletes k columns starting from the i-th (i==d1 -> deletes the last k columns)
template<class T> void rai::Array<T>::delColumns(int i, uint k) {
  CHECK(memMove, "only with memMove");
  CHECK_EQ(nd, 2, "only for matricies");
  if(!k) return;
  if(i<0) i+=d1;
  CHECK_LE(i+k, d1, "range check error");
  uint n=d1;
  for(uint j=0; j<d0; j++) {
    memmove(p+j*(n-k), p+j*n, sizeT*i);
    memmove(p+j*(n-k)+i, p+j*n+(i+k), sizeT*(n-i-k));
  }
  resizeCopy(d0, n-k);
}

/// inserts k columns at the i-th column [must be 2D]
template<class T> void rai::Array<T>::insColumns(int i, uint k) {
  CHECK(memMove, "only with memMove");
  CHECK_EQ(nd, 2, "only for matricies");
  if(!k) return;
  if(i<0) i+=d1;
  CHECK_LE(i, (int)d1, "range check error");
  uint n=d1;
  resizeCopy(d0, n+k);
  for(uint j=d0; j--;) {
    memmove(p+j*d1+(i+k), p+j*n+i, sizeT*(n-i));
    memset(p+j*d1+i, 0, sizeT*k);
    memmove(p+j*d1, p+j*n, sizeT*i);
  }
}

/// changes the range of one dimension (generalization of ins/delColumn to arbitrary tensors)
template<class T> void rai::Array<T>::resizeDim(uint k, uint dk) {
  if(dim(k)==dk) return;
  uint i;
  uint L=1, R=1, subR=1;
  uintA oldDim(nd);
  for(i=0; i<nd; i++) {
    oldDim(i)=dim(i);
    if(i<k) L*=dim(i);
    if(i>=k) R*=dim(i);
    if(i>k) subR*=dim(i);
  }
  //cout <<oldDim <<" L=" <<L <<" R=" <<R <<" subR=" <<subR <<endl;
  reshape(L, R);
  if(dk>oldDim(k)) {  //need to increase dim
    i=dk-oldDim(k);
    insColumns(R, i*subR);
    oldDim(k)+=i;
  } else {
    i=oldDim(k)-dk;
    delColumns(R, i*subR);
    oldDim(k)-=i;
  }
  reshape(oldDim);
}

//***** access operations

/// return a uint-Array that contains (acutally refers to) the dimensions of 'this'
template<class T> rai::Array<uint> rai::Array<T>::dim() const {
  Array<uint> dims(d, nd, false);
  return dims;
}

/// the \c ith element
template<class T> T& rai::Array<T>::elem(int i) const {
  if(i<0) i+=N;
  CHECK(i>=0 && i<(int)N, "range error (" <<i <<">=" <<N <<")");
  return p[i];
}

/// access that invariantly works for sparse and non-sparse matrices
template<class T> T& rai::Array<T>::elem(int i, int j) {
  if(i<0) i += d0;
  if(j<0) j += d1;
  CHECK(nd==2 && (uint)i<d0 && (uint)j<d1,
        "2D range error (" <<nd <<"=2, " <<i <<"<" <<d0 <<", " <<j <<"<" <<d1 <<")");
  if(isSparseMatrix(*this)) {
    return sparse().addEntry(i, j);
  }
  if(isRowShifted(*this)) {
    return rowShifted().elemNew(i, j);
  }
  return p[i*d1+j];

}

/// multi-dimensional (tensor) access
//template<class T> T& rai::Array<T>::elem(const Array<int> &I) const {
//  CHECK_EQ(I.N , nd, "wrong dimensions");
//  uint i, j;
//  i=0;
//  for(j=0; j<nd; j++){
//    int Ij = I.elem(j);
//    if(Ij<0) Ij += dim(j);
//    i = i*dim(j) + Ij;
//  }
//  return p[i];
//}

/// multi-dimensional (tensor) access
template<class T> T& rai::Array<T>::elem(const Array<uint>& I) const {
  CHECK_EQ(I.N, nd, "wrong dimensions");
  uint i, j;
  i=0;
  for(j=0; j<nd; j++) {
    uint Ij = I.elem(j);
//    if(Ij<0) Ij += dim(j);
    i = i*dim(j) + Ij;
  }
  return p[i];
}

/// a random element
template<class T> T& rai::Array<T>::rndElem() const {
  return elem(rnd(N));
}

/// scalar reference (valid only for a 0-dim or 1-dim array of size 1)
template<class T> T& rai::Array<T>::scalar() const {
  CHECK(nd<=2 && N==1, "scalar range error (nd=" <<nd <<"), N=" <<N <<")");
  return *p;
}

/// reference to the last element
template<class T> T& rai::Array<T>::first() const {
  CHECK(N, "can't take first of none");
  return *p;
}

/// reference to the last element
template<class T> T& rai::Array<T>::last(int i) const {
  CHECK_GE((int)N+i, 0, "can't take last ("<<i<<") from " <<N <<" elements");
  return p[N+i];
}

/// 1D reference access (throws an error if not 1D or out of range)
template<class T> T& rai::Array<T>::operator()(int i) const {
  if(i<0) i += d0;
  CHECK(nd==1 && (uint)i<d0,
        "1D range error (" <<nd <<"=1, " <<i <<"<" <<d0 <<")");
  return p[i];
}

/// 2D reference access
template<class T> T& rai::Array<T>::operator()(int i, int j) const {
  if(i<0) i += d0;
  if(j<0) j += d1;
  CHECK(nd==2 && (uint)i<d0 && (uint)j<d1 && !isSparseMatrix(*this),
        "2D range error (" <<nd <<"=2, " <<i <<"<" <<d0 <<", " <<j <<"<" <<d1 <<")");
  return p[i*d1+j];
}

/// 3D reference access
template<class T> T& rai::Array<T>::operator()(int i, int j, int k) const {
  if(i<0) i += d0;
  if(j<0) j += d1;
  if(k<0) k += d2;
  CHECK(nd==3 && (uint)i<d0 && (uint)j<d1 && (uint)k<d2 && !isSparseMatrix(*this),
        "3D range error (" <<nd <<"=3, " <<i <<"<" <<d0 <<", " <<j <<"<" <<d1 <<", " <<k <<"<" <<d2 <<")");
  return p[(i*d1+j)*d2+k];
}

template<class T> rai::Array<T> rai::Array<T>::ref() const {
  Array<T> x;
  x.referTo(*this);
  return x;
}

template<class T> rai::Array<T> rai::Array<T>::operator()(std::pair<int, int> I) const {
  rai::Array<T> z;
  z.referToRange(*this, I.first, I.second);
  //  if(I.size()==2) z.referToRange(*this, I.begin()[0], I.begin()[1]);
  //  else if(I.size()==0) z.referTo(*this);
  //  else if(I.size()==1) z.referToDim(*this, I.begin()[0]);
  //  else HALT("range list needs 0,1, or 2 entries exactly");
  return z;
}

/// range reference access
template<class T> rai::Array<T> rai::Array<T>::operator()(int i, std::pair<int, int> J) const {
  rai::Array<T> z;
  z.referToRange(*this, i, J.first, J.second);
//  if(J.size()==2)
//  else if(J.size()==0) z.referToDim(*this, i);
//  else if(J.size()==1) z.referToDim(*this, i, J.begin()[0]);
//  else HALT("range list needs 0,1, or 2 entries exactly");
  return z;
}

///// range reference access
//template<class T> rai::Array<T> rai::Array<T>::operator()(uint i, std::initializer_list<int> J) const {
//  CHECK_EQ(J.size(), 0, "the {} notation here must be empty or a pair");
//  rai::Array<T> z;
//  z.referToDim(*this, i);
//  return z;
//}

/// range reference access
template<class T> rai::Array<T> rai::Array<T>::operator()(int i, int j, std::initializer_list<int> K) const {
  rai::Array<T> z;
  if(K.size()==2) z.referToRange(*this, i, j, K.begin()[0], K.begin()[1]);
  else if(K.size()==0) z.referToDim(*this, i, j);
  else if(K.size()==1) z.referToDim(*this, i, j, K.begin()[0]);
  else HALT("range list needs 0,1, or 2 entries exactly");
  return z;
}

/// get a subarray (e.g., row of a matrix); use in conjuction with operator()() to get a reference
template<class T> rai::Array<T> rai::Array<T>::operator[](int i) const {
//  return Array(*this, i);
  rai::Array<T> z;
  z.referToDim(*this, i);
  return z;
}

/// convert a subarray into a reference (e.g. a[3]()+=.123)
//template<class T> T& rai::Array<T>::operator()() const { return scalar(); } //return (*this); }

/// reference to the max entry
template<class T> T& rai::Array<T>::max() const { CHECK(N, ""); uint i, m=0; for(i=1; i<N; i++) if(p[i]>p[m]) m=i; return p[m]; }

/// reference to the min entry
template<class T> T& rai::Array<T>::min() const { CHECK(N, ""); uint i, m=0; for(i=1; i<N; i++) if(p[i]<p[m]) m=i; return p[m]; }

/// gets the min and max
template<class T> void rai::Array<T>::minmax(T& minVal, T& maxVal) const {
  CHECK(N, "");
  uint i;
  minVal=maxVal=p[0];
  for(i=1; i<N; i++) {
    if(p[i]<minVal) minVal=p[i];
    else if(p[i]>maxVal) maxVal=p[i];
  }
}

/*
/// also returns the index (argmin) \c ind
template<class T> T minA(const rai::Array<T>& v, uint & ind, uint start, uint end){
  CHECK(v.N>0, "");
  CHECK(v.N>start, "");
  CHECK_GE(v.N, end, "");
  CHECK_GE(end, start, "");
  T t(v(start));
  ind=start;
  if(end==0) end=v.N;
  for(uint i=start; i<end; ++i) if(v.p[i]<t){
    t  =v.p[i];
    ind=i;
  }
  return t;
}

/// also returns the index (argmax) \c ind
template<class T> T maxA(const rai::Array<T>& v, uint & ind, uint start, uint end){
  CHECK(v.N>0, "");
  CHECK(v.N>start, "");
  CHECK_GE(v.N, end, "");
  CHECK_GE(end, start, "");
  T t(v(start));
  ind=start;
  if(end==0){
    end=v.N;
  }
  for(uint i=start; i<end; ++i) if(v.p[i]>t){
    t  =v.p[i];
    ind=long(i);
  }
  return t;
}
*/

/** @brief the index of the maxium; precondition: the comparision operator
  > exists for type T */
template<class T> uint rai::Array<T>::argmax() const { uint i, m=0; for(i=0; i<N; i++) if(p[i]>p[m]) m=i; return m; }

/** @brief the index of the maxium; precondition: the comparision operator
  > exists for type T */
template<class T> void rai::Array<T>::argmax(uint& i, uint& j) const { CHECK_EQ(nd, 2, "needs 2D array"); j=argmax(); i=j/d1; j=j%d1; }

/** @brief the index of the maxium; precondition: the comparision operator
  > exists for type T */
template<class T> void rai::Array<T>::argmax(uint& i, uint& j, uint& k) const { CHECK_EQ(nd, 3, "needs 3D array"); k=argmax(); i=k/(d1*d2); k=k%(d1*d2); j=k/d2; k=k%d2; }

/// get the maximal and second maximal value
template<class T> void rai::Array<T>::maxIndeces(uint& m1, uint& m2) const {
  uint i;
  if(p[0]>p[1]) { m1=0; m2=1; } else { m1=1; m2=0; }
  for(i=2; i<N; i++) {
    if(p[i]>p[m2]) {  //greater than m2
      if(p[i]>p[m1]) {  //greater than m1
        m2=m1;
        m1=i;
      } else {
        m2=i;
      }
    }
  }
}

/// the index of the minimum; precondition: the comparision operator > exists for type T
template<class T> uint rai::Array<T>::argmin() const { uint i, m=0; for(i=0; i<N; i++) if(p[i]<p[m]) m=i; return m; }

/// return the index of an entry equal to x, or -1
template<class T> int rai::Array<T>::findValue(const T& x) const { uint i; for(i=0; i<N; i++) if(p[i]==x) return i; return -1; }

/// return all indices to entries equal to x
template<class T> void rai::Array<T>::findValues(rai::Array<uint>& indices, const T& x) const {
  indices.clear();
  uint i;
  for(i=0; i<N; i++)
    if(p[i]==x) indices.append(i);
}

/** @brief whether at least one object is contained more than once  */
template<class T> bool rai::Array<T>::containsDoubles() const {
  if(N<2) return false;
  for(uint i=0; i<N; i++) {
    for(uint j=0; j<i; j++) {
      if(p[i]==p[j]) return true;
    }
  }
  return false;
}

/// non-reference copy (to apply followup operators, like x.copy().reshape(3,5))
template<class T> rai::Array<T> rai::Array<T>::copy() const { return rai::Array<T>(*this); }

/** @brief a sub array of a 1D Array (corresponds to matlab [i:I]); when
  the upper limit I is -1, it is replaced by the max limit (like
  [i:]) */
template<class T> rai::Array<T> rai::Array<T>::sub(int i, int I) const {
  CHECK_EQ(nd, 1, "1D range error ");
  rai::Array<T> x;
  if(i<0) i+=d0;
  if(I<0) I+=d0;
  CHECK(i>=0 && I>=0 && i<=I, "lower limit higher than upper!");
  x.resize(I-i+1);
  if(memMove==1) {
    memmove(x.p, p+i, sizeT*x.N);
  } else {
    for(uint ii=0; ii<x.N; ii++) x.p[ii]=p[ii+i];
  }
  return x;
}

/** @brief copies a sub array of a 2D Array (corresponds to matlab [i:I, j:J]);
  when the upper limits I or J are -1, they are replaced by the
  max limit (like [i:, j:]) */
template<class T> rai::Array<T> rai::Array<T>::sub(int i, int I, int j, int J) const {
  CHECK_EQ(nd, 2, "2D range error ");
  rai::Array<T> x;
  if(i<0) i+=d0;
  if(j<0) j+=d1;
  if(I<0) I+=d0;
  if(J<0) J+=d1;
  CHECK(i>=0 && j>=0 && I>=0 && J>=0 && i<=I && j<=J, "lower limit higher than upper!");
  x.resize(I-i+1, J-j+1);
  if(memMove==1) {
    for(uint ii=0; ii<x.d0; ii++) memmove(x.p+(ii*x.d1), p+((ii+i)*d1+j), sizeT*x.d1);
  } else {
    for(uint ii=0; ii<x.d0; ii++) for(uint jj=0; jj<x.d1; jj++) x(ii, jj)=operator()(ii+i, jj+j);
  }
  return x;
}

/** @brief copies a sub array of a 3D Array (corresponds to matlab [i:I, j:J]);
  when the upper limits I or J are -1, they are replaced by the
  max limit (like [i:, j:]) */
template<class T> rai::Array<T> rai::Array<T>::sub(int i, int I, int j, int J, int k, int K) const {
  CHECK_EQ(nd, 3, "3D range error ");
  rai::Array<T> x;
  if(i<0) i+=d0;
  if(j<0) j+=d1;
  if(k<0) k+=d2;
  if(I<0) I+=d0;
  if(J<0) J+=d1;
  if(K<0) K+=d2;
  CHECK(i>=0 && j>=0 && k>=0 && I>=0 && J>=0 && K>=0 && i<=I && j<=J && k<=K, "lower limit higher than upper!");
  x.resize(I-i+1, J-j+1, K-k+1);
  if(memMove==1) {
    for(uint ii=0; ii<x.d0; ii++) for(uint jj=0; jj<x.d1; jj++) {
        memmove(x.p+((ii*x.d1+jj)*x.d2), p+(((ii+i)*d1+jj+j)*d2+k), sizeT*x.d2);
      }
  } else {
    for(uint ii=0; ii<x.d0; ii++) for(uint jj=0; jj<x.d1; jj++) for(uint kk=0; kk<x.d2; kk++)
          x(ii, jj, kk)=operator()(ii+i, jj+j, kk+k);
  }
  return x;
}

/** @brief copies a selection of columns from a 2D array, the first index (rows)
  runs from i to I (as explained above) while the second index runs
  over the columns explicitly referred to by cols. (col doesn't have
  to be ordered or could also contain some columns multiply) */
template<class T> rai::Array<T> rai::Array<T>::sub(int i, int I, Array<uint> cols) const {
  CHECK_EQ(nd, 2, "2D range error ");
  rai::Array<T> x;
  if(i<0) i+=d0;
  if(I<0) I+=d0;
  CHECK(i>=0 && I>=0 && i<=I, "lower limit higher than upper!");
  x.resize(I-i+1, cols.N);
  for(int ii=i; ii<=I; ii++) for(int l=0; l<(int)cols.N; l++) x(ii-i, l)=operator()(ii, cols(l));
  return x;
}

template<class T> rai::Array<T> rai::Array<T>::sub(Array<uint> elems) const {
  rai::Array<T> x;
  if(nd==1) {
    x.resize(elems.N);
    for(int l=0; l<(int)elems.N; l++) x.elem(l)=operator()(elems.elem(l));
  } else if(nd==2) {
    x.resize(elems.N, d1);
    for(int l=0; l<(int)elems.N; l++) for(uint j=0; j<d1; j++) x(l, j)=operator()(elems(l), j);
  } else if(nd==3) {
    x.resize(elems.N, d1, d2);
    for(int l=0; l<(int)elems.N; l++) for(uint j=0; j<d1; j++) for(uint k=0; k<d2; k++) x(l, j, k)=operator()(elems(l), j, k);
  } else NIY;
  return x;
}

/**
 * @brief Return a copy of row `row_index` of the Array.
 *
 * This is just a convenient wrapper around `sub`.
 *
 * @tparam T data type of the array.
 * @param row_index the row to access.
 *
 * @return  copy of row `row_index`.
 */
template<class T>
rai::Array<T> rai::Array<T>::row(uint row_index) const {
  return sub(row_index, row_index, 0, d1 - 1);
}

/**
 * @brief Return a copy of column col_index of the Array.
 *
 * This is just a convenient wrapper around `sub`.
 *
 * @tparam T data type of the array.
 * @param col_index the column to access.
 *
 * @return  copy of column `col_index`.
 */
template<class T>
rai::Array<T> rai::Array<T>::col(uint col_index) const {
  return sub(0, d0 - 1, col_index, col_index).reshape(d0);
}

/**
 * @brief Return a copy of the rows from `start_row` to excluding `end_row`.
 *
 * This is just a convenient wrapper around `sub`.
 *
 * @tparam T Data type of the array.
 * @param start_row Start copying from index `start_row`.
 * @param end_row Stop copying at excluding `end_row`.
 *
 * @return Copy of the rows from `start` to excluding `end_row`.
 */
template<class T>
rai::Array<T> rai::Array<T>::rows(uint start_row, uint end_row) const {
  return sub(start_row, end_row - 1, 0, d1 - 1);
}

/**
 * @brief Return a copy of the columns from column `start_col` to (excluding) `end_col`.
 *
 * This is just a convenient wrapper around `sub`.
 *
 * @tparam T Data type of the array.
 * @param start_col Start copying from index `start_col`.
 * @param end_col Stop copying at excluding `end_col`.
 *
 * @return Copy of the columns from `start_col` to excluding `end_col`.
 */
template<class T>
rai::Array<T> rai::Array<T>::cols(uint start_col, uint end_col) const {
  return sub(0, d0 - 1, start_col, end_col - 1);
}

//***** C-array interfacing

#if 0
/// allocates, sets and returns the \c Array::pp pointer (of type \c T**)
template<class T> T** rai::Array<T>::getCarray() const {
  CHECK_GE(nd, 2, "only 2D or higher-D arrays gives C-array of type T**");
  HALT("I think this is buggy");
  if(special==hasCarrayST) return (T**) special;
  T** pp;
  ((rai::Array<T>*)this)->special = hasCarrayST;
  ((rai::Array<T>*)this)->special = pp = new T* [d0];
  uint skip;
  if(nd==2) skip=d1; else skip=N/d0;
  for(uint i=0; i<d0; i++) pp[i]=p+i*skip;
  return pp;
}
#endif

/// makes this array a reference to the C buffer
template<class T> void rai::Array<T>::referTo(const T* buffer, uint n) {
  freeMEM();
  isReference=true;
  nd=1; d0=N=n; d1=d2=0;
  p=(T*)buffer;
#if 0
  vec_type::_M_impl._M_start = p;
  vec_type::_M_impl._M_finish = p+N;
  vec_type::_M_impl._M_end_of_storage = p+N;
#endif
}

/** @brief returns an ordinary 2-dimensional C-pointer to the Array content.
  Requires the Array<T*> as buffer. */
template<class T> T** rai::Array<T>::getCarray(Array<T*>& Cpointers) const {
  CHECK_EQ(nd, 2, "only 2D array gives C-array of type T**");
  Cpointers.resize(d0);
  for(uint i=0; i<d0; i++) Cpointers(i)=p+i*d1;
  return Cpointers.p;
}

/** @brief returns an ordinary 2-dimensional C-pointer to the Array content.
  Requires the Array<T*> as buffer. */
template<class T> rai::Array<T*> rai::Array<T>::getCarray() const {
  CHECK_EQ(nd, 2, "only 2D array gives C-array of type T**");
  Array<T*> Cpointers(d0);
  for(uint i=0; i<d0; i++) Cpointers(i)=p+i*d1;
  return Cpointers;
}

#if 0
/// returns an ordinary 3-dimensional C-pointer-array
template<class T> T*** rai::Array<T>::getPointers(Array<T**>& array3d, Array<T*>& array2d) const {
  CHECK_EQ(nd, 3, "only 3D array gives C-array of type T*** ");
  array2d.resize(d0, d1);
  for(uint i=0; i<d0; i++) {
    for(uint j=0; j<d1; j++) array2d(i, j)=&operator()(i, j, 0);
    array3d(i)=&array2d(i, 0);
  }
  return array3d.p;
}
#endif

//***** assignments

template<class T> rai::Array<T>& rai::Array<T>::operator=(std::initializer_list<T> values) {
  resize(values.size());
  uint i=0;
  for(T t : values) elem(i++)=t;
  return *this;
}

/// set all elements to value \c v
template<class T> rai::Array<T>& rai::Array<T>::operator=(const T& v) {
  uint i;
  //if(memMove && typeid(T)==typeid(T)) memset(p, *((int*)&v), N); else
//  CHECK(N,"assigning constant to empty array");
  for(i=0; i<N; i++) p[i]=v;
  return *this;
}

/// copy operator
template<class T> rai::Array<T>& rai::Array<T>::operator=(const rai::Array<T>& a) {
  CHECK(this!=&a, "never do this!!!");
  //if(a.temp){ takeOver(*((rai::Array<T>*)&a)); return *this; }
  resizeAs(a);
  if(memMove) memmove(p, a.p, sizeT*N);
  else for(uint i=0; i<N; i++) p[i]=a.p[i];
  if(special) { delete special; special=NULL; }
  if(isSpecial(a)) {
    if(isRowShifted(a)) {
      CHECK(typeid(T) == typeid(double), "");
      special = new RowShifted(*((arr*)this), *dynamic_cast<RowShifted*>(a.special));
    } else if(isSparseVector(a)) {
      CHECK(typeid(T) == typeid(double), "");
      special = new SparseVector(*((arr*)this), *dynamic_cast<SparseVector*>(a.special));
    } else if(isSparseMatrix(a)) {
      CHECK(typeid(T) == typeid(double), "");
      special = new SparseMatrix(*((arr*)this), *dynamic_cast<SparseMatrix*>(a.special));
    } else if(isNoArr(a)){
      setNoArr();
    } else NIY;

  }
  return *this;
}

/// copy operator
template<class T> rai::Array<T>& rai::Array<T>::operator=(const std::vector<T>& a) {
  setCarray(&a.front(), a.size());
  return *this;
}

/** @brief same as memset(p, 0, sizeT*N); precondition: memMove is
  true! */
template<class T> void rai::Array<T>::setZero(byte zero) {
  CHECK(memMove, "can set array's memory to zero only if memMove option is true");
  memset(p, zero, sizeT*N);
}

/// concatenate 2D matrices (or vectors) column-wise
template<class T> rai::Array<T> catCol(const rai::Array<rai::Array<T>*>& X) {
  uint d0=X(0)->d0, d1=0;
  for(rai::Array<T>* x:X) { CHECK((x->nd==2 || x->nd==1) && x->d0==d0, ""); d1+=x->nd==2?x->d1:1; }
  rai::Array<T> z;
  if(X.first()->isSparse()){
      z.sparse().resize(d0, d1, 0);
      d1=0;
      for(rai::Array<T>* x:  X) {
          CHECK(x->isSparse(), "");
          CHECK(x->nd==2,"");
          z.sparse().add(x->sparse(), 0, d1);
          d1+=x->d1;
      }
  }else{
      z.resize(d0, d1);
      d1=0;
      for(rai::Array<T>* x:  X) { z.setMatrixBlock(*x, 0, d1); d1+=x->nd==2?x->d1:1; }
  }
  return z;
}

/// concatenate 2D matrices (or vectors) column-wise
template<class T> rai::Array<T> catCol(const rai::Array<rai::Array<T>>& X) {
  rai::Array<rai::Array<T>*> Xp;
  for(rai::Array<T>& x:  X) Xp.append(&x);
  return catCol(Xp);
}

/// set all entries to same value x [default: don't change dimension]
template<class T> void rai::Array<T>::setUni(const T& x, int d) {
  if(d!=-1) resize(d);
  uint i;
  for(i=0; i<N; i++) elem(i)=x;
}

/** @brief becomes the n-dim identity matrix [default:
  don't change dimension (already has to be squared matrix)] */
template<class T> void rai::Array<T>::setId(int d) {
  CHECK(d!=-1 || (nd==2 && d0==d1), "need squared matrix to set to identity");
  if(d!=-1) resize(d, d);
  setZero();
  for(uint i=0; i<d0; i++) operator()(i, i)=(T)1;
}

template<class T> void rai::Array<T>::setDiag(const T& x, int d) {
  CHECK(d!=-1 || nd==2, "need squared matrix to set to diagonal");
  if(d!=-1) resize(d, d);
  if(d==-1) d=(int)rai::MIN(d0, d1);
  setZero();
  for(uint i=0; i<(uint)d; i++) p[i*d+i]=x; //operator()(i, i)=x;
  //mtype=diagMT;
}

/// sets x to be the diagonal matrix with diagonal v
template<class T> void rai::Array<T>::setDiag(const rai::Array<T>& v) {
  CHECK_EQ(v.nd, 1, "can only give diagonal of 1D array");
  resize(v.d0, v.d0);
  setZero();
  uint i;
  for(i=0; i<v.d0; i++) operator()(i, i)=v(i);
  //mtype=diagMT;
}

/// constructs the block matrix X=[A, B ; C, D]
template<class T> void rai::Array<T>::setBlockMatrix(const rai::Array<T>& A, const rai::Array<T>& B, const rai::Array<T>& C, const rai::Array<T>& D) {
  CHECK(A.nd==2 && B.nd==2 && C.nd==2 && D.nd==2, "");
  CHECK(A.d0==B.d0 && A.d1==C.d1 && B.d1==D.d1 && C.d0==D.d0, "");
  resize(A.d0+C.d0, A.d1+B.d1);
  setMatrixBlock(A,  0, 0);
  setMatrixBlock(B,  0, A.d1);
  setMatrixBlock(C, A.d0, 0);
  setMatrixBlock(D, A.d0, A.d1);
  /*
  for(i=0;i<A.d0;i++) for(j=0;j<A.d1;j++) operator()(i  , j  )=A(i, j);
  for(i=0;i<B.d0;i++) for(j=0;j<B.d1;j++) operator()(i  , j+b)=B(i, j);
  for(i=0;i<C.d0;i++) for(j=0;j<C.d1;j++) operator()(i+a, j  )=C(i, j);
  for(i=0;i<D.d0;i++) for(j=0;j<D.d1;j++) operator()(i+a, j+b)=D(i, j);*/
}

/// constructs the block matrix X=[A; B]
template<class T> void rai::Array<T>::setBlockMatrix(const rai::Array<T>& A, const rai::Array<T>& B) {
  if(isNoArr(*this)) return;
  if(!isSpecial(A)){
    CHECK(A.nd==2 && B.nd==2, "");
    CHECK(A.d1==B.d1, "");
    resize(A.d0+B.d0, A.d1);
    setMatrixBlock(A,  0, 0);
    setMatrixBlock(B, A.d0, 0);
  }else{
    if(A.isSparse()){
      CHECK(B.isSparse(), "");
      CHECK(A.d1==B.d1, "");
      sparse().resize(A.d0+B.d0, A.d1, 0);
      sparse().add(A.sparse(), 0, 0);
      sparse().add(B.sparse(), A.d0, 0);
    } else if(isRowShifted(A)){
      CHECK(isRowShifted(B), "");
      CHECK(A.d1==B.d1, "");
      rowShifted().resize(A.d0+B.d0, A.d1, rai::MAX(A.rowShifted().rowSize, B.rowShifted().rowSize));
      rowShifted().add(A, 0, 0);
      rowShifted().add(B, A.d0, 0);
    } else if(isNoArr(A)){
      CHECK(isNoArr(B), "");
      setNoArr();
    } else NIY;
  }
  /*
  for(i=0;i<A.d0;i++) for(j=0;j<A.d1;j++) operator()(i  , j  )=A(i, j);
  for(i=0;i<B.d0;i++) for(j=0;j<B.d1;j++) operator()(i  , j+b)=B(i, j);
  for(i=0;i<C.d0;i++) for(j=0;j<C.d1;j++) operator()(i+a, j  )=C(i, j);
  for(i=0;i<D.d0;i++) for(j=0;j<D.d1;j++) operator()(i+a, j+b)=D(i, j);*/
}

/// constructs a vector x=[a, b]
template<class T> void rai::Array<T>::setBlockVector(const rai::Array<T>& a, const rai::Array<T>& b) {
  CHECK(a.nd==1 && b.nd==1, "");
  resize(a.N+b.N);
  setVectorBlock(a, 0);   //for(i=0;i<a.N;i++) operator()(i    )=a(i);
  setVectorBlock(b, a.N); //for(i=0;i<b.N;i++) operator()(i+a.N)=b(i);
}

/// write the matrix B into 'this' matrix at location lo0, lo1
template<class T> void rai::Array<T>::setMatrixBlock(const rai::Array<T>& B, uint lo0, uint lo1) {
  CHECK(B.nd==1 || B.nd==2, "");
  if(B.nd==2) {
    CHECK(nd==2 && lo0+B.d0<=d0 && lo1+B.d1<=d1, "");
    uint i, j;
    if(!isSpecial(*this)) {
      CHECK(!isSparseMatrix(B), "");
      if(memMove) {
        for(i=0; i<B.d0; i++) memmove(p+(lo0+i)*d1+lo1, B.p+i*B.d1, B.d1*sizeT);
      } else {
        for(i=0; i<B.d0; i++) for(j=0; j<B.d1; j++) p[(lo0+i)*d1+lo1+j] = B.p[i*B.d1+j];   // operator()(lo0+i, lo1+j)=B(i, j);
      }
    } else if(isSparseMatrix(*this)) {
#if 1
        sparse().add(B, lo0, lo1);
#else
      if(!isSparseMatrix(B)) {
        for(i=0; i<B.d0; i++) for(j=0; j<B.d1; j++){
            double z = B.p[i*B.d1+j];
            if(z) sparse().addEntry(lo0+i, lo1+j) = z;
        }
      } else {
        SparseMatrix& S = sparse();
        const SparseMatrix& BS = B.sparse();
        for(i=0; i<B.N; i++) {
          S.addEntry(lo0 + BS.elems(i, 0), lo1 + BS.elems(i, 1)) = B.elem(i);
        }
      }
#endif
    } else if(isRowShifted(*this)) {
      rowShifted().add(B, lo0, lo1);
    }
  } else {
    CHECK(nd==2 && lo0+B.d0<=d0 && lo1+1<=d1, "");
    uint i;
    if(!isSparseMatrix(*this)) {
      for(i=0; i<B.d0; i++) p[(lo0+i)*d1+lo1] = B.p[i];  // operator()(lo0+i, lo1+j)=B(i, j);
    } else {
      for(i=0; i<B.d0; i++) sparse().addEntry(lo0+i, lo1) = B.p[i];
    }
  }
}

/// return a sub-matrix of 'this' taken at location lo0, lo1
template<class T> void rai::Array<T>::getMatrixBlock(rai::Array<T>& B, uint lo0, uint lo1) const {
  CHECK(nd==2 && B.nd==2 && lo0+B.d0<=d0 && lo1+B.d1<=d1, "");
  uint i, j;
  for(i=0; i<B.d0; i++) for(j=0; j<B.d1; j++) B(i, j)=operator()(lo0+i, lo1+j);
}

/// write the vector B into 'this' vector at location lo0
template<class T> void rai::Array<T>::setVectorBlock(const rai::Array<T>& B, uint lo) {
  CHECK(nd==1 && B.nd==1 && lo+B.N<=N, "");
  uint i;
  for(i=0; i<B.N; i++) operator()(lo+i)=B(i);
}

/// B (needs to be sized before) becomes sub-vector of 'this' at location lo
template<class T> void rai::Array<T>::getVectorBlock(rai::Array<T>& B, uint lo) const {
  CHECK(nd==1 && B.nd==1 && lo+B.N<=N, "");
  uint i;
  for(i=0; i<B.N; i++) B(i)=operator()(lo+i);
}

/// sorted permutation of length \c n
template<class T> void rai::Array<T>::setStraightPerm(int n) {
  if(n!=-1) resize(n);
  for(uint i=0; i<N; i++) elem(i)=(T)i;
}

/// reverse sorted permutation of lenth \c N
template<class T> void rai::Array<T>::setReversePerm(int n) {
  if(n!=-1) resize(n);
  for(uint i=0; i<N; i++) elem(N-1-i)=(T)i;
}

/// permute all elements randomly
template<class T> void rai::Array<T>::setRandomPerm(int n) {
  setStraightPerm(n);
  int j, r;
  for(j=N-1; j>=1; j--) {
    r=rnd(j+1);
    permute(r, j);
  }
}

/// 'this' becomes a copy (not reference to!) of the 1D C array
template<class T> void rai::Array<T>::setCarray(const T* buffer, uint D0) {
  if(N!=D0) resize(D0);
  uint i;
  if(memMove && typeid(T)==typeid(T))
    memmove(p, buffer, sizeT*d0);
  else for(i=0; i<d0; i++) operator()(i)=(T)buffer[i];
}

/// 'this' becomes a copy (not reference to!) of the 2D C array
template<class T> void rai::Array<T>::setCarray(const T** buffer, uint D0, uint D1) {
  resize(D0, D1);
  uint i, j;
  for(i=0; i<d0; i++) {
    if(memMove && typeid(T)==typeid(T))
      memmove(p+i*d1, buffer[i], sizeT*d1);
    else for(j=0; j<d1; j++) operator()(i, j)=(T)buffer[i][j];
  }
}

/// copy 'this' into a C array
template<class T> void rai::Array<T>::copyInto(T* buffer) const {
  if(memMove && typeid(T)==typeid(T)) memmove(buffer, p, sizeT*N);
  else for(uint i=0; i<N; i++) buffer[i]=(T)elem(i);
}

/// copy 'this' into a C array
template<class T> void rai::Array<T>::copyInto2D(T** buffer) const {
  CHECK_EQ(nd, 2, "can only copy 2D Array into 2D C-array");
  uint i, j;
  for(i=0; i<d0; i++) {
    if(memMove && typeid(T)==typeid(T)) memmove(buffer[i], p+i*d1, sizeT*d1);
    else for(j=0; j<d1; j++) buffer[i][j]=(T)operator()(i, j);
  }
}

/// make this array a reference to the array \c a
template<class T> void rai::Array<T>::referTo(const rai::Array<T>& a) {
  referTo(a.p, a.N);
  reshapeAs(a);
}

/// make this array a subarray reference to \c a
template<class T> void rai::Array<T>::referToRange(const rai::Array<T>& a, int i_lo, int i_up) {
  CHECK_LE(a.nd, 3, "not implemented yet");
  if(i_lo<0) i_lo+=a.d0;
  if(i_up<0) i_up+=a.d0;
  if(i_lo>i_up) { clear(); return; }
  CHECK((uint)i_lo<a.d0 && (uint)i_up<a.d0, "SubRange range error (" <<i_lo <<"<" <<a.d0 <<", " <<i_up <<"<" <<a.d0 <<")");

  if(a.nd==1) {
    referTo(a.p+i_lo, i_up+1-i_lo);
  }
  if(a.nd==2) {
    referTo(a.p+i_lo*a.d1, (i_up+1-i_lo)*a.d1);
    nd=2;  d0=i_up+1-i_lo;  d1=a.d1;
  }
  if(a.nd==3) {
    referTo(a.p+i_lo*a.d1*a.d2, (i_up+1-i_lo)*a.d1*a.d2);
    nd=3;  d0=i_up+1-i_lo;  d1=a.d1;  d2=a.d2;
  }
}

/// make this array a subarray reference to \c a
template<class T> void rai::Array<T>::referToRange(const Array<T>& a, int i, int j_lo, int j_up) {
  CHECK(a.nd>1, "does not make sense");
  CHECK_LE(a.nd, 3, "not implemented yet");
  if(i<0) i+=a.d0;
  if(j_lo<0) j_lo+=a.d1;
  if(j_up<0) j_up+=a.d1;
  if(j_lo>j_up) return;
  CHECK((uint)i<a.d0, "SubRange range error (" <<i <<"<" <<a.d0 <<")");
  CHECK((uint)j_lo<a.d1 && (uint)j_up<a.d1, "SubRange range error (" <<j_lo <<"<" <<a.d1 <<", " <<j_up <<"<" <<a.d1 <<")");

  if(a.nd==2) {
    referTo(&a(i, j_lo), (j_up+1-j_lo));
  }
  if(a.nd==3) {
    referTo(&a(i, j_lo, 0), (j_up+1-j_lo)*a.d2);
    nd=2;  d0=j_up+1-j_lo;  d1=a.d2;
  }
}

/// make this array a subarray reference to \c a
template<class T> void rai::Array<T>::referToRange(const Array<T>& a, int i, int j, int k_lo, int k_up) {
  CHECK(a.nd>2, "does not make sense");
  CHECK_LE(a.nd, 3, "not implemented yet");
  if(i<0) i+=a.d0;
  if(j<0) j+=a.d1;
  if(k_lo<0) k_lo+=a.d2;
  if(k_up<0) k_up+=a.d2;
  if(k_lo>k_up) return;
  CHECK((uint)i<a.d0, "SubRange range error (" <<i <<"<" <<a.d0 <<")");
  CHECK((uint)j<a.d1, "SubRange range error (" <<j <<"<" <<a.d1 <<")");
  CHECK((uint)k_lo<a.d2 && (uint)k_up<a.d2, "SubRange range error (" <<k_lo <<"<" <<a.d2 <<", " <<k_up <<"<" <<a.d2 <<")");

  if(a.nd==3) {
    referTo(&a(i, j, k_lo), k_up+1-k_lo);
  }
}

/// make this array a subarray reference to \c a
template<class T> void rai::Array<T>::referToDim(const rai::Array<T>& a, int i) {
  CHECK(a.nd>1, "can't create subarray of array less than 2 dimensions");
  CHECK(!isSparseMatrix(*this), "can't refer to row of sparse matrix");
  if(i<0) i+=a.d0;
  CHECK(i>=0 && i<(int)a.d0, "SubDim range error (" <<i <<"<" <<a.d0 <<")");

  if(a.nd==2) {
    referTo(a.p+i*a.d1, a.d1);
  }
  if(a.nd==3) {
    referTo(a.p+i*a.d1*a.d2, a.d1*a.d2);
    nd=2;  d0=a.d1;  d1=a.d2;
  }
  if(a.nd>3) {
    uint n=a.N/a.d0;
    referTo(a.p+i*n, n);
    nd=a.nd-1;  d0=a.d1;  d1=a.d2;  d2=a.d[3];
    if(nd>3) { d=new uint[nd];  memmove(d, a.d+1, nd*sizeof(uint)); }
  }
}

/// make this array a subarray reference to \c a
template<class T> void rai::Array<T>::referToDim(const rai::Array<T>& a, uint i, uint j) {
  CHECK(a.nd>2, "can't create subsubarray of array less than 3 dimensions");
  CHECK(i<a.d0 && j<a.d1, "SubDim range error (" <<i <<"<" <<a.d0 <<", " <<j <<"<" <<a.d1 <<")");

  if(a.nd==3) {
    referTo(&a(i, j, 0), a.d2);
  } else {
    NIY // TODO
  }
}

/// make this array a subarray reference to \c a
template<class T> void rai::Array<T>::referToDim(const rai::Array<T>& a, uint i, uint j, uint k) {
  CHECK(a.nd>3, "can't create subsubarray of array less than 3 dimensions");
  CHECK(i<a.d0 && j<a.d1 && k<a.d2, "SubDim range error (" <<i <<"<" <<a.d0 <<", " <<j <<"<" <<a.d1 <<", " <<k <<"<" <<a.d2 << ")");

  if(a.nd==4) {
    referTo(&a(i, j, k), a.d[3]);
  }
  if(a.nd==5) {
    NIY;
//    nd=2; d0=a.d[3]; d1=a.d[4]; d2=0; N=d0*d1;
  }
  if(a.nd>5) {
    NIY;
//    nd=a.nd-3; d0=a.d[3]; d1=a.d[4]; d2=a.d[5]; N=a.N/(a.d0*a.d1*a.d2);
//    resetD();
//    if(nd>3) { d=new uint[nd];  memmove(d, a.d+3, nd*sizeof(uint)); }
  }
//  p=a.p+(i*a.N+(j*a.N+(k*a.N/a.d2))/a.d1)/a.d0;
}

/** @brief takes over the memory buffer from a; afterwards, this is a
  proper array with own memory and a is only a reference on the
  memory */
template<class T> void rai::Array<T>::takeOver(rai::Array<T>& a) {
  freeMEM();
  memMove=a.memMove;
  N=a.N; nd=a.nd; d0=a.d0; d1=a.d1; d2=a.d2;
  p=a.p; M=a.M;
  a.isReference=true;
  a.M=0;
}

template<class T> void rai::Array<T>::swap(Array<T>& a) {
  CHECK(!isReference && !a.isReference, "NIY for references");
//  CHECK(!special&& !a.special, "NIY for specials");
  CHECK(nd<=3 && a.nd<=3, "only for 1D");
#ifdef RAI_USE_STDVEC
  std::swap((vec_type&)*this, (vec_type&)a);
#endif

#define SWAPx(X, Y){ auto z=X; X=Y; Y=z; }
  SWAPx(p, a.p);
//  SWAPx(special, a.special);
//  SWAPx(isReference, a.isReference);
//  T* p_tmp = p;
//  p=a.p;
//  a.p=p_tmp;
#undef SWAPx

  uint z;
#define SWAP(X, Y){ z=X; X=Y; Y=z; }
  SWAP(N, a.N);
  SWAP(M, a.M);
  SWAP(nd, a.nd);
  SWAP(d0, a.d0);
  SWAP(d1, a.d1);
  SWAP(d2, a.d2);
#undef SWAP

#ifdef RAI_USE_STDVEC
  CHECK_EQ(p, vec_type::data(), "");
#endif
}

/** @brief return a `dim'-dimensional grid with `steps' intervals
  filling the range [lo, hi] in each dimension. Note: returned array is
  `flat', rather than grid-shaped. */
template<class T> void
rai::Array<T>::setGrid(uint dim, T lo, T hi, uint steps) {
  uint i, j, k;
  if(dim==1) {
    resize(steps+1, 1);
    for(i=0; i<d0; i++) operator()(i, 0)=lo+(hi-lo)*i/steps;
    return;
  }
  if(dim==2) {
    resize(steps+1, steps+1, 2);
    for(i=0; i<d0; i++) for(j=0; j<d1; j++) {
        operator()(i, j, 0)=lo+(hi-lo)*i/steps;
        operator()(i, j, 1)=lo+(hi-lo)*j/steps;
      }
    reshape(d0*d1, 2);
    return;
  }
  if(dim==3) {
    resize(TUP(steps+1, steps+1, steps+1, 3));
    for(i=0; i<d0; i++) for(j=0; j<d1; j++) for(k=0; k<d2; k++) {
          elem(TUP(i, j, k, 0))=lo+(hi-lo)*i/steps;
          elem(TUP(i, j, k, 1))=lo+(hi-lo)*j/steps;
          elem(TUP(i, j, k, 2))=lo+(hi-lo)*k/steps;
        }
    reshape(d0*d1*d2, 3);
    return;
  }
  HALT("not implemented yet");
}

//----- sorting etc
template<class T> T rai::Array<T>::median_nonConst() {
  CHECK_GE(N, 1, "");
  std::nth_element(p, p+N/2, p+N);
  return *(p+N/2);
}

template<class T> T rai::Array<T>::nthElement_nonConst(uint n) {
  CHECK_GE(N, n+1, "");
  std::nth_element(p, p+n, p+N);
  return *(p+n);
}

/// sort this list
template<class T> rai::Array<T>& rai::Array<T>::sort(ElemCompare comp) {
  std::sort(p, p+N, comp);
  return *this;
}

/// check whether list is sorted
template<class T> bool rai::Array<T>::isSorted(ElemCompare comp) const {
  uint i;
  for(i=0; i<N-1; i++) {
    if(!comp(elem(i), elem(i+1))) return false;
  }
  return true;
}

/// fast find method in a sorted array, returns index where x would fit into array
template<class T> uint rai::Array<T>::rankInSorted(const T& x, ElemCompare comp, bool rankAfterIfEqual) const {
  if(!N) return 0;
  T* lo=p, *hi=p+N-1, *mi;
  if(!rankAfterIfEqual) {
    if(comp(x, *lo)) return 0;
    if(!comp(x, *hi)) return N;
  } else {
    if(comp(*hi, x)) return N;
    if(!comp(*lo, x)) return 0;
  }
  for(;;) {
    if(lo+1>=hi) return hi-p;
    mi=lo+(hi-lo)/2; //works (the minus operator on pointers gives #objects)
    if(comp(*mi, x)) {
      if(!rankAfterIfEqual && comp(x, *mi)) hi=mi; //x and mi are equal -> we crop from hi
      else lo=mi;
    } else {
      hi=mi;
    }
  }
  HALT("you shouldn't be here");
  return 0;
}

/// fast find method in a sorted array, returns index to element equal to x
template<class T> int rai::Array<T>::findValueInSorted(const T& x, ElemCompare comp) const {
  uint cand_pos = rankInSorted(x, comp);
  if(cand_pos == N) return -1;
  else if(elem(cand_pos) != x) return -1;
  else return cand_pos;
}

/// fast insert method in a sorted array, the array remains sorted
template<class T> uint rai::Array<T>::insertInSorted(const T& x, ElemCompare comp, bool insertAfterIfEqual) {
  uint cand_pos = rankInSorted(x, comp, insertAfterIfEqual);
  insert(cand_pos, x);
//  if(true){
//      for(uint i=0;i<N-1;i++) CHECK(comp(elem(i),elem(i+1)), "this is not sorted!");
//  }
  return cand_pos;
}

template<class T> uint rai::Array<T>::setAppendInSorted(const T& x, ElemCompare comp) {
  CHECK(memMove, "");
  uint cand_pos = rankInSorted(x, comp);
  if(cand_pos<N && elem(cand_pos)==x) return cand_pos;
  if(cand_pos>0 && elem(cand_pos-1)==x) return cand_pos-1;
  insert(cand_pos, x);
  return cand_pos;
}

/// fast remove method in a sorted array, the array remains sorted
template<class T> void rai::Array<T>::removeValueInSorted(const T& x, ElemCompare comp) {
  uint i=findValueInSorted(x, comp);
  CHECK_EQ(elem(i), x, "value not found");
  remove(i);
}

template<class T> rai::Array<T>& rai::Array<T>::removeDoublesInSorted() {
  for(int i=N-1; i>0; i--) if(elem(i)==elem(i-1)) remove(i);
  return *this;
}

//***** permutations

/// permute the elements \c i and \c j
template<class T> void rai::Array<T>::permute(uint i, uint j) { T x=p[i]; p[i]=p[j]; p[j]=x; }

/// permute the entries according to the given permutation
template<class T> void rai::Array<T>::permute(const rai::Array<uint>& permutation) {
  CHECK_LE(permutation.N, N, "array smaller than permutation (" <<N <<"<" <<permutation.N <<")");
  rai::Array<T> b=(*this);
  for(uint i=0; i<N; i++) elem(i)=b.elem(permutation(i));
}

/// permute the rows (operator[]) according to the given permutation
template<class T> void rai::Array<T>::permuteRows(const rai::Array<uint>& permutation) {
  CHECK_LE(permutation.N, d0, "array smaller than permutation ("<<N<<"<"<<permutation.N<<")");
  rai::Array<T> b=(*this);
  for(uint i=0; i<d0; i++) operator[](i)()=b[permutation(i)];
}

/// apply the given 'permutation' on 'this'
template<class T> void rai::Array<T>::permuteInv(const rai::Array<uint>& permutation) {
  CHECK_LE(permutation.N, N, "array smaller than permutation (" <<N <<"<" <<permutation.N <<")");
  rai::Array<T> b=(*this);
  for(uint i=0; i<N; i++) elem(permutation(i))=b.elem(i);
}

/// permute the rows (operator[]) according to the given permutation
template<class T> void rai::Array<T>::permuteRowsInv(const rai::Array<uint>& permutation) {
  CHECK_LE(permutation.N, d0, "array smaller than permutation ("<<N<<"<"<<permutation.N<<")");
  rai::Array<T> b=(*this);
  for(uint i=0; i<d0; i++) operator[](permutation(i))()=b[i];
}

/// randomly permute all entries of 'this'
template<class T> void rai::Array<T>::permuteRandomly() {
  uintA perm;
  perm.setRandomPerm(N);
  permute(perm);
}

/// push all elements forward or backward (depending on sign of offset)
template<class T> void rai::Array<T>::shift(int offset, bool wrapAround) {
  rai::Array<T> tmp;
  CHECK(memMove, "pushing only works with memMove enabled");
  uint m=offset>0?offset:-offset;
  if(!m) return;
  CHECK(m<N, "shift offset needs to be smaller than memory size");
  if(wrapAround) tmp.resize(m);
  if(offset>0) {
    if(wrapAround) memmove(tmp.p, p+N-m, sizeT*m);
    memmove(p+m, p, sizeT*(N-m));
    if(wrapAround) memmove(p, tmp.p, sizeT*m); else memset(p, 0, sizeT*m);
  }
  if(offset<0) {
    if(wrapAround) memmove(tmp.p, p, sizeT*m);
    memmove(p, p+m, sizeT*(N-m));
    if(wrapAround) memmove(p+(N-m), tmp.p, sizeT*m); else memset(p+(N-m), 0, sizeT*m);
  }
}

template<class T> void rai::Array<T>::setNoArr() {
  clear();
  special = new SpecialArray(SpecialArray::ST_NoArr);
}

#ifndef RAI_MSVC
template<typename T> struct is_shared_ptr : std::false_type {};
template<typename T> struct is_shared_ptr<std::shared_ptr<T>> : std::true_type {};

template <class T>
typename std::enable_if<is_shared_ptr<T>::value, std::ostream&>::type
operator<<(std::ostream& os, const rai::Array<T>& x) {
  os <<'{';
  for(uint i=0; i<x.N; i++) { if(x.elem(i)) os <<' ' <<*x.elem(i); else os <<" <NULL>"; }
  os <<" }" <<std::flush;
  //  x.write(os);
  return os;
}

template <class T>
typename std::enable_if<!is_shared_ptr<T>::value, std::ostream&>::type
operator<<(std::ostream& os, const rai::Array<T>& x) {
  x.write(os); return os;
}

#else //MSVC

template <class T> std::ostream& operator<<(std::ostream& os, const rai::Array<T>& x) {
  x.write(os); return os;
}

#endif

/** @brief prototype for operator<<, writes the array by separating elements with ELEMSEP, separating rows with LINESEP, using BRACKETS[0] and BRACKETS[1] to brace the data, optionally writs a dimensionality tag before the data (see below), and optinally in binary format */
template<class T> void rai::Array<T>::write(std::ostream& os, const char* ELEMSEP, const char* LINESEP, const char* BRACKETS, bool dimTag, bool binary) const {
  CHECK(!binary || memMove, "binary write works only for memMoveable data");
  uint i, j, k;
  if(!ELEMSEP) ELEMSEP=rai::arrayElemsep;
  if(!LINESEP) LINESEP=rai::arrayLinesep;
  if(!BRACKETS) BRACKETS=rai::arrayBrackets;

  if(binary) {
    writeDim(os);
    os <<std::endl;
    os.put(0);
    os.write((char*)p, sizeT*N);
    os.put(0);
    os <<std::endl;
  } else if(isSparseVector(*this)) {
    intA& elems = dynamic_cast<SparseVector*>(special)->elems;
    for(uint i=0; i<N; i++) os <<"( " <<elems(i) <<" ) " <<elem(i) <<endl;
  } else if(isSparseMatrix(*this)) {
    intA& elems = dynamic_cast<SparseMatrix*>(special)->elems;
    for(uint i=0; i<N; i++) os <<'(' <<elems[i] <<") " <<elem(i) <<endl;
  } else {
    if(BRACKETS[0]) os <<BRACKETS[0];
    if(dimTag || nd>3) { os <<' '; writeDim(os); if(nd==2) os <<'\n'; else os <<' '; }
    if(nd>=3) os <<'\n';
    if(nd==0 && N==1) {
      os <<(const T&)scalar();
    }
    if(nd==1) {
      for(i=0; i<N; i++) os <<(i?ELEMSEP:"")  <<operator()(i);
    }
    if(nd==2) for(j=0; j<d0; j++) {
        if(j) os <<LINESEP;
        if(isRowShifted(*this)) {
          RowShifted* rs = dynamic_cast<RowShifted*>(special);
          cout <<"[row-shift:" <<rs->rowShift(j) <<" row-len:" <<rs->rowLen(j) <<"] ";
          for(i=0; i<rs->rowSize; i++) os <<(i?ELEMSEP:"") <<rs->entry(j,i);
        }else{
          for(i=0; i<d1; i++) os <<(i?ELEMSEP:"") <<operator()(j, i);
        }
      }
    if(nd==3) for(k=0; k<d0; k++) {
        if(k) os <<'\n';
        for(j=0; j<d1; j++) {
          for(i=0; i<d2; i++) os <<(i?ELEMSEP:"") <<operator()(k, j, i);
          os <<LINESEP;
        }
      }
    if(nd>3) {
      CHECK(d && d!=&d0, "");
      for(i=0; i<N; i++) {
        if(i && !(i%d[nd-1])) os <<LINESEP;
        if(nd>1 && !(i%(d[nd-2]*d[nd-1]))) os <<LINESEP;
        os <<(i?ELEMSEP:"") <<elem(i);
      }
    }
    if(BRACKETS[1]) os <<BRACKETS[1];
  }
}

/** @brief prototype for operator>>, if there is a dimensionality tag: fast reading of ascii (if there is brackets[]) or binary (if there is \\0\\0 brackets) data; otherwise slow ascii read */
template<class T> void rai::Array<T>::read(std::istream& is) {
  uint d, i;
  char c;
  T x;
  bool expectBracket=false;

#define PARSERR(x) HALT("Error in parsing Array of type '" <<typeid(T).name() <<"' (line=" <<rai::lineCount <<"):\n" <<x)

  c=rai::peerNextChar(is);
  if(c=='[') {
    is >>PARSE("[");
    expectBracket=true;
    c=rai::peerNextChar(is);
  }

  if(c=='<') {
    readDim(is);
    c=rai::peerNextChar(is);
    if(c==0) {  //binary read
      c=is.get();  if(c!=0) PARSERR("couldn't read \0 before binary data block :-(");
      is.read((char*)p, sizeT*N);
      if(is.fail()) PARSERR("could not binary data");
      c=is.get();  if(c!=0) PARSERR("couldn't read \0 after binary data block :-(");
    } else { //fast ascii read
      for(i=0; i<N; i++) {
        if(is.fail()) PARSERR("could not read " <<i <<"-th element of an array");
        is >>p[i];
      }
    }
    if(expectBracket) {
      is >>PARSE("]");
      if(is.fail()) PARSERR("could not read array end tag");
    }
  } else { //slow ascii read (inferring size from formatting)
    uint i=0;
    d=0;
    for(;;) {
      rai::skip(is, " ,\r\t", NULL, true);
      is.get(c);
      if(c==']' || !is.good()) { is.clear(); break; }
      if(c==';' || c=='\n') {  //set an array width
        if(!d) d=i; else if(i%d) PARSERR("mis-structured array in row " <<i/d);
        continue;
      }
      if(c!=',') is.putback(c);
      is >>x;
      if(!is.good()) { is.clear(); break; }
      if(i>=N) resizeCopy(i+1000);
      elem(i)=x;
      i++;
    }
    resizeCopy(i);
    if(d) {
      if(N%d) PARSERR("mis-structured array in last row");
      reshape(N/d, d);
    }
  }

#undef PARSERR

}

/// write a dimensionality tag of format <d0 d1 d2 ...>
template<class T> void rai::Array<T>::writeDim(std::ostream& os) const {
  uint i;
  os <<'<';
  if(nd) os <<dim(0);
  for(i=1; i<nd; i++) os <<' ' <<dim(i);
  os <<'>';
}

/// read a dimensionality tag of format <d0 d1 d2 ...> and resize this array accordingly
template<class T> void rai::Array<T>::readDim(std::istream& is) {
  char c;
  uint ND, dim[10];
  is >>PARSE("<");
  is.get(c);
  if(c=='>'){
    clear();
    return;
  }else{
    is.putback(c);
  }
  for(ND=0;; ND++) {
    is >>dim[ND];
    is.get(c);
    if(c=='>') break;
    CHECK_EQ(c, ' ', "error in reading dimensionality");
  }
  resize(ND+1, dim);
}

/// OBSOLETE - I should remove this. IO-manip: arrays will be streamed as raw (without tags)
template<class T> const rai::Array<T>& rai::Array<T>::ioraw() const { IOraw=true; return *this; }

/// IO-manip: arrays will be streamed non-raw (with tags)
//template<class T> const rai::Array<T>& rai::Array<T>::ionoraw() const{ IOraw=false; return *this; }

/// array must have correct size! simply streams in all elements sequentially
template<class T> void rai::Array<T>::readRaw(std::istream& is) {
  uint i;
  if(N) {
    for(i=0; i<N; i++) {
      is >>p[i];
      if(is.fail()) HALT("could not read " <<i <<"-th element of an array");
    }
  } else {
    T x;
    for(;;) {
      is >>x;
      if(!is.good()) { is.clear(); return; }
      append(x);
    }
  }
}

//----- macros for piping doubles EXACTLY (without rounding errors) in hex coding:
//#define OUTHEX(y) "0x" <<std::hex <<*((unsigned long*)&y) <<std::dec
//#define INHEX(y)  std::hex >>*((unsigned long*)&y) >>std::dec

/// same as write(os, " ", "\n", "  ");
template<class T> void rai::Array<T>::writeRaw(std::ostream& os) const {
  write(os, " ", "\n", "  ");
}

/// write data with a name tag (convenient to write multiple data arrays into one file)
template<class T> void rai::Array<T>::writeTagged(std::ostream& os, const char* tag, bool binary) const {
  os <<tag <<": ";
  write(os, " ", "\n ", "[]", true, binary);
}

/// read data with a name tag (convenient to read multiple data arrays from one file)
template<class T> bool rai::Array<T>::readTagged(std::istream& is, const char* tag) {
  if(tag) {
    String read_tag;
    read_tag.read(is, " \t\n\r", ": \t\n\r");
    if(!is.good() || read_tag.N==0) return false;
    CHECK_EQ(read_tag, tag, "read `" <<read_tag <<"' instead of `" <<tag <<"' in arr file");
    rai::skip(is, ": \t\n\r");
  };
  read(is);
  return true;
}

/// write robustly in file
template<class T> void rai::Array<T>::writeTagged(const char* filename, const char* tag, bool binary) const {
  ofstream fil;
  rai::open(fil, filename);
  return writeTagged(fil, tag, binary);
}

/// read robustly from file
template<class T> bool rai::Array<T>::readTagged(const char* filename, const char* tag) {
  ifstream fil;
  rai::open(fil, filename);
  return readTagged(fil, tag);
}

template<class T> uint rai::Array<T>::serial_size() {
  return 6+6*sizeof(uint)+N*sizeT;
}

template<class T> uint rai::Array<T>::serial_encode(char* data, uint data_size) {
  CHECK_GE(data_size, serial_size(), "buffer doesn't have right size!");
  uint intSize = sizeof(uint);
  uint typeSize = sizeof(T);
  memcpy(data, "ARRAY", 6);
  memcpy(data+6+0*intSize, &typeSize, intSize);
  memcpy(data+6+1*intSize, &N, intSize);
  memcpy(data+6+2*intSize, &nd, intSize);
  memcpy(data+6+3*intSize, &d0, intSize);
  memcpy(data+6+4*intSize, &d1, intSize);
  memcpy(data+6+5*intSize, &d2, intSize);
  memcpy(data+6+6*intSize, p, N*typeSize);
  return serial_size();
}

template<class T> uint rai::Array<T>::serial_decode(char* data, uint data_size) {
  CHECK_GE(data_size, 6+6*sizeof(uint), "");
  CHECK(!memcmp(data, "ARRAY", 6), "");
  uint typeSize, n;
  uint intSize = sizeof(uint);
  memcpy(&typeSize, data+6+0*intSize,  intSize);
  memcpy(&n,  data+6+1*intSize,  intSize);
  memcpy(&nd, data+6+2*intSize, intSize);
  memcpy(&d0, data+6+3*intSize, intSize);
  memcpy(&d1, data+6+4*intSize, intSize);
  memcpy(&d2, data+6+5*intSize, intSize);
  CHECK_EQ(typeSize, (uint)sizeT, "");
  CHECK_GE(data_size, 6+6*sizeof(uint)+n*sizeT, "buffer doesn't have right size!");
  if(nd==1) CHECK_EQ(n, d0, "");
  if(nd==2) CHECK_EQ(n, d0*d1, "");
  if(nd==3) CHECK_EQ(n, d0*d1*d2, "");
  resizeMEM(n, false);
  memcpy(p, data+6+6*intSize, N*typeSize);
  return serial_size();
}

/// gdb pretty printing
template<class T> const char* rai::Array<T>::prt() {
  static rai::String tmp;
  tmp.clear();
  tmp <<"GDB Array<" <<typeid(T).name() <<"> dim=";
  writeDim(tmp);
  write(tmp);
  tmp <<endl;
  return tmp.p;
}

/// x = y^T
template<class T> void transpose(rai::Array<T>& x, const rai::Array<T>& y) {
  CHECK(&x!=&y, "can't transpose matrix into itself");
  CHECK_LE(y.nd, 3, "can only transpose up to 3D arrays");
  if(y.nd==3) {
    uint i, j, k, d0=y.d2, d1=y.d1, d2=y.d0;
    x.resize(d0, d1, d2);
    for(i=0; i<d0; i++) for(j=0; j<d1; j++) for(k=0; k<d2; k++)
          x(i, j, k) = y(k, j, i);
    //x.p[(i*d1+j)*d2+k]=y.p[(k*d1+j)*d0+i];
    return;
  }
  if(y.nd==2) {
    if(isSparseMatrix(y)) {
      x = y;
      x.sparse().transpose();
      return;
    }
    x.resize(y.d1, y.d0);
//    for(i=0; i<d0; i++)
    T* xp=x.p;
    for(uint i=0; i<x.d0; i++) {
      T* yp=y.p+i, *xstop=xp+x.d1;
      uint ystep=y.d1;
      for(; xp!=xstop; xp++, yp+=ystep) *xp = *yp;
//      for(j=0; j<d1; j++) x.p[i*d1+j]=y.p[j*d0+i];
    }
    return;
  }
  if(y.nd==1) {
    x=y;
    x.reshape(1, y.N);
    return;
  }
  HALT("transpose not implemented for this dims");
}

/// returns the diagonal x = diag(y) (the diagonal-vector of the symmetric 2D matrix y)
template<class T> rai::Array<T> getDiag(const rai::Array<T>& y) {
  CHECK(y.nd==2 && y.d0==y.d1, "can only give diagonal of symmetric 2D matrix");
  arr x;
  x.resize(y.d0);
  uint i;
  for(i=0; i<x.d0; i++) x(i)=y(i, i);
  return x;
}

/// inverse of a 2d matrix
template<class T> void inverse2d(rai::Array<T>& Ainv, const rai::Array<T>& A) {
  Ainv.resize(2, 2);
  Ainv(0, 0)=A(1, 1); Ainv(1, 1)=A(0, 0); Ainv(0, 1)=-A(0, 1); Ainv(1, 0)=-A(1, 0);
  Ainv/=(T)(A(0, 0)*A(1, 1)-A(0, 1)*A(1, 0));
}

/// sets x to be the diagonal matrix with diagonal v
template<class T> rai::Array<T> skew(const rai::Array<T>& v) {
  rai::Array<T> y;
  CHECK(v.nd==1 && v.N==3, "can only give diagonal of 1D array");
  y.resize(3, 3);
  y.p[0]=   0.; y.p[1]=-v(2); y.p[2]= v(1);
  y.p[3]= v(2); y.p[4]=   0.; y.p[5]=-v(0);
  y.p[6]=-v(1); y.p[7]= v(0); y.p[8]=   0.;
  return y;
}

/// check for Nans in the array (checks x.elem(i)==x.elem(i) for all elements)
template<class T> void checkNan(const rai::Array<T>& x) {
  for(uint i=0; i<x.N; i++) {
    //CHECK(x.elem(i)!=NAN, "found a NaN" <<x.elem(i) <<'[' <<i <<']');
    CHECK_EQ(x.elem(i), x.elem(i), "inconsistent number: " <<x.elem(i) <<'[' <<i <<']');
  }
}

template<class T> void sort(rai::Array<T>& x) {
  T* pstop=x.p+x.N;
  std::sort(x.p, pstop);
}

template<class T> rai::Array<T> replicate(const rai::Array<T>& A, uint d0) {
  arr x;
  uintA d=A.dim();
  d.prepend(d0);
  x.resize(d);
  if(x.memMove) {
    for(uint i=0; i<x.d0; i++) memmove(&x.elem(i*A.N), A.p, A.sizeT*A.N);
  } else {
    for(uint i=0; i<x.d0; i++) x[i]=A;
  }
  return x;
}

/// return the integral image, or vector
template<class T> rai::Array<T> integral(const rai::Array<T>& x) {
  CHECK(x.nd==1 || x.nd==2, "");
  if(x.nd==1) {
    T s(0);
    rai::Array<T> y(x.N);
    for(uint i=0; i<x.N; i++) { s+=x.elem(i); y.elem(i)=s; }
    return y;
  }
  if(x.nd==2) {
    NIY;
  }
  return rai::Array<T>();
}

#ifdef RAI_CLANG
#  pragma clang diagnostic pop
#endif

//===========================================================================
//
/// @name probability distribution operations
//

/** @brief entropy \f$H_i = - \sum_x p(X_i=x) \ln p(X_i=x)\f$, where \f$x\f$ ranges
  from \c 0 to \c range-1, of the \c ith variable */
template<class T> T entropy(const rai::Array<T>& v) {
  T t(0);
  for(uint i=v.N; i--;) if(v.p[i]) t-=(T)(v.p[i]*::log((double)v.p[i]));
  return (T)(t/RAI_LN2);
}

/// v = v / sum(v)
template<class T> T normalizeDist(rai::Array<T>& v) {
  T Z=sum(v);
  if(Z>1e-100) v/=Z; else v=(T)1./(T)v.N;
  return Z;
}

/// v = v / sum(v)
template<class T> void makeConditional(rai::Array<T>& P) {
  RAI_MSG("makeConditional: don't use this anymore because it normalizes over the second index!!!, rather use tensorCondNormalize and condition on _later_ indices");
  CHECK_EQ(P.nd, 2, "");
  uint i, j;
  T pi;
  for(i=0; i<P.d0; i++) {
    pi=(T)0;
    for(j=0; j<P.d1; j++) pi+=P(i, j);
    for(j=0; j<P.d1; j++) P(i, j) /= pi;
  }
}

//inline uintA TUP(uint i, uint j, uint k, uint l){                      uintA z(4); z(0)=i; z(1)=j; z(2)=k; z(3)=l; return z; }

/// check whether this is a distribution over the first index w.r.t. the later indices
template<class T> void checkNormalization(rai::Array<T>& v, double tol) {
  double p;
  uint i, j, k, l;
  switch(v.nd) {
    case 1:
      for(p=0, i=0; i<v.d0; i++) p+=v(i);
      CHECK(fabs(1.-p)<tol, "distribution is not normalized: " <<v);
      break;
    case 2:
      for(j=0; j<v.d1; j++) {
        for(p=0, i=0; i<v.d0; i++) p+=v(i, j);
        CHECK(fabs(1.-p)<tol, "distribution is not normalized: " <<v);
      }
      break;
    case 3:
      for(j=0; j<v.d1; j++) for(k=0; k<v.d2; k++) {
          for(p=0, i=0; i<v.d0; i++) p+=v(i, j, k);
          CHECK(fabs(1.-p)<tol, "distribution is not normalized: " <<v);
        }
      break;
    case 4:
      for(j=0; j<v.d1; j++) for(k=0; k<v.d2; k++) {
          for(l=0; l<v.N/(v.d0*v.d1*v.d2); l++) {
            for(p=0, i=0; i<v.d0; i++) p+=v.elem(TUP(i, j, k, l));
            CHECK(fabs(1.-p)<tol, "distribution is not normalized: " <<v <<" " <<p);
          }
        }
      break;
    default:
      CHECK(false, "Not implemented yet");
      break;
  }
}

template<class T> void eliminate(rai::Array<T>& x, const rai::Array<T>& y, uint d) {
  CHECK_EQ(y.nd, 2, "only implemented for 2D yet");
  uint i, j;
  if(d==1) {
    x.resize(y.d0); x=(T)0;
    for(i=0; i<y.d0; i++) for(j=0; j<y.d1; j++) x(i)+=y(i, j);
  }
  if(d==0) {
    x.resize(y.d1); x=(T)0;
    for(i=0; i<y.d0; i++) for(j=0; j<y.d1; j++) x(j)+=y(i, j);
  }
}

template<class T> void eliminate(rai::Array<T>& x, const rai::Array<T>& y, uint d, uint e) {
  CHECK_EQ(y.nd, 3, "only implemented for 3D yet");
  uint i, j, k;
  if(d==1 && e==2) {
    x.resize(y.d0); x=(T)0;
    for(i=0; i<y.d0; i++) for(j=0; j<y.d1; j++) for(k=0; k<y.d2; k++) x(i)+=y(i, j, k);
  }
  if(d==0 && e==2) {
    x.resize(y.d1); x=(T)0;
    for(i=0; i<y.d0; i++) for(j=0; j<y.d1; j++) for(k=0; k<y.d2; k++) x(j)+=y(i, j, k);
  }
  if(d==0 && e==1) {
    x.resize(y.d2); x=(T)0;
    for(i=0; i<y.d0; i++) for(j=0; j<y.d1; j++) for(k=0; k<y.d2; k++) x(k)+=y(i, j, k);
  }
}

// Eliminates one-dimension, d, from a 3D-tensor, y, and puts the result in x.
template<class T> void eliminatePartial(rai::Array<T>& x, const rai::Array<T>& y, uint d) {
  CHECK_EQ(y.nd, 3, "only implemented for 3D yet");
  uint i, j, k;
  if(d==2) {
    x.resize(y.d0, y.d1); x=(T)0;
    for(i=0; i<y.d0; i++) for(j=0; j<y.d1; j++) for(k=0; k<y.d2; k++) x(i, j)+=y(i, j, k);
  }
  if(d==1) {
    x.resize(y.d0, y.d2); x=(T)0;
    for(i=0; i<y.d0; i++) for(j=0; j<y.d1; j++) for(k=0; k<y.d2; k++) x(i, k)+=y(i, j, k);
  }
  if(d==0) {
    x.resize(y.d1, y.d2); x=(T)0;
    for(i=0; i<y.d0; i++) for(j=0; j<y.d1; j++) for(k=0; k<y.d2; k++) x(j, k)+=y(i, j, k);
  }
}

//===========================================================================
//
/// @name distances
//

/// \f$\sum_i (v^i-w^i)^2\f$
template<class T>
T sqrDistance(const rai::Array<T>& v, const rai::Array<T>& w) {
  CHECK_EQ(v.N, w.N,
           "sqrDistance on different array dimensions (" <<v.N <<", " <<w.N <<")");
  T d, t(0);
  for(uint i=v.N; i--;) { d=v.p[i]-w.p[i]; t+=d*d; }
  return t;
}

template<class T> T maxDiff(const rai::Array<T>& v, const rai::Array<T>& w, uint* im) {
  CHECK_EQ(v.N, w.N,
           "maxDiff on different array dimensions (" <<v.N <<", " <<w.N <<")");
  T d(0), t(0);
  if(!im)
    for(uint i=v.N; i--;) {
      d=(T)::fabs((double)(v.p[i]-w.p[i]));
      if(d>t) t=d;
    } else {
    *im=0;
    for(uint i=v.N; i--;) { d=(T)::fabs((double)(v.p[i]-w.p[i])); if(d>t) { t=d; *im=i; } }
  }
  return t;
}

template<class T> T maxRelDiff(const rai::Array<T>& v, const rai::Array<T>& w, T tol) {
  CHECK_EQ(v.N, w.N,
           "maxDiff on different array dimensions (" <<v.N <<", " <<w.N <<")");
  T d, t(0), a, b, c;
  for(uint i=v.N; i--;) {
    a=(T)::fabs((double)v.p[i]) + tol;
    b=(T)::fabs((double)w.p[i]) + tol;
    if(a<b) { c=a; a=b; b=c; }
    d=a/b-(T)1;
    if(d>t) t=d;
  }
  return t;
}

/// \f$\sum_{i|{\rm mask}_i={\rm true}} (v^i-w^i)^2\f$
/*template<class T>
  T sqrDistance(const rai::Array<T>& v, const rai::Array<T>& w, const rai::Array<bool>& mask){
  CHECK_EQ(v.N,w.N,
  "sqrDistance on different array dimensions (" <<v.N <<", " <<w.N <<")");
  T d, t(0);
  for(uint i=v.N;i--;) if(mask(i)){ d=v.p[i]-w.p[i]; t+=d*d; }
  return t;
  }*/

/// \f$\sqrt{\sum_{ij} g_{ij} (v^i-w^i) (v^j-w^j)}\f$
template<class T> T sqrDistance(const rai::Array<T>& g, const rai::Array<T>& v, const rai::Array<T>& w) {
  rai::Array<T> d(v);
  d -= w;
  return scalarProduct(g, d, d);
}

/// \f$\sqrt{\sum_i (v^i-w^i)^2}\f$
template<class T>
T euclideanDistance(const rai::Array<T>& v, const rai::Array<T>& w) {
  return (T)::sqrt((double)sqrDistance(v, w));
}

/// \f$\sqrt{\sum_i (v^i-w^i)^2}\f$
template<class T>
T metricDistance(const rai::Array<T>& g, const rai::Array<T>& v, const rai::Array<T>& w) {
  return (T)::sqrt((double)sqrDistance(g, v, w));
}

//===========================================================================
//
/// @name running sums
//

/// \f$\sum_i x_i\f$
template<class T> T sum(const rai::Array<T>& v) {
  T t(0);
  for(uint i=v.N; i--; t+=v.p[i]) {};
  return t;
}

/// \f$\max_i x_i\f$
template<class T> T max(const rai::Array<T>& v) {
  CHECK(v.N, "");
  T m(v.p[0]);
  for(uint i=v.N; i--;) if(v.p[i]>m) m=v.p[i];
  return m;
}

/// \f$\min_i x_i\f$
template<class T> T min(const rai::Array<T>& v) {
  CHECK(v.N, "");
  T m(v.p[0]);
  for(uint i=v.N; i--;) if(v.p[i]<m) m=v.p[i];
  return m;
}

template<class T> T scalar(const rai::Array<T>& x) {
  return x.scalar();
}

/// \f$\sum_i x_i\f$
template<class T> rai::Array<T> sum(const rai::Array<T>& v, uint d) {
  CHECK(v.nd>d, "array doesn't have this dimension");
  rai::Array<T> x;
  x.referTo(v);
  rai::Array<T> S;
  uint i, j, k;
  if(d==v.nd-1) {  //sum over last index - contiguous in memory
    x.reshape(x.N/x.dim(x.nd-1), x.dim(x.nd-1));
    S.resize(x.d0);  S.setZero();
    for(i=0; i<x.d0; i++) for(j=0; j<x.d1; j++) S(i) += x(i, j);
    return S;
  }
  if(d==0) {  //sum over first index
    x.reshape(x.d0, x.N/x.d0);
    S.resize(x.d1);  S.setZero();
    for(i=0; i<x.d0; i++) for(j=0; j<x.d1; j++) S(j) += x(i, j);
    if(v.nd>2) S.reshape(v.dim().sub(1, -1));
    return S;
  }
  //any other index (includes the previous cases, but marginally slower)
  uintA IV, IS, dimV, dimS;
  dimV = v.dim();
  dimS.resize(dimV.N-1);
  for(i = 0, j = 0; i < dimS.N; i++, j++) {
    if(i == d) j++;
    dimS(i) = dimV(j);
  }
  x.referTo(v);
  x.reshape(x.N);
  S.resize(dimS); S.setZero();
  IS.resize(dimS.N);
  for(k = 0; k < x.N; k++) {
    v.getIndexTuple(IV, k);
    for(i = 0, j = 0; i < IS.N; i++, j++) {
      if(i == d) j++;
      IS(i) = IV(j);
    }
    S.elem(IS) += x(k);
  }
//  S.reshape(S.N); //(mt: Hä? Hab ich das gemacht???)
  return S;
}

/// \f$\max_i x_i\f$
template<class T> rai::Array<T> max(const rai::Array<T>& v, uint d) {
  CHECK(d<v.nd, "array doesn't have this dimension");
  rai::Array<T> x;
  x.referTo(v);
  rai::Array<T> M;
  uint i, j;
  if(d==v.nd-1) {  //max over last index - contiguous in memory
    x.reshape(x.N/x.dim(x.nd-1), x.dim(x.nd-1));
    M.resize(x.d0);
    for(i=0; i<x.d0; i++) M(i) = max(x[i]);
    return M;
  }
  if(d==0) {  //max over first index
    x.reshape(x.d0, x.N/x.d0);
    M = x[0]; //first row
    for(i=1; i<x.d0; i++) for(j=0; j<x.d1; j++)
        if(x(i, j)>M(j)) M(j)=x(i, j);
    return M;
  }
  NIY;
}

/// \f$\max_i x_i\f$
template<class T> rai::Array<T> min(const rai::Array<T>& v, uint d) {
  CHECK(v.nd>d, "array doesn't have this dimension");
  rai::Array<T> x;
  x.referTo(v);
  rai::Array<T> M;
  uint i, j;
  if(d==v.nd-1) {  //max over last index - contiguous in memory
    x.reshape(x.N/x.dim(x.nd-1), x.dim(x.nd-1));
    M.resize(x.d0);
    for(i=0; i<x.d0; i++) M(i) = min(x[i]);
    return M;
  }
  if(d==0) {  //sum over first index
    x.reshape(x.d0, x.N/x.d0);
    M = x[0]; //first row
    for(i=1; i<x.d0; i++) for(j=0; j<x.d1; j++)
        if(x(i, j)<M(j)) M(j)=x(i, j);
    return M;
  }
  NIY;
}

/// \f$\sum_i |x_i|\f$
template<class T> T sumOfAbs(const rai::Array<T>& v) {
  T t(0);
  for(uint i=v.N; i--; t+=(T)::fabs((double)v.p[i])) {};
  return t;
}

/// \f$\sum_i x_i^2\f$
template<class T> T sumOfSqr(const rai::Array<T>& v) {
  T t(0);
  for(uint i=v.N; i--; t+=v.p[i]*v.p[i]) {};
  return t;
}

/// \f$\sqrt{\sum_i x_i^2}\f$
template<class T> T length(const rai::Array<T>& x) { return (T)::sqrt((double)sumOfSqr(x)); }

template<class T> T var(const rai::Array<T>& x) { T m=sum(x)/x.N; return sumOfSqr(x)/x.N-m*m; }

template<class T> rai::Array<T> mean(const rai::Array<T>& X) { CHECK_EQ(X.nd, 2, ""); return sum(X, 0)/T(X.d0); }

template<class T> arr covar(const rai::Array<T>& X) { arr m=mean(X); return ((~X)*X)/T(X.d0)-m*~m; }

template<class T> rai::Array<T> stdDev(const rai::Array<T>& v) {
  CHECK(v.d0 > 1, "empirical standard deviation makes sense only for N>1")
  rai::Array<T> m = sum(v, 0);
  rai::Array<T> vX;
  vX.referTo(v);
  vX.reshape(vX.d0, vX.N/vX.d0);
  rai::Array<T> x = zeros(vX.d1);
  for(uint i = 0; i < v.d0; i++) {
    for(uint j = 0; j < vX.d1; j++) {
      x(j) += rai::sqr(vX(i, j)-m(j)/vX.d0)/(vX.d0-1);
    }
  }
  x = sqrt(x);
  return x;
}

/// \f$\sum_i x_{ii}\f$
template<class T> T trace(const rai::Array<T>& v) {
  CHECK(v.nd==2 && v.d0==v.d1, "only for squared matrix");
  T t(0);
  for(uint i=0; i<v.d0; i++) t+=v(i, i);
  return t;
}

template<class T> T minDiag(const rai::Array<T>& v) {
  CHECK(v.nd==2 && v.d0==v.d1, "only for squared matrix");
  T t=v(0, 0);
  for(uint i=1; i<v.d0; i++) if(v(i, i)<t) t=v(i, i);
  return t;
}

/// get absolute min (using fabs)
template<class T> T absMin(const rai::Array<T>& x) {
  CHECK(x.N, "");
  uint i;
  T t((T)::fabs((double)x.p[0]));
  for(i=1; i<x.N; i++) if(fabs((double)x.p[i])<t) t=(T)::fabs((double)x.p[i]);
  return t;
}

/// get absolute maximum (using fabs)
template<class T> T absMax(const rai::Array<T>& x) {
  if(!x.N) return (T)0;
  uint i;
  T t((T)::fabs((double)x.p[0]));
  for(i=1; i<x.N; i++) if(fabs((double)x.p[i])>t) t=(T)::fabs((double)x.p[i]);
  return t;
}

/// get absolute min (using fabs)
template<class T> void clip(const rai::Array<T>& x, T lo, T hi) {
  for(uint i=0; i<x.N; i++) ::clip(x.p[i], lo, hi);
}

//===========================================================================
//
/// @name products
//

/// \f$\prod_i x_i\f$
template<class T> T product(const rai::Array<T>& v) {
  T t(1);
  for(uint i=v.N; i--; t *= v.p[i]);
  return t;
}

/** @brief inner product (also ordinary matrix or scalar product):
  \f$\forall_{ik}:~ x_{ik} = \sum_j v_{ij}\, w_{jk}\f$ but also:
  \f$\forall_{i}:~ x_{i} = \sum_j v_{ij}\, w_{j}\f$*/
template<class T>
void innerProduct(rai::Array<T>& x, const rai::Array<T>& y, const rai::Array<T>& z) {
  if(!y || !z){ x.setNoArr(); return; }
  /*
    if(y.nd==2 && z.nd==2 && y.N==z.N && y.d1==1 && z.d1==1){  //elem-wise
    HALT("make element-wise multiplication explicite!");
    mult(x, y, z);
    return;
    }
  */
  if(y.nd==2 && z.nd==1) {  //matrix x vector -> vector
    CHECK_EQ(y.d1, z.d0, "wrong dimensions for inner product");
    if(rai::useLapack && typeid(T)==typeid(double)) { blas_Mv(x, y, z); return; }
    uint i, d0=y.d0, dk=y.d1;
    T* a, *astop, *b, *c;
    x.resize(d0); x.setZero();
    c=x.p;
    for(i=0; i<d0; i++) {
      //for(s=0., k=0;k<dk;k++) s+=y.p[i*dk+k]*z.p[k];
      //this is faster:
      a=y.p+i*dk; astop=a+dk; b=z.p;
      for(; a!=astop; a++, b++)(*c)+=(*a) * (*b);
      c++;
    }
    return;
  }
  if(y.nd==2 && z.nd==2) {  //plain matrix multiplication
    CHECK_EQ(y.d1, z.d0, "wrong dimensions for inner product");
    uint i, j, d0=y.d0, d1=z.d1, dk=y.d1;
#if 0
    if(y.mtype==rai::Array<T>::diagMT) {
      x.resize(d0, d1);
      for(i=0; i<d0; i++) for(j=0; j<d1; j++) x(i, j) = y(i, i) * z(i, j);
      return;
    }
    if(z.mtype==rai::Array<T>::diagMT) {
      x.resize(d0, d1);
      for(i=0; i<d0; i++) for(j=0; j<d1; j++) x(i, j) *= y(i, j) * z(j, j);
      return;
    }
#endif
    if(typeid(T)==typeid(double)) {
      if(isSparseMatrix(y)) { x = y.sparse().A_B(z); return; }
      if(isSparseMatrix(z)) { x = z.sparse().B_A(y); return; }
      if(isRowShifted(y)) { x = y.rowShifted().A_B(z); return; }
      if(isRowShifted(z)) { x = z.rowShifted().B_A(y); return; }
      if(rai::useLapack){ blas_MM(x, y, z); return; }
    }
    T* a, *astop, *b, *c;
    x.resize(d0, d1); x.setZero();
    c=x.p;
    for(i=0; i<d0; i++) for(j=0; j<d1; j++) {
        //for(s=0., k=0;k<dk;k++) s+=y.p[i*dk+k]*z.p[k*d1+j];
        //this is faster:
        a=y.p+i*dk; astop=a+dk; b=z.p+j;
        for(; a!=astop; a++, b+=d1)(*c)+=(*a) * (*b);
        c++;
      }
    return;
  }
  if(y.nd==1 && z.nd==1 && z.N==1) {  //vector multiplied with scalar (disguised as 1D vector)
    uint k, dk=y.N;
    x.resize(y.N);
    for(k=0; k<dk; k++) x.p[k]=y.p[k]*z.p[0];
    return;
  }
  if(y.nd==1 && z.nd==2 && z.d0==1) {  //vector x vector^T -> matrix (outer product)
    if(typeid(T)==typeid(double)) {
      arr _y;
      _y.referTo(y);
      _y.reshape(y.N, 1);
      if(z.isSparse()) { x = z.sparse().B_A(_y); return; }
    }
    uint i, j, d0=y.d0, d1=z.d1;
    x.resize(d0, d1);
    for(i=0; i<d0; i++) for(j=0; j<d1; j++) x(i, j)=y(i)*z(0, j);
    return;
  }
  if(y.nd==1 && z.nd==2) {  //vector^T x matrix -> vector^T
    HALT("This is a stupid old inconsistent case of matrix multiplication! Never use this convention! Change your code: transpose first term explicitly.");
    /*
    rai::Array<T> zt;
    transpose(zt, z);
    x = zt*y;
    */
  }
  if(y.nd==2 && z.nd==3) {
    rai::Array<T> zz; zz.referTo(z);
    zz.reshape(z.d0, z.d1*z.d2);
    innerProduct(x, y, zz);
    x.reshape(y.d0, z.d1, z.d2);
    return;
  }
  if(y.nd==3 && z.nd==2) {
    rai::Array<T> yy; yy.referTo(y);
    yy.reshape(y.d0*y.d1, y.d2);
    innerProduct(x, yy, z);
    x.reshape(y.d0, y.d1, z.d1);
    return;
  }
  if(y.nd==3 && z.nd==1) {
    rai::Array<T> yy; yy.referTo(y);
    yy.reshape(y.d0*y.d1, y.d2);
    innerProduct(x, yy, z);
    x.reshape(y.d0, y.d1);
    return;
  }
  if(y.nd==1 && z.nd==3) {
    rai::Array<T> zz; zz.referTo(z);
    zz.reshape(z.d0, z.d1*z.d2);
    innerProduct(x, y, zz);
    x.reshape(z.d1, z.d2);
    return;
  }
  if(y.nd==1 && z.nd==1) {  //should be scalar product, but be careful
    HALT("what do you want? scalar product or element wise multiplication?");
    CHECK_EQ(y.d0, z.d0, "wrong dimensions for inner product");
    uint k, dk=y.d0;
    x.resize(1);
    T s;
    for(s=0, k=0; k<dk; k++) s+=y.p[k]*z.p[k];
    x.p[0]=s;
    return;
  }
  HALT("inner product - not yet implemented for these dimensions: " <<y.nd <<" " <<z.nd);
}

/** @brief outer product (also exterior or tensor product): \f$\forall_{ijk}:~
  x_{ijk} = v_{ij}\, w_{k}\f$ */
template<class T>
void outerProduct(rai::Array<T>& x, const rai::Array<T>& y, const rai::Array<T>& z) {
  if(y.nd==1 && z.nd==1) {
#if 1
    uint i, j, d0=y.d0, d1=z.d0;
    x.resize(d0, d1);
    for(i=0; i<d0; i++) for(j=0; j<d1; j++) x.p[i*d1+j] = y.p[i] * z.p[j];
#else
    x.resize(y.N, z.N);
    T yi, *zp=z.p, *zstop=zp+z.N, *xp;
    for(uint i=0; i<y.N; i++) {
      yi=y.p[i];
      xp=&x(i, 0);
      zp=z.p;
      for(; zp!=zstop; zp++, xp++) *xp = yi * *zp;
    }
#endif
    return;
  }
  if(y.nd==2 && z.nd==1) {
    uint i, j, k, d0=y.d0, d1=y.d1, d2=z.d0;
    x.resize(d0, d1, d2);
    for(i=0; i<d0; i++) for(j=0; j<d1; j++) for(k=0; k<d2; k++) x.p[(i*d1+j)*d2+k] = y.p[i*d1+j] * z.p[k];
    return;
  }
  HALT("outer product - not yet implemented for these dimensions");
}

/** @brief element wise product */
template<class T>
void elemWiseProduct(rai::Array<T>& x, const rai::Array<T>& y, const rai::Array<T>& z) {
  CHECK_EQ(y.N, z.N, "");
  x = z;
  for(uint i=0; i<x.N; i++) x.elem(i) *= y.elem(i);
  return;
}

/** @brief index wise (element-wise for vectors and matrices) product (also ordinary matrix or scalar product).:
  \f$\forall_{ik}:~ x_{ik} = \sum_j v_{ij}\, w_{jk}\f$ but also:
  \f$\forall_{i}:~ x_{i} = \sum_j v_{ij}\, w_{j}\f$
  \f$\forall_{i}:~ x_{ij} = v_{ij} w_{ij}\f$*/
template<class T>
void indexWiseProduct(rai::Array<T>& x, const rai::Array<T>& y, const rai::Array<T>& z) {
  if(y.N==1) { //scalar x any -> ..
    x=z;
    x*=y.scalar();
    return;
  }
  if(y.nd==1 && z.nd==1) {  //vector x vector -> element wise
    x=y;
    x*=z;
    return;
  }
  if(y.nd==1 && z.nd==2) {  //vector x matrix -> index-wise
    CHECK_EQ(y.N, z.d0, "wrong dims for indexWiseProduct:" <<y.N <<"!=" <<z.d0);
    x = z;
    if(isSparseMatrix(z)){
      CHECK(typeid(T)==typeid(double), "only for double!");
      x.sparse().rowWiseMult(y);
      return;
    }
    if(isRowShifted(z)){
      CHECK(typeid(T)==typeid(double), "only for double!");
      uint rowSize = x.rowShifted().rowSize;
      for(uint i=0; i<x.d0; i++) {
        T yi=y.p[i];
        double *xp=&x.rowShifted().entry(i,0);
        double *xstop=xp+rowSize;
        for(; xp!=xstop; xp++) *xp *= yi;
      }
      return;
    }
    for(uint i=0; i<x.d0; i++) {
      T yi=y.p[i];
      T* xp=&x(i, 0), *xstop=xp+x.d1;
      for(; xp!=xstop; xp++) *xp *= yi;
    }
    return;
  }
  if(y.nd==2 && z.nd==1) {  //matrix x vector -> index-wise
    CHECK_EQ(y.d1, z.N, "wrong dims for indexWiseProduct:" <<y.d1 <<"!=" <<z.N);
    x=y;
    for(uint i=0; i<x.d0; i++) for(uint j=0; j<x.d1; j++) x(i, j) *= z(j);
    return;
  }
  if(y.dim() == z.dim()) { //matrix x matrix -> element-wise
//    HALT("THIS IS AMBIGUOUS!");
    x = y;
    T* xp=x.p, *xstop=x.p+x.N, *zp=z.p;
    for(; xp!=xstop; xp++, zp++) *xp *= *zp;
    return;
  }
  HALT("operator% not implemented for "<<y.dim() <<" %" <<z.dim() <<" [I would like to change convention on the interpretation of operator% - contact Marc!")
}

/** @brief outer product (also exterior or tensor product): \f$\forall_{ijk}:~
  x_{ijk} = v_{ij}\, w_{k}\f$ */
template<class T>
rai::Array<T> crossProduct(const rai::Array<T>& y, const rai::Array<T>& z) {
  if(isNoArr(y)) return NoArr;
  if(y.nd==1 && z.nd==1) {
    CHECK(y.N==3 && z.N==3, "cross product only works for 3D vectors!");
    rai::Array<T> x(3);
    x.p[0]=y.p[1]*z.p[2]-y.p[2]*z.p[1];
    x.p[1]=y.p[2]*z.p[0]-y.p[0]*z.p[2];
    x.p[2]=y.p[0]*z.p[1]-y.p[1]*z.p[0];
    return x;
  }
  if(y.nd==2 && z.nd==1) { //every COLUMN of y is cross-product'd with z!
    CHECK(y.d0==3 && z.N==3, "cross product only works for 3D vectors!");
    return skew(-z) * y;
  }
  HALT("cross product - not yet implemented for these dimensions");
}

/// \f$\sum_i v_i\, w_i\f$, or \f$\sum_{ij} v_{ij}\, w_{ij}\f$, etc.
template<class T>
T scalarProduct(const rai::Array<T>& v, const rai::Array<T>& w) {
  T t(0);
  if(!v.special && !w.special) {
    CHECK_EQ(v.N, w.N,
             "scalar product on different array dimensions (" <<v.N <<", " <<w.N <<")");
    for(uint i=v.N; i--; t+=v.p[i]*w.p[i]);
  } else {
    if(isSparseVector(v) && isSparseVector(w)) {
      rai::SparseVector* sv = dynamic_cast<rai::SparseVector*>(v.special);
      rai::SparseVector* sw = dynamic_cast<rai::SparseVector*>(w.special);
      CHECK_EQ(v.d0, w.d0,
               "scalar product on different array dimensions (" <<v.d0 <<", " <<w.d0 <<")");
      int* ev=sv->elems.p, *ev_stop=ev+v.N, *ew=sw->elems.p, *ew_stop=ew+w.N;
      T* vp=v.p, *wp=w.p;
      for(; ev!=ev_stop && ew!=ew_stop;) {
        if(*ev==*ew) {
          t += *vp * *wp;
          ev++; vp++;
          ew++; wp++;
        } else if(*ev<*ew) { ev++; vp++; } else { ew++; wp++; }
      }
    } else {
      NIY;
    }
  }
  return t;
}

/// \f$\sum_{ij} g_{ij}\, v_i\, w_i\f$
template<class T>
T scalarProduct(const rai::Array<T>& g, const rai::Array<T>& v, const rai::Array<T>& w) {
  CHECK(v.N==w.N && g.nd==2 && g.d0==v.N && g.d1==w.N,
        "scalar product on different array dimensions (" <<v.N <<", " <<w.N <<")");
  T t(0);
  uint i, j;
  T* gp=g.p, *vp=v.p;
  for(i=0; i<g.d0; i++) {
    for(j=0; j<g.d1; j++) {
      t+=(*gp)*(*vp)*w.p[j];
      gp++;
    }
    vp++;
  }
  return t;
}

template<class T>
rai::Array<T> diagProduct(const rai::Array<T>& y, const rai::Array<T>& z) {
  CHECK((y.nd==1 && z.nd==2) || (y.nd==2 && z.nd==1), "");
  arr x;
  uint i, j;
  if(y.nd==1) {
    CHECK_EQ(y.N, z.d0, "");
    x=z;
    for(i=0; i<x.d0; i++) for(j=0; j<x.d1; j++) x(i, j) *= y(i);
    return x;
  }
  if(z.nd==1) {
    CHECK_EQ(z.N, y.d1, "");
    x=y;
    for(i=0; i<x.d0; i++) for(j=0; j<x.d1; j++) x(i, j) *= z(j);
    return x;
  }
  HALT("");
}

template<class T> rai::Array<T> elemWiseMin(const rai::Array<T>& v, const rai::Array<T>& w) {
  rai::Array<T> z;
  z.resizeAs(v);
  for(uint i=0; i<v.N; i++) z.elem(i) = v.elem(i)<w.elem(i)?v.elem(i):w.elem(i);
  return z;
}

template<class T> rai::Array<T> elemWiseMax(const rai::Array<T>& v, const rai::Array<T>& w) {
  rai::Array<T> z;
  z.resizeAs(v);
  for(uint i=0; i<v.N; i++) z.elem(i) = v.elem(i)>w.elem(i)?v.elem(i):w.elem(i);
  return z;
}

template<class T> rai::Array<T> elemWiseMax(const rai::Array<T>& v, const T& w) {
  rai::Array<T> z;
  z.resizeAs(v.N);
  for(uint i=0; i<v.N; i++) z.elem(i) = v.elem(i)>w?v.elem(i):w;
  return z;
}

template<class T> rai::Array<T> elemWiseMax(const T& v, const rai::Array<T>& w) {
  rai::Array<T> z;
  z.resizeAs(w.N);
  for(uint i=0; i<w.N; i++) z.elem(i) = v>w.elem(i)?v:w.elem(i);
  return z;
}

template<class T> rai::Array<T> elemWiseHinge(const rai::Array<T>& x) {
  rai::Array<T> z = x;
  for(T& v:z) if(v<0.) v=0.;
  return z;
}

template<class T> void writeConsecutiveConstant(std::ostream& os, const rai::Array<T>& x) {
  if(!x.N) return;
  uint yi=0;
  T y=x.elem(yi);
  for(uint i=1; i<x.N; i++) if(x.elem(i)!=y) {
      os <<'(' <<yi <<".." <<i-1 <<')' <<y <<' ';
      yi=i;
      y = x.elem(yi);
    }
  os <<'(' <<yi <<".." <<x.N-1 <<')' <<y;
}

//===========================================================================
//
/// @name tensor operations
//

#define DEBUG_TENSOR(x) //x
/// @name tensor

/** makes X to be a distribution over the left leftmost-indexed
  variables and normalizes it */
template<class T> void tensorCondNormalize(rai::Array<T>& X, int left) {
  uint i, j, dl=1, dr;
  T sum;
  if(left>=0) {  //normalize over the left variables
    for(j=0; j<(uint)left; j++) dl*=X.dim(j);
    dr=X.N/dl;
    CHECK_EQ(dl*dr, X.N, "");
    for(i=0; i<dr; i++) {
      sum=(T)0;
      for(j=0; j<dl; j++)  sum += X.p[j*dr + i];
      if(sum) for(j=0; j<dl; j++) X.p[j*dr + i] /= sum;
      else    for(j=0; j<dl; j++) X.p[j*dr + i] = (T)1/dl;
    }
  } else { //normalize over the right variables
    for(j=0; j<(uint)-left; j++) dl*=X.dim(j);
    dr=X.N/dl;
    CHECK_EQ(dl*dr, X.N, "");
    for(i=0; i<dl; i++) {
      sum=(T)0;
      for(j=0; j<dr; j++)  sum += X.p[i*dr + j];
      if(sum) for(j=0; j<dr; j++) X.p[i*dr + j] /= sum;
      else    for(j=0; j<dr; j++) X.p[i*dr + j] = (T)1/dr;
    }
  }
}

/** makes X to be a distribution over the left leftmost-indexed
  variables and normalizes it */
template<class T> void tensorCondMax(rai::Array<T>& X, uint left) {
  uint i, j, dl=1, dr, jmax;
  T pmax;
  for(j=0; j<left; j++) dl*=X.dim(j);
  dr=X.N/dl;
  CHECK_EQ(dl*dr, X.N, "");
  for(i=0; i<dr; i++) {
    jmax=0;
    pmax=X.p[jmax*dr + i];
    X.p[i]=(T)0;
    for(j=1; j<dl; j++) {
      if(X.p[j*dr + i]>pmax) {  jmax=j;  pmax=X.p[jmax*dr + i];  }
      X.p[j*dr + i]=(T)0;
    }
    X.p[jmax*dr + i]=(T)1;
  }
}

template<class T> void tensorCond11Rule(rai::Array<T>& X, uint left, double rate) {
  uint i, j, dl=1, dr, jmax1, jmax2;
  for(j=0; j<left; j++) dl*=X.dim(j);
  dr=X.N/dl;
  CHECK_EQ(dl*dr, X.N, "");
  arr X_i(dl);
  double amin=10.;
  for(i=0; i<dr; i++) {
    for(j=0; j<dl; j++) X_i(j) = X.p[j*dr + i];  //copy this row into one array
    //find the 1st and 2nd highest in this conditional part
    X_i.maxIndeces(jmax1, jmax2);
    if(X_i(jmax1)==X_i(jmax2)) continue;
    CHECK(X_i(jmax1)>X_i(jmax2), "must be really greater...");
    //compute the log ratio
    double a = ::log(rate)/::log(X_i(jmax1)/X_i(jmax2));
    //rescale everything [no -> globally rescale]
    //for(j=0;j<dl;j++)  X.p[j*dr + i] = pow(X.p[j*dr + i], a);
    if(a<amin) amin=a;
  }
  if(amin>1.) for(i=0; i<X.N; i++) X.elem(i) = pow(X.elem(i), amin);
}

/** makes X to be a distribution over the left leftmost-indexed
  variables and normalizes it */
template<class T> void tensorCondSoftMax(rai::Array<T>& X, uint left, double beta) {
  uint i;
  for(i=0; i<X.N; i++) X.p[i] = (T)::exp(beta*X.p[i]);
  tensorCondNormalize(X, left);
}

/** @brief checks whether X is a normalized distribution over the left leftmost-indexed
  variables */
template<class T> void tensorCheckCondNormalization(const rai::Array<T>& X, uint left, double tol) {
  uint i, j, dl=1, dr;
  double sum;
  for(j=0; j<left; j++) dl*=X.dim(j);
  dr=X.N/dl;
  CHECK_EQ(dl*dr, X.N, "");
  for(i=0; i<dr; i++) {
    sum=0.;
    for(j=0; j<dl; j++) sum += X.p[j*dr + i];
    CHECK(fabs(1.-sum)<tol, "distribution is not normalized: " <<X);
  }
}

template<class T> void tensorCheckCondNormalization_with_logP(const rai::Array<T>& X, uint left, double logP, double tol) {
  uint i, j, dl=1, dr;
  double sum, coeff=::exp(logP);
  for(j=0; j<left; j++) dl*=X.dim(j);
  dr=X.N/dl;
  CHECK_EQ(dl*dr, X.N, "");
  for(i=0; i<dr; i++) {
    sum=0.;
    uintA checkedIds;
    for(j=0; j<dl; j++) { sum += X.p[j*dr + i]*coeff; checkedIds.append(j*dr + i); }
    CHECK(fabs(1.-sum)<tol, "distribution is not normalized for parents-config#" <<i <<endl <<checkedIds<<endl <<" " <<X);
  }
}

/** X becomes a tensor product (maybe with some variables summed out)
  of A and B. pickA and pickB indicate which slots of A and B are
  associated with which slots of C and the summation sign. More
  precisely, if we have \f$C_{i_0i_1i_2} = \sum_{i_3i_4}
  A_{i_4i_2i_1}~ B_{i_3i_0}\f$ then you should call
  tensor(C, A, TUP(4, 2, 1), B, TUP(3, 0), 2); Here, the `2` indicates that
  the last two indices of i_0, .., i_4 are summed over, and C only
  becomes a 3rd rank instead of 5th rank tensor */
template<class T> void tensorEquation(rai::Array<T>& X, const rai::Array<T>& A, const uintA& pickA, const rai::Array<T>& B, const uintA& pickB, uint sum) {
  CHECK(&X!=&A && &X!=&B, "output tensor must be different from input tensors");
  CHECK(A.nd==pickA.N && B.nd==pickB.N, "miss-sized tensor references: " <<A.nd <<"!=" <<pickA.N <<" " <<B.nd <<"!=" <<pickB.N);

  uint n=1+rai::MAX(pickA.max(), pickB.max());
  uint i, j, r, s, N, res;
  intA a(n), b(n);
  uintA d(n), dx(n-sum), I, Ia(A.nd), Ib(B.nd);

  DEBUG_TENSOR(cout <<"pickA=" <<pickA <<" pickB=" <<pickB <<endl;);

  // permutation for A
  a=-1;
  for(i=0; i<A.nd; i++) a(pickA(i))=i;
  //j=A.nd;
  //for(i=0;i<n;i++) if(a(i)==-1){ a(i)=j; j++;  }
  DEBUG_TENSOR(cout <<"permutation for A: " <<a <<endl;);

  //permutation for B
  b=-1;
  for(i=0; i<B.nd; i++) b(pickB(i))=i;
  //j=B.nd;
  //for(i=0;i<n;i++) if(b(i)==-1){ b(i)=j; j++; }
  DEBUG_TENSOR(cout <<"permutation for B: " <<b <<endl;);

  //dimensionalities
  for(i=0; i<n; i++) {
    if(a(i)!=-1) r=A.dim(a(i)); else r=0;
    if(b(i)!=-1) s=B.dim(b(i)); else s=0;
    CHECK(!r || !s || r==s, "inconsistent sharing dimensionalities: " <<r <<"!=" <<s);
    d(i)=rai::MAX(r, s);
  }
  DEBUG_TENSOR(cout <<"full dimensionality d=" <<d <<endl;);

  //total elements:
  N=product(d);
  if(!sum) {
    res=1;
    //X.resizeTensor(d);
    CHECK_EQ(d, X.dim(), "for security, please set size before");
  } else {
    dx.resize(d.N-sum);
    res=1;
    for(j=0; j<dx.N; j++) dx(j)=d(j);
    for(; j<d .N; j++) res*=d(j);
    //X.resizeTensor(dx);
    CHECK_EQ(dx, X.dim(), "for security, please set size before");
  }
  CHECK_EQ(N, X.N*res, "");
  DEBUG_TENSOR(cout <<"dx=" <<dx <<" res=" <<res <<endl;);

  //here the copying and multiplying takes place...
  X.setZero();
  for(i=0; i<N; i++) {
    I = getIndexTuple(i, d);
    for(j=0; j<A.nd; j++) Ia(j)=I(pickA(j));
    for(j=0; j<B.nd; j++) Ib(j)=I(pickB(j));
    //DEBUG_TENSOR(cout <<"i=" <<i <<" I=" <<I <<" i/res=" <<i/res <<" Ia=" <<Ia <<" Ib=" <<Ib <<endl;)
    if(!sum) {
      X.elem(i) = A.elem(Ia) * B.elem(Ib);
    } else {
      X.elem(i/res) += A.elem(Ia) * B.elem(Ib);
    }
  }
}

template<class T> void tensorEquation_doesntWorkLikeThat(rai::Array<T>& X, const rai::Array<T>& A, const uintA& pickA, const rai::Array<T>& B, const uintA& pickB, uint sum) {
  CHECK(A.nd==pickA.N && B.nd==pickB.N, "miss-sized tensor references: " <<A.nd <<"!=" <<pickA.N <<" " <<B.nd <<"!=" <<pickB.N);

  uint n=1+rai::MAX(pickA.max(), pickB.max());
  uint i, j;
  uintA a(n), b(n);

  DEBUG_TENSOR(cout <<"pickA=" <<pickA <<" pickB=" <<pickB <<endl;);

  // permutations for A & B
  a=-1;  for(i=0; i<A.nd; i++) a(pickA(i))=i;
  b=-1;  for(i=0; i<B.nd; i++) b(pickB(i))=i;
  DEBUG_TENSOR(cout <<"permutation for A: " <<a <<"\npermutation for B: " <<b <<endl;);

  //permute tensors
  arr Aperm, Bperm;
  tensorPermutation(Aperm, A, a);
  tensorPermutation(Bperm, B, b);

  //dimensionalities: left-sum-right
  uint ldim=1, sdim=1, rdim=1;
  for(i=0; i<Aperm.nd-sum; i++) { ldim *= Aperm.d[i]; }
  for(i=0; i<sum; i++) { j = Aperm.d[sum+i]; CHECK_EQ(j, Bperm.d[i], ""); sdim*=j; }
  for(i=0; i<Bperm.nd-sum; i++) { rdim *= Bperm.d[sum+i]; }
  DEBUG_TENSOR(cout <<"ldim=" <<ldim <<" sdim=" <<sdim <<" rdim=" <<rdim <<endl;)

  //reshape to matrices
  Aperm.reshape(ldim, sdim);
  Bperm.reshape(sdim, rdim);

  //matrix multiplication
  innerProduct(X, Aperm, Bperm);

  //reshape
}

inline void getStrides(uintA& stride, uintA& dim) {
  stride.resize(dim.N+1);
  stride(dim.N) = 1;
  for(uint i=dim.N; i--;) stride(i) = stride(i+1)*dim(i);
}

//index and limit are w.r.t is the GLOBAL indexing!, j_stride w.r.t. the permuted
inline void multiDimIncrement(uint& Ycount, uint* index, uint* limit, uint* Yinc, uint* Ydec, uint nd) {
  uint k;
  for(k=nd; k--;) {
    Ycount+=Yinc[k];
    index[k]++;
    if(index[k]<limit[k]) break;  //we need the index only to decide when to overflow -- is there a more efficient way?
    index[k]=0;
    Ycount-=Ydec[k];
  }
}

inline void getMultiDimIncrement(const uintA& Xdim, const uintA& Yid, uint* Ydim, uint* Yinc, uint* Ydec) {
  uint i;
  memset(Ydim, 0, sizeof(uint)*maxRank);  for(i=0; i<Xdim.N; i++) if(i<Yid.N) Ydim[i]=Xdim(Yid.p[i]);    //dimension of Y
  memset(Yinc, 0, sizeof(uint)*maxRank);  Yinc[Yid.p[Yid.N-1]]=1;  for(i=Yid.N-1; i--;) Yinc[Yid.p[i]] = Ydim[i+1] * Yinc[Yid.p[i+1]];  //stride of Y
  for(i=Xdim.N; i--;) Ydec[i] = Xdim(i)*Yinc[i];
  //cout <<"Xdim=" <<Xdim <<"\nYid =" <<Yid <<"\nYdim=" <<uintA(Ydim, Yid.N) <<"\nYinc=" <<uintA(Yinc, Xdim.N) <<"\nYdec=" <<uintA(Ydec, Xdim.N) <<endl;
}

/** \f$Y_{i_Yid(0), i_Yid(1)} = \sum_{i_1} X_{i_0, i_1, i_2}\f$. Get the marginal Y
  from X, where Y will share the slots `Yid' with X */
template<class T> void tensorMarginal(rai::Array<T>& Y, const rai::Array<T>& X, const uintA& Yid) {
  uint Xcount, Ycount;
  CHECK_LE(Yid.N, X.nd, "can't take slots " <<Yid <<" from " <<X.nd <<"D tensor");

  //handle scalar case
  if(!Yid.N) {  Y.resize(1);  Y.nd=0;  Y.scalar()=sum(X);  return;  }

  //initialize looping
  uint I[maxRank];     memset(I, 0, sizeof(uint)*maxRank);  //index on X
  uint Ydim[maxRank], Yinc[maxRank], Ydec[maxRank];
  getMultiDimIncrement(X.dim(), Yid, Ydim, Yinc, Ydec);
  Y.resize(Yid.N, Ydim);
  Y.setZero();

  //loop
  for(Xcount=0, Ycount=0; Xcount<X.N; Xcount++) {
#if 0 //use this to check looping -- all routines below don't have this check anymore!
    cout <<"current globalIndex=" <<Xcount <<' ' <<Ycount <<' ' <<uintA(I, X.nd) <<endl;
    //check if incrementing Y worked out
    uint k, jj=0;
    for(k=0; k<Yid.N; k++) { jj*=Ydim[k]; jj+=I[Yid(k)]; }
    CHECK_EQ(jj, Ycount, "");
    //check if incrementing I worked out
    uintA II;
    getIndexTuple(II, Xcount, uintA(X.d, X.nd));
    CHECK_EQ(II, I, "not equal: " <<II <<uintA(I, X.nd));
#endif
    Y.p[Ycount] += X.p[Xcount];
    multiDimIncrement(Ycount, I, X.d, Yinc, Ydec, X.nd);
  }
}

/** \f$Y_{i_Yid(0), i_Yid(1)} = \sum_{i_1} X_{i_0, i_1, i_2}\f$. Get the marginal Y
  from X, where Y will share the slots `Yid' with X */
template<class T> void tensorPermutation(rai::Array<T>& Y, const rai::Array<T>& X, const uintA& Yid) {
  uint Xcount, Ycount;
  CHECK_EQ(Yid.N, X.nd, "can't take slots " <<Yid <<" from " <<X.nd <<"D tensor");

  //initialize looping
  uint I[maxRank];     memset(I, 0, sizeof(uint)*maxRank);  //index on X
  uint Ydim[maxRank], Yinc[maxRank], Ydec[maxRank];
  getMultiDimIncrement(X.dim(), Yid, Ydim, Yinc, Ydec);
  Y.resize(Yid.N, Ydim);

  //loop
  for(Xcount=0, Ycount=0; Xcount<X.N; Xcount++) {
    Y.p[Ycount] = X.p[Xcount];
    multiDimIncrement(Ycount, I, X.d, Yinc, Ydec, X.nd);
  }
}

/** \f$Y_{i_2, i_0} = {\rm max}_{i_1} X_{i_0, i_1, i_2}\f$. Get the ``max-marginal'' Y
  from X, where Y will share the slots `Yid' with X (basis of max-product BP) */
template<class T> void tensorMaxMarginal(rai::Array<T>& Y, const rai::Array<T>& X, const uintA& Yid) {
  uint Xcount, Ycount;
  CHECK_LE(Yid.N, X.nd, "can't take slots " <<Yid <<" from " <<X.nd <<"D tensor");

  //initialize looping
  uint I[maxRank];     memset(I, 0, sizeof(uint)*maxRank);  //index on X
  uint Ydim[maxRank], Yinc[maxRank], Ydec[maxRank];
  getMultiDimIncrement(X.dim(), Yid, Ydim, Yinc, Ydec);
  Y.resize(Yid.N, Ydim);
  Y.setZero();
  HALT("WRONG IMPLEMENTATION! - zero don't guarantee max...");

  //loop
  for(Xcount=0, Ycount=0; Xcount<X.N; Xcount++) {
    if(X.p[Xcount]>Y.p[Ycount]) Y.p[Ycount] = X.p[Xcount];
    multiDimIncrement(Ycount, I, X.d, Yinc, Ydec, X.nd);
  }
}

/** @brief \f$X_{i_0, i_1, i_2} \gets X_{i_0, i_1, i_2}~ Y_{i_Yid(0), i_Yid(1)}\f$. Multiply Y onto X,
  where Y shares the slots `Yid' with X */
template<class T> void tensorAdd_old(rai::Array<T>& X, const rai::Array<T>& Y, const uintA& Yid) {
  uint Xcount, Ycount;
  CHECK_EQ(Yid.N, Y.nd, "need to specify " <<Y.nd <<" slots, not " <<Yid.N);
  CHECK_LE(Yid.N, X.nd, "can't take slots " <<Yid <<" from " <<X.nd <<"D tensor");

  //handle scalar case
  if(!Yid.N) { CHECK_EQ(Y.N, 1, "");  X+=Y.scalar();  return; } //Y is only a scalar

  //initialize looping
  uint I[maxRank];     memset(I, 0, sizeof(uint)*maxRank);  //index on X
  uint Ydim[maxRank], Yinc[maxRank], Ydec[maxRank];
  getMultiDimIncrement(X.dim(), Yid, Ydim, Yinc, Ydec);
  Y.resize(Yid.N, Ydim);
  Y.setZero();

  //loop
  for(Xcount=0, Ycount=0; Xcount<X.N; Xcount++) {
    X.p[Xcount] += Y.p[Ycount];
    multiDimIncrement(Ycount, I, X.d, Yinc, Ydec, X.nd);
  }
}

template<class T> void tensorMarginal_old(rai::Array<T>& y, const rai::Array<T>& x, const uintA& xd, const uintA& ids) {
  uint i, j, k, n=product(xd);
  CHECK_EQ(x.N, n, "");
  //size y
  uintA yd(ids.N);
  for(i=0; i<ids.N; i++) yd(i)=xd(ids(i));
  //y.resize(yd); y.setZero();
  y.resize(product(yd)); y.setZero();

  uintA xt(xd.N); xt.setZero();
  for(i=0; i<n; i++) {
    //compute j
    for(j=0, k=0; k<ids.N; k++) { j*=yd(k); j+=xt(ids(k)); }
    //uintA yt(ids.N); for(k=ids.N;k--;){ yt(k)=xt(ids(k)); }
    //cout <<"i=" <<i <<" j=" <<j <<" yt=" <<yt <<" xt=" <<xt <<endl;
    y(j)+=x.elem(i);
    //increment xt
    for(k=xt.N; k--;) { xt(k)++; if(xt(k)<xd(k)) break; else xt(k)=0; }
  }
}

/** \f$X_{i_0, i_1, i_2} \gets X_{i_0, i_1, i_2}~ Y_{i_Yid(0), i_Yid(1)}\f$. Multiply Y onto X,
  where Y shares the slots `Yid' with X */
template<class T> void tensorMultiply(rai::Array<T>& X, const rai::Array<T>& Y, const uintA& Yid) {
  uint Xcount, Ycount;
  CHECK_EQ(Yid.N, Y.nd, "need to specify " <<Y.nd <<" slots, not " <<Yid.N);
  CHECK_LE(Yid.N, X.nd, "can't take slots " <<Yid <<" from " <<X.nd <<"D tensor");

  //handle scalar case
  if(!Yid.N) { CHECK_EQ(Y.N, 1, "");  X*=Y.scalar();  return; } //Y is only a scalar

  //initialize looping
  uint I[maxRank];     memset(I, 0, sizeof(uint)*maxRank);  //index on X
  uint Ydim[maxRank], Yinc[maxRank], Ydec[maxRank];
  getMultiDimIncrement(X.dim(), Yid, Ydim, Yinc, Ydec);

  //loop
  for(Xcount=0, Ycount=0; Xcount<X.N; Xcount++) {
    X.p[Xcount] *= Y.p[Ycount];
    multiDimIncrement(Ycount, I, X.d, Yinc, Ydec, X.nd);
  }
}

/** \f$X_{i_0, i_1, i_2} \gets X_{i_0, i_1, i_2}~ Y_{i_Yid(0), i_Yid(1)}\f$. Multiply Y onto X,
  where Y shares the slots `Yid' with X */
// TODO cope with division by 0, in particular 0/0
template<class T> void tensorDivide(rai::Array<T>& X, const rai::Array<T>& Y, const uintA& Yid) {
  uint Xcount, Ycount;
  CHECK_EQ(Yid.N, Y.nd, "need to specify " <<Y.nd <<" slots, not " <<Yid.N);
  CHECK_LE(Yid.N, X.nd, "can't take slots " <<Yid <<" from " <<X.nd <<"D tensor");

  //handle scalar case
  if(!Yid.N) { CHECK_EQ(Y.N, 1, "");  X/=Y.scalar();  return; } //Y is only a scalar

  //initialize looping
  uint I[maxRank];     memset(I, 0, sizeof(uint)*maxRank);  //index on X
  uint Ydim[maxRank], Yinc[maxRank], Ydec[maxRank];
  getMultiDimIncrement(X.dim(), Yid, Ydim, Yinc, Ydec);

  //loop
  for(Xcount=0, Ycount=0; Xcount<X.N; Xcount++) {
    // TODO division by zero??
    X.p[Xcount] = rai::DIV(X.p[Xcount], Y.p[Ycount]);
    multiDimIncrement(Ycount, I, X.d, Yinc, Ydec, X.nd);
  }
}

template<class T> void tensorAdd(rai::Array<T>& X, const rai::Array<T>& Y, const uintA& Yid) {
  uint Xcount, Ycount;
  CHECK_EQ(Yid.N, Y.nd, "need to specify " <<Y.nd <<" slots, not " <<Yid.N);
  CHECK_LE(Yid.N, X.nd, "can't take slots " <<Yid <<" from " <<X.nd <<"D tensor");

  //handle scalar case
  if(!Yid.N) { CHECK_EQ(Y.N, 1, "");  X+=Y.scalar();  return; } //Y is only a scalar

  //initialize looping
  uint I[maxRank];     memset(I, 0, sizeof(uint)*maxRank);  //index on X
  uint Ydim[maxRank], Yinc[maxRank], Ydec[maxRank];
  getMultiDimIncrement(X.dim(), Yid, Ydim, Yinc, Ydec);

  //loop
  for(Xcount=0, Ycount=0; Xcount<X.N; Xcount++) {
    X.p[Xcount] += Y.p[Ycount];
    multiDimIncrement(Ycount, I, X.d, Yinc, Ydec, X.nd);
  }
}

/** multiply y onto x, where x has dimensions `d' and y shares the
  dimensions `ids' with x */
template<class T> void tensorMultiply_old(rai::Array<T>& x, const rai::Array<T>& y, const uintA& d, const uintA& ids) {
  uint i, j, k, n=x.N;
  CHECK_EQ(n, product(d), "");

  uintA yd(ids.N);
  for(i=0; i<ids.N; i++) yd(i)=d(ids(i));
  CHECK_EQ(y.N, product(yd), "");

  uintA I(d.N); I.setZero();
  for(i=0; i<n; i++) {
    for(j=0, k=0; k<ids.N; k++) { j*=yd(k); j+=I(ids(k)); }
    x.elem(i) *= y.elem(j);
    for(k=I.N; k--;) { I(k)++; if(I(k)<d(k)) break; else I(k)=0; }
  }
}

//===========================================================================
//
/// @name set operations
//

/// x becomes the section of y and z
template<class T>
void setSection(rai::Array<T>& x, const rai::Array<T>& y, const rai::Array<T>& z) {
  x.clear();
  x.reserveMEM(rai::MIN(y.N, z.N));
  T* yp=y.p, *zp=z.p, *ystop=y.p+y.N, *zstop=z.p+z.N;
  for(yp=y.p; yp!=ystop; yp++) {
    for(zp=z.p; zp!=zstop; zp++) {
      if(*yp==*zp) {
        x.append(*yp); break;
      }
    }
  }
}

/// x becomes the section of y and z
template<class T>
void setUnion(rai::Array<T>& x, const rai::Array<T>& y, const rai::Array<T>& z) {
  uint i, j;
  if(&x!=&y) x=y;
  for(i=0; i<z.N; i++) {
    for(j=0; j<y.N; j++) if(z(i)==y(j)) break;
    if(j==y.N) x.append(z(i));
  }
}

/// x becomes the section of y and z
template<class T>
void setMinus(rai::Array<T>& x, const rai::Array<T>& y) {
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
template<class T> rai::Array<T> setSectionSorted(const rai::Array<T>& x, const rai::Array<T>& y,
    bool (*comp)(const T& a, const T& b)) {
  rai::Array<T> R;
//  R.anticipateMEM(rai::MIN(x.N,y.N));
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

template<class T>
void setMinusSorted(rai::Array<T>& x, const rai::Array<T>& y,
                    bool (*comp)(const T& a, const T& b)) {
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
template<class T> uint numberSharedElements(const rai::Array<T>& x, const rai::Array<T>& y) {
  rai::Array<T> z;
  setSection(z, x, y);
  return z.N;
}

//===========================================================================
//
/// @name randomizations
//

/// Assign all elements of \c a to a uniformly distributed discrete value in {low, .., hi}
template<class T> void rndInteger(rai::Array<T>& a, int low, int high, bool add) {
  if(!add) for(uint i=0; i<a.N; i++) a.p[i] =(T)(low+(int)rnd.num(1+high-low));
  else     for(uint i=0; i<a.N; i++) a.p[i]+=(T)(low+(int)rnd.num(1+high-low));
}

/// Assign all elements of \c a to a uniformly distributed continuous value in [low, hi]
template<class T> void rndUniform(rai::Array<T>& a, double low, double high, bool add) {
  if(!add) for(uint i=0; i<a.N; i++) a.p[i] =(T)rnd.uni(low, high);
  else     for(uint i=0; i<a.N; i++) a.p[i]+=(T)rnd.uni(low, high);
}

template<class T> void rndNegLogUniform(rai::Array<T>& a, double low, double high, bool add) {
  if(!add) for(uint i=0; i<a.N; i++) a.p[i] =(T)(-::log(rnd.uni(low, high)));
  else     for(uint i=0; i<a.N; i++) a.p[i]+=(T)(-::log(rnd.uni(low, high)));
}

/** Assign all elements of x to a Gaussian random variable where
    _each_ elelemt has the given stdDev -- devide by sqrt(N) if you
    want the multivariate Gaussian to have a given standard deviation!
    If add is true, the Gaussian noise is added to the existing
    value */
template<class T> void rndGauss(rai::Array<T>& x, double stdDev, bool add) {
  if(!add) for(uint i=0; i<x.N; i++) x.p[i] =(T)(stdDev*rnd.gauss());
  else     for(uint i=0; i<x.N; i++) x.p[i]+=(T)(stdDev*rnd.gauss());
}

/// a gaussian random vector with Id covariance matrix (sdv = sqrt(dimension))
/*template<class T> void rndGauss(rai::Array<T>& a, bool add){
  if(!add) for(uint i=0;i<a.N;i++) a.p[i]=rnd.gauss();
  else     for(uint i=0;i<a.N;i++) a.p[i]+=rnd.gauss();
  }*/

/// returns an array with \c dim Gaussian noise elements
/*template<class T> rai::Array<T>& rndGauss(double stdDev, uint dim){
  static rai::Array<T> z;
  stdDev/=::sqrt(dim);
  z.resize(dim);
  rndGauss(z, stdDev);
  return z;
  }*/

/** @brief from a vector of numbers, calculates the softmax distribution
  soft(i) = exp(beta*a(i)), and returns a random sample from
  this distribution (an index in {0, .., a.N-1}) */
template<class T> uint softMax(const rai::Array<T>& a, arr& soft, double beta) {
  double norm=0., r;
  uint i; int sel=-1;
  resizeAs(soft, a);
  for(i=0; i<a.N; i++) {
    soft(i)=exp(beta*a(i));
    norm+=soft(i);
  }
  r=rnd.uni();
  for(i=0; i<a.N; i++) {
    soft(i)/=norm;
    r-=soft(i);
    if(sel==-1 && r<0.) sel=i;
  }
  return sel;
}

//===========================================================================
//
/// @name operators
//

namespace rai {
//addition
template<class T> Array<T> operator+(const Array<T>& y, const Array<T>& z) { Array<T> x(y); x+=z; return x; }

//subtraction
template<class T> Array<T> operator-(const Array<T>& y, const Array<T>& z) { Array<T> x(y); x-=z; return x; }

/// transpose
template<class T> Array<T> operator~(const Array<T>& y) { Array<T> x; transpose(x, y); return x; }
/// negative
template<class T> Array<T> operator-(const Array<T>& y) { Array<T> x; negative(x, y);  return x; }
/// outer product (notation borrowed from the wedge product, though not anti-symmetric)
template<class T> Array<T> operator^(const Array<T>& y, const Array<T>& z) { Array<T> x; outerProduct(x, y, z); return x; }

/// inner product
template<class T> Array<T> operator*(const Array<T>& y, const Array<T>& z) { Array<T> x; innerProduct(x, y, z); return x; }
/// scalar multiplication
template<class T> Array<T> operator*(const Array<T>& y, T z) {             Array<T> x(y); x*=z; return x; }
/// scalar multiplication
template<class T> Array<T> operator*(T y, const Array<T>& z) {             Array<T> x(z); x*=y; return x; }

/// inverse
template<class T> Array<T> operator/(int y, const Array<T>& z) {  CHECK_EQ(y, 1, ""); Array<T> x=inverse(z); return x; }
/// scalar division
template<class T> Array<T> operator/(const Array<T>& y, T z) {             Array<T> x(y); x/=z; return x; }
/// element-wise division
template<class T> Array<T> operator/(const Array<T>& y, const Array<T>& z) { Array<T> x(y); x/=z; return x; }

/// A^-1 B
template<class T> Array<T> operator|(const Array<T>& A, const Array<T>& B) { Array<T> x; lapack_mldivide(x, A, B); return x; }

/// contatenation of two arrays
template<class T> Array<T> operator, (const Array<T>& y, const Array<T>& z) { Array<T> x(y); x.append(z); return x; }

/// x.append(y)
template<class T> Array<T>& operator<<(Array<T>& x, const T& y) { x.append(y); return x; }

/// x.append(y)
template<class T> Array<T>& operator<<(Array<T>& x, const Array<T>& y) { x.append(y); return x; }

/// index-wise (elem-wise) product (x_i = y_i z_i   or  X_{ij} = y_i Z_{ij}  or  X_{ijk} = Y_{ij} Z_{jk}   etc)
template<class T> Array<T> operator%(const Array<T>& y, const Array<T>& z) { Array<T> x; indexWiseProduct(x, y, z); return x; }

#define UpdateOperator( op )        \
  template<class T> Array<T>& operator op (Array<T>& x, const Array<T>& y){ \
    if(isNoArr(x)){ return x; } \
    if(isSparseMatrix(x) && isSparseMatrix(y)){ x.sparse() op y.sparse(); return x; }  \
    if(isRowShifted(x) && isRowShifted(y)){ x.rowShifted() op y.rowShifted(); return x; }  \
    CHECK(!isSpecial(x), "");  \
    CHECK(!isSpecial(y), "");  \
    CHECK_EQ(x.N, y.N, "binary operator on different array dimensions (" <<x.N <<", " <<y.N <<")"); \
    T *xp=x.p, *xstop=xp+x.N;              \
    const T *yp=y.p;              \
    for(; xp!=xstop; xp++, yp++) *xp op *yp;       \
    return x;           \
  }                 \
  \
  template<class T> Array<T>& operator op (Array<T>& x, T y ){ \
    if(isNoArr(x)){ return x; } \
    if(isSparseMatrix(x)){ x.sparse() op y; return x; }  \
    if(isRowShifted(x)){ x.rowShifted() op y; return x; }  \
    CHECK(!isSpecial(x), "");  \
    T *xp=x.p, *xstop=xp+x.N;              \
    for(; xp!=xstop; xp++) *xp op y;        \
    return x;           \
  } \
  \
  template<class T> void operator op (Array<T>&& x, const Array<T>& y){ \
    if(isNoArr(x)){ return; } \
    if(isSparseMatrix(x) && isSparseMatrix(y)){ x.sparse() op y.sparse(); return; }  \
    if(isRowShifted(x) && isRowShifted(y)){ x.rowShifted() op y.rowShifted(); return; }  \
    CHECK(!isSpecial(x), "");  \
    CHECK_EQ(x.N, y.N, "binary operator on different array dimensions (" <<x.N <<", " <<y.N <<")"); \
    T *xp=x.p, *xstop=xp+x.N;              \
    const T *yp=y.p;              \
    for(; xp!=xstop; xp++, yp++) *xp op *yp;       \
  }                 \
  \
  template<class T> void operator op (Array<T>&& x, T y ){ \
    if(isNoArr(x)){ return; } \
    if(isSparseMatrix(x)){ x.sparse() op y; return; }  \
    if(isRowShifted(x)){ x.rowShifted() op y; return; }  \
    CHECK(!isSpecial(x), "");  \
    T *xp=x.p, *xstop=xp+x.N;              \
    for(; xp!=xstop; xp++) *xp op y;        \
  }

UpdateOperator(|=)
UpdateOperator(^=)
UpdateOperator(&=)
UpdateOperator(+=)
UpdateOperator(-=)
UpdateOperator(*=)
UpdateOperator(/=)
UpdateOperator(%=)
#undef UpdateOperator

#define BinaryOperator( op, updateOp)         \
  template<class T> Array<T> operator op(T y, const Array<T>& z){               Array<T> x; x.resizeAs(z); x=y; x updateOp z; return x; } \
  template<class T> Array<T> operator op(const Array<T>& y, T z){               Array<T> x(y); x updateOp z; return x; }

BinaryOperator(+, +=)
BinaryOperator(-, -=)
//BinaryOperator(% , *=);
//BinaryOperator(/ , /=);
#undef BinaryOperator

/// allows a notation such as x <<"[0 1; 2 3]"; to initialize an array x
//template<class T> Array<T>& operator<<(Array<T>& x, const char* str) { std::istringstream ss(str); ss >>x; return x; }

/// calls Array<T>::read
template<class T> std::istream& operator>>(std::istream& is, Array<T>& x) { x.read(is); return is; }

/// calls Array<T>::write
//template<class T> std::ostream& operator<<(std::ostream& os, const Array<T>& x) {
//  if(is_shared_ptr<T>::value == true){
//    os <<'{';
//    for(uint i=0; i<x.N; i++){ if(x.elem(i)) os <<' ' <<*x.elem(i); else os <<" <NULL>"; }
//    os <<" }" <<std::flush;
//  }else{
//    x.write(os);
//  }
//  return os;
//}

/// equal in size and all elements
template<class T> bool operator==(const Array<T>& v, const Array<T>& w) {
  if(!w) return !v; //if w==NoArr
  if(!samedim(v, w)) return false;
  const T* vp=v.p, *wp=w.p, *vstop=vp+v.N;
  for(; vp!=vstop; vp++, wp++)
    if(*vp != *wp) return false;
  return true;
}

/// element-wise equal to constant
template<class T> Array<byte> operator==(const Array<T>& v, const T& w) {
  Array<byte> x;
  resizeAs(x, v);
  x.setZero();
  const T* vp=v.p, *vstop=vp+v.N;
  byte* xp=x.p;
  for(; vp!=vstop; vp++, xp++)
    if(*vp == w) *xp=1;
  return x;
}

///// equal in size and all elements
//template<class T> bool operator==(const Array<T>& v, const T *w) {
//  const T *vp=v.p, *wp=w, *vstop=vp+v.N;
//  for(; vp!=vstop; vp++, wp++)
//    if(*vp != *wp) return false;
//  return true;
//}

/// not equal
template<class T> bool operator!=(const Array<T>& v, const Array<T>& w) {
  return !(v==w);
}

/// lexical comparison
template<class T> bool operator<(const Array<T>& v, const Array<T>& w) {
  if(v.N==w.N) {
    for(uint i=0; i<v.N; i++) {
      if(v.p[i]>w.p[i]) return false;
      if(v.p[i]<w.p[i]) return true;
    }
    return false; //they are equal
  }
  return v.N<w.N;
}

} //namespace

//===========================================================================
//
/// @name arithmetic operators
//

template<class T> void negative(rai::Array<T>& x, const rai::Array<T>& y) {
  x=y;
  T* xp=x.p, *xstop=xp+x.N;
  for(; xp!=xstop; xp++) *xp = - (*xp);
}

//---------- unary functions

inline double sigm(double x) {  return 1./(1.+::exp(-x)); }

inline double sign(double x) {  return (x > 0) - (x < 0); }

#define UnaryFunction( func )         \
  template<class T>           \
  rai::Array<T> func (const rai::Array<T>& y){    \
    rai::Array<T> x;           \
    if(&x!=&y) x.resizeAs(y);         \
    T *xp=x.p, *xstop=xp+x.N, *yp=y.p;            \
    for(; xp!=xstop; xp++, yp++) *xp = (T)::func( (double) *yp );  \
    return x;         \
  }

// trigonometric functions
UnaryFunction(acos);
UnaryFunction(asin);
UnaryFunction(atan);
UnaryFunction(cos);
UnaryFunction(sin);
UnaryFunction(tan);

// hyperbolic functions
UnaryFunction(cosh);
UnaryFunction(sinh);
UnaryFunction(tanh);
UnaryFunction(acosh);
UnaryFunction(asinh);
UnaryFunction(atanh);

// exponential and logarithmic functions
UnaryFunction(exp);
UnaryFunction(log);
UnaryFunction(log10);

//roots
UnaryFunction(sqrt);
UnaryFunction(cbrt);

// nearest integer and absolute value
UnaryFunction(ceil);
UnaryFunction(fabs);
UnaryFunction(floor);
UnaryFunction(sigm);

UnaryFunction(sign);

#undef UnaryFunction

//---------- binary functions

#define BinaryFunction( func )            \
  template<class T>             \
  rai::Array<T> func(const rai::Array<T>& y, const rai::Array<T>& z){ \
    CHECK_EQ(y.N,z.N,             \
             "binary operator on different array dimensions (" <<y.N <<", " <<z.N <<")"); \
    rai::Array<T> x;             \
    x.resizeAs(y);              \
    for(uint i=x.N;i--; ) x.p[i]= func(y.p[i], z.p[i]);      \
    return x;           \
  }                 \
  \
  template<class T>             \
  rai::Array<T> func(const rai::Array<T>& y, T z){     \
    rai::Array<T> x;             \
    x.resizeAs(y);              \
    for(uint i=x.N;i--; ) x.p[i]= func(y.p[i], z);     \
    return x;           \
  }                 \
  \
  template<class T>             \
  rai::Array<T> func(T y, const rai::Array<T>& z){     \
    rai::Array<T> x;             \
    x.resizeAs(z);              \
    for(uint i=x.N;i--; ) x.p[i]= func(y, z.p[i]);     \
    return x;           \
  }

BinaryFunction(atan2);
#ifndef RAI_MSVC
BinaryFunction(pow);
#endif
BinaryFunction(fmod);
#undef BinaryFunction

#ifndef RAI_doxy // exclude these macros when generating the documentation
// (doxygen can't handle them...)
#endif //(doxygen exclusion)

/*

/// element-wise linear combination (plus with scalar factors for each array)
template<class T> void plusSASA(rai::Array<T>& x, T a, const rai::Array<T>& y, T b, const rai::Array<T>& z){
CHECK_EQ(y.N,z.N, "must have same size for adding!");
uint i, n=y.N;
x.resizeAs(y);
for(i=0;i<n;i++) x.p[i]=a*y.p[i]+b*z.p[i];
}*/

#if 0
#define IMPLEMENT_Array(x) void implement_Array_##x(){ rai::Array<x> dummy; }
#define IMPLEMENT_Array_(x, key) void implement_Array_##key(){ rai::Array<x> dummy; }

template struct rai::Array<double>;
template struct rai::Array<int>;
template struct rai::Array<uint>;
template struct rai::Array<float>;
template struct rai::Array<byte>;
template struct rai::Array<char>;
template struct rai::Array<bool>;
#define T double
#  include "array_instantiate.cpp"
#define T int
#  include "array_instantiate.cpp"
#endif

//===========================================================================
//
// lists
//

template<class T> char listWrite(const rai::Array<std::shared_ptr<T>>& L, std::ostream& os, const char* ELEMSEP, const char* delim) {
  if(delim) os <<delim[0];
  for(uint i=0; i<L.N; i++) { if(i) os <<ELEMSEP;  if(L.elem(i)) os <<*L.elem(i); else os <<"<NULL>"; }
  if(delim) os <<delim[1] <<std::flush;
  return '#';
}

template<class T> char listWrite(const rai::Array<T*>& L, std::ostream& os, const char* ELEMSEP, const char* delim) {
  if(delim) os <<delim[0];
  for(uint i=0; i<L.N; i++) { if(i) os <<ELEMSEP;  if(L.elem(i)) os <<*L.elem(i); else os <<"<NULL>"; }
  if(delim) os <<delim[1] <<std::flush;
  return '#';
}

template<class T> rai::String listString(const rai::Array<T*>& L) {
  rai::String str;
  listWrite(L, str);
  return str;
}

template<class T> void listWriteNames(const rai::Array<T*>& L, std::ostream& os) {
  os <<'(';
  for(uint i=0; i<L.N; i++) { if(i) os <<' ';  if(L.elem(i)) os <<L.elem(i)->name; else os <<"<NULL>"; }
  os <<')' <<std::flush;
}

template<class T> void listRead(rai::Array<T*>& L, std::istream& is, const char* delim) {
  CHECK(!L.N, "delete the list before reading!");
  CHECK(delim, "automatic list reading requires delimiters");
  char c;
  if(delim) { rai::skip(is); is.get(c); CHECK_EQ(c, delim[0], "couldn't parse opening list delimiter"); }
  for(;;) {
    c=rai::peerNextChar(is);
    if(c==delim[1]) { is.get(c); break; }
    if(!is.good()) break;
    L.append(new T());
    L.last()->read(is);
  }
}

template<class T> void listClone(rai::Array<T*>& L, const rai::Array<T*>& M) {
  listDelete(L);
  L.resizeAs(M);
  for(uint i=0; i<L.N; i++) L.elem(i)=M.elem(i)->newClone();
}

template<class T> void listResize(rai::Array<T*>& L, uint N) {
  listDelete(L);
  L.resize(N);
  for(uint i=0; i<N; i++) L.elem(i)=new T();
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

template<class T> void listCopy(rai::Array<T*>& L, const rai::Array<T*>& M) {
  listDelete(L);
  L.resizeAs(M);
  for(uint i=0; i<L.N; i++) L.elem(i)=new T(*M.elem(i));
}

template<class T> void listCopy(rai::Array<ptr<T>>& L, const rai::Array<ptr<T>>& M) {
  L.clear();
  L.resizeAs(M);
  for(uint i=0; i<L.N; i++) L.elem(i) = make_shared<T>(*M.elem(i));
}

template<class T> void listDelete(rai::Array<T*>& L) {
  for(uint i=L.N; i--;) delete L.elem(i);
  L.clear();
}

template<class T> void listReindex(rai::Array<T*>& L) {
  for(uint i=0; i<L.N; i++) L.elem(i)->ID=i;
}

template<class T> T* listFindByName(const rai::Array<T*>& L, const char* name) {
  for_list(T,  e,  L) if(e->name==name) return e;
  //std::cerr <<"\n*** name '" <<name <<"' not in this list!" <<std::endl;
  return NULL;
}

template<class T> T* listFindByType(const rai::Array<T*>& L, const char* type) {
  for_list(T,  e,  L) if(!strcmp(e->type, type)) return e;
  //std::cerr <<"type '" <<type <<"' not in this list!" <<std::endl;
  return NULL;
}

template<class T> T* listFindValue(const rai::Array<T*>& L, const T& x) {
  for_list(T,  e,  L) if(*e==x) return e;
  //std::cerr <<"value '" <<x <<"' not in this list!" <<std::endl;
  return NULL;
}

template<class T, class LowerOperator> void listSort(rai::Array<T*>& L, LowerOperator lowerop) {
  std::sort(L.p, L.pstop, lowerop);
  //for(uint i=0;i<L.N;i++) L(i)->index=i;
}

//===========================================================================
//
// graphs
//

template<class vert, class edge> void graphDelete(rai::Array<vert*>& V, rai::Array<edge*>& E) {
  listDelete(E);
  listDelete(V);
}

template<class vert, class edge> edge* graphGetEdge(vert* from, vert* to) {
  for_list(edge,  e,  to->inLinks) if(e->from==from) return e;
  return NULL;
}

template<class vert, class edge> void graphRandomUndirected(rai::Array<vert*>& V, rai::Array<edge*>& E, uint N, double connectivity) {
  uint i, j;
  for(i=0; i<N; i++) V.append(new vert);
  for(i=0; i<N; i++) for(j=i+1; j<N; j++) {
      if(rnd.uni()<connectivity) newEdge(i, j, E);
    }
}

template<class vert, class edge> void graphRandomLinear(rai::Array<vert*>& V, rai::Array<edge*>& E, uint N) {
  uint i;
  for(i=0; i<N; i++) V.append(new vert);
  for(i=1; i<N; i++) newEdge(i-1, i, E);
}

template<class vert, class edge> void graphRandomTree(rai::Array<vert*>& V, rai::Array<edge*>& E, uint N, uint roots) {
  uint i;
  for(i=0; i<N; i++) V.append(new vert);
  CHECK_GE(roots, 1, "");
  for(i=roots; i<N; i++)  newEdge(rnd(i), i, E);
}

template<class vert, class edge>
void graphMaximumSpanningTree(rai::Array<vert*>& V, rai::Array<edge*>& E, const arr& W) {
  CHECK(W.nd==2 && W.d0==W.d1, "");
  uint i;
  for(i=0; i<W.d0; i++) new_elem(V);

  boolA done(V.N);  done=false;
  uintA addedNodes;

  if(!V.N) return;

  i=rnd(V.N);
  addedNodes.append(i); done(i)=true;

  uint j, k;
  double Wmax;
  uintA m;
  while(addedNodes.N<V.N) {
    m.clear();
    for(i=0; i<addedNodes.N; i++) {
      j=addedNodes(i);
      for(k=0; k<V.N; k++) if(!done(k) && (!m.N || W(j, k)>Wmax)) { m=TUP(j, k); Wmax=W(j, k); }
    }
    CHECK(m.N, "graph is not connected!");
    if(done(m(1))) m.permute(0, 1);
    newEdge(m(0), m(1), E);
    CHECK(!done(m(1)), "node added twice??");
    done(m(1))=true;
    addedNodes.append(m(1));
  }
  graphMakeLists(V, E);
}

/*template<class vert, class edge> void graphRandomFixedDegree(rai::Array<vert*>& V, rai::Array<edge*>& E, uint N, uint degree){
  uint i;
  for(i=0;i<N;i++) V.append(new vert);
  uintA D(N); D.setZero(); //degrees of vertices
  for(i=0;i<N;i++) for(j=i+1;j<N;j++){
    if(rnd.uni()<connectivity) newEdge(i, j, E);
  }
  for(i=0;i<I.N;i++) I(i)=i/degree; //a list, where each node appears degree-times
  I.permuteRandomly();
  cout <<I <<endl;
  for(i=0;i<I.N;i+=2) newEdge(I(i), I(i+1), E);
  graphConnectUndirected(V, E);
  graphWriteUndirected(cout, V, E);
}*/

template<class vert, class edge> void graphRandomFixedDegree(rai::Array<vert*>& V, rai::Array<edge*>& E, uint N, uint d) {
  // --- from Joris' libDAI!!
  // Algorithm 1 in "Generating random regular graphs quickly"
  // by A. Steger and N.C. Wormald
  //
  // Draws a random graph with size N and uniform degree d
  // from an almost uniform probability distribution over these graphs
  // (which becomes uniform in the limit that d is small and N goes
  // to infinity).

  CHECK_EQ((N*d)%2, 0, "");

  uint i;
  for(i=0; i<N; i++) V.append(new vert);

  bool ready = false;
  uint tries = 0;
  while(!ready) {
    tries++;

    // Start with N*d points {0, 1, ..., N*d-1} (N*d even) in N groups.
    // Put U = {0, 1, ..., N*d-1}. (U denotes the set of unpaired points.)
    uintA U;
    U.setStraightPerm(N*d);

    // Repeat the following until no suitable pair can be found: Choose
    // two random points i and j in U, and if they are suitable, pair
    // i with j and delete i and j from U.
    listDelete(E);
    bool finished = false;
    while(!finished) {
      U.permuteRandomly();
      uint i1, i2;
      bool suit_pair_found = false;
      for(i1=0; i1<U.N-1 && !suit_pair_found; i1++) {
        for(i2=i1+1; i2<U.N && !suit_pair_found; i2++) {
          if((U(i1)/d) != (U(i2)/d)) {  // they are suitable (refer to different nodes)
            suit_pair_found = true;
            newEdge(U(i1)/d, U(i2)/d, E);
            U.remove(i2);  // first remove largest
            U.remove(i1);  // remove smallest
          }
          if(!suit_pair_found || !U.N)  finished = true;
        }
      }
    }

    if(!U.N) {
      // G is a graph with edge from vertex r to vertex s if and only if
      // there is a pair containing points in the r'th and s'th groups.
      // If G is d-regular, output, otherwise return to Step 1.
      uintA degrees(N);
      degrees.setZero();
      for_list(edge,  e,  E) {
        degrees(e->ifrom)++;
        degrees(e->ito)  ++;
      }
      ready = true;
      for(uint n=0; n<N; n++) {
        CHECK_LE(degrees(n), d, "");
        if(degrees(n)!=d) {
          ready = false;
          break;
        }
      }
    } else ready=false;
  }

  graphConnectUndirected(V, E);
  //graphWriteUndirected(cout, V, E);
}

template<class vert, class edge> void graphLayered(rai::Array<vert*>& V, rai::Array<edge*>& E, const uintA& layers, bool interConnections) {
  uint i, j, a=0, b=0;
  uint l, L=layers.N;
  for(l=0; l<L; l++) {
    for(i=0; i<layers(l); i++) {
      V.append(new vert);
      if(l) for(j=0; j<layers(l-1); j++) newEdge(b+j, a+i, E);
      if(l && interConnections) for(j=0; j<i; j++) newEdge(a+j, a+i, E);
    }
    a+=layers(l);             //a is the offset of current layer
    if(l) b+=layers(l-1);  //b is the offset of previous layer
  }
}

template<class vert, class edge> void graphMakeLists(rai::Array<vert*>& V, rai::Array<edge*>& E) {
  for_list(vert,  v,  V) {
    v->parentOf.clear();
    v-> inLinks.clear();
  }
  for_list(edge,  e,  E) {
    e->from = V(e->ifrom);
    e->to   = V(e->ito);
    e->from->parentOf.append(e);
    e->to  -> inLinks.append(e);
  }
}

template<class vert, class edge> void graphConnectUndirected(rai::Array<vert*>& V, rai::Array<edge*>& E) {
  for_list(vert,  v,  V) v->edges.clear();
  for_list(edge,  e,  E) {
    e->from = V(e->ifrom);
    e->to   = V(e->ito);
    e->from->edges.append(e);
    e->to  ->edges.append(e);
  }
}

template<class vert, class edge> edge* newEdge(vert* a, vert* b, rai::Array<edge*>& E) {
  return newEdge(a->index, b->index, E);
}

template<class edge> edge* newEdge(uint a, uint b, rai::Array<edge*>& E) {
  edge* e=new edge;
  e->index=E.N;
  E.append(e);
  e->ifrom=a; e->ito=b;
  return e;
}

template<class vert, class edge> edge* new_edge_deprecated(vert* a, vert* b, rai::Array<edge*>& E) {
  edge* e=new edge;
  e->index=E.N;
  E.append(e);
  e->a=a->index; e->b=b->index;
  return e;
}

template<class vert, class edge> edge* del_edge(edge* e, rai::Array<vert*>& V, rai::Array<edge*>& E, bool remakeLists) {
  //uint i, k=e-E.p;
  //E.remove(k);
  uint i;
  for(i=0; i<E.N; i++) if(E(i)==e) break;
  E.remove(i);
  if(remakeLists) graphMakeLists(V, E);
  return e;
}

template<class vert, class edge> void graphWriteDirected(std::ostream& os, const rai::Array<vert*>& V, const rai::Array<edge*>& E) {
  for_list(vert,  v,  V) {
    for_list(edge,  e,  v->inLinks) os <<e->ifrom <<' ';
    os <<"-> ";
    os <<v_COUNT <<" -> ";
    for_list(edge,  e2,  v->parentOf) os <<e2->ito <<' ';
    os <<'\n';
  }
  for_list(edge,  e,  E) os <<e->ifrom <<"->" <<e->ito <<'\n';
  //for_list(Type,  e,  E) os <<e->from->name <<"->" <<e->to->name <<'\n';
}

template<class vert, class edge> void graphWriteUndirected(std::ostream& os, const rai::Array<vert*>& V, const rai::Array<edge*>& E) {
  for_list(vert,  v,  V) {
    os <<v_COUNT <<": ";
    for_list(edge,  e,  v->edges) if(e->ifrom==v_COUNT) os <<e->ito <<' '; else os <<e->ifrom <<' ';
    os <<'\n';
  }
  for_list(edge,  e,  E) os <<e->ifrom <<"-" <<e->ito <<'\n';
}

template<class vert, class edge> bool graphTopsort(rai::Array<vert*>& V, rai::Array<edge*>& E) {
  NIY;
#if 0
  rai::Array<vert*> noInputs;
  noInputs.memMove=true;
  uintA newIndex(V.N);
  intA inputs(V.N);

  uint count=0;

  for_list(vert,  v,  V) v->index = v_COUNT;

  for(vert* v:V) {
    inputs(v->index)=v->numInputs(); //inLinks.N;
    if(!inputs(v->index)) noInputs.append(v);
  }

  while(noInputs.N) {
    v=noInputs.popFirst();
    newIndex(v->index)=count++;
    for_list(edge,  e,  v->parentOf) {
      inputs(e->to->index)--;
      if(!inputs(e->to->index)) noInputs.append(e->to);
    }
  }

  if(count!=V.N) return false;

  //success!
  V.permuteInv(newIndex);
  for_list(vert,  vv,  V) vv->index = vv_COUNT;

  //-- reindex edges as well:
  newIndex.resize(E.N);
  count=0;
  for(vert* v:V) for(edge* e:v->parentOf) newIndex(e->index)=count++;
  E.permuteInv(newIndex);
  for_list(edge, e, E) e->index=e_COUNT;
#endif
  return true;
}

template<class vert> rai::Array<vert*> graphGetTopsortOrder(rai::Array<vert*>& V) {
  rai::Array<vert*> fringe;
  rai::Array<vert*>::memMove=true;
  intA inputs(V.N);
  rai::Array<vert*> order;

  for_list(vert,  v,  V) v->ID = v_COUNT;

  for(vert* v:V) {
    inputs(v->ID)=v->numInputs(); //inLinks.N;
    if(!inputs(v->ID)) fringe.append(v);
  }

  while(fringe.N) {
    v=fringe.popFirst();
    order.append(v);

    for(vert* to : v->parentOf) {
      inputs(to->ID)--;
      if(!inputs(to->ID)) fringe.append(to);
    }
  }

  CHECK_EQ(order.N, V.N, "can't top sort");

  return order;
}

template<class vert, class edge>
void graphRevertEdge(rai::Array<vert*>& V, rai::Array<edge*>& E, edge* e) {
  uint i=e->ifrom;  e->ifrom=e->ito;  e->ito=i;
  graphMakeLists(V, E);
}

template<class vert, class edge, class CompareOp>
void maximumSpanningTree(rai::Array<vert*>& V, rai::Array<edge*>& E, const CompareOp& cmp) {
  vert* n;
  edge* m;
  boolA nodeAdded(V.N);  nodeAdded=false;
  boolA edgeAdded(E.N);  edgeAdded=false;
  uintA addedNodes;

  if(!V.N) return;

  n=V(rnd(V.N));
  addedNodes.append(n->index); nodeAdded(n->index)=true;
  while(addedNodes.N<V.N) {
    m=0;
    for(uint i=0; i<addedNodes.N; i++) {
      n=V(addedNodes(i));
      for_list(edge,  e,  n->parentOf) if(!nodeAdded(e->to  ->index) && (!m || cmp(e, m))) m=e;
      for(edge* e: n->inLinks) if(!nodeAdded(e->from->index) && (!m || cmp(e, m))) m=e;
    }
    CHECK(m, "graph is not connected!");
    edgeAdded(m->index)=true;
    if(nodeAdded(m->to->index)) graphRevertEdge(V, E, m);
    CHECK(!nodeAdded(m->to->index), "node added twice??");
    nodeAdded(m->to->index)=true;
    addedNodes.append(m->to->index);
  }
  for_list_rev(edge, e, E) if(!edgeAdded(e_COUNT)) del_edge(e, V, E, true);
  graphMakeLists(V, E);
}

//namespace rai{
//template<class T> Array<T>::Array(std::initializer_list<const char*> list) {
//  init();
//  for(const char* t : list) append(rai::String(t));
//}

//template<> template<class S> Array<rai::String>::Array(std::initializer_list<S> list) {
//    HALT("NEVER CALL THIS CONSTRUCTOR!")
//}
//}
