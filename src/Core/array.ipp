/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "array.h"

#include <algorithm>
#include <type_traits>

#define ARRAY_flexiMem true

namespace rai {

//fwd declarations

extern uint lineCount;
char skip(std::istream& is, const char* skipSymbols, const char* stopSymbols, bool skipCommentLines);
char peerNextChar(std::istream& is, const char* skipSymbols, bool skipCommentLines);
char getNextChar(std::istream& is, const char* skipSymbols, bool skipCommentLines);
bool parse(std::istream& is, const char* str, bool silent);
uint rndInt(uint up);

//===========================================================================
//
// Array class
//

template<class T> char Array<T>::memMove=(char)-1;
template<class T> int Array<T>::sizeT=-1;

/// standard constructor -- this becomes an empty array
template<class T> Array<T>::Array()
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
template<class T> Array<T>::Array(const Array<T>& a) : Array() { operator=(a); }

/// copy constructor
template<class T> Array<T>::Array(Array<T>&& a)
  : /*std::vector<T>(std::move(a)),*/
    p(a.p),
    N(a.N),
    nd(a.nd),
    d0(a.d0), d1(a.d1), d2(a.d2),
    d(&d0),
    isReference(a.isReference),
    M(a.M),
    special(a.special) {
  if constexpr(std::is_same_v<T, double>){
    if(a.jac) jac = std::move(a.jac);
  }
  // CHECK_EQ(a.d, &a.d0, "NIY for larger tensors");
  if(a.d!=&a.d0) { d=a.d; a.d=&a.d0; }
  a.p=NULL;
  a.N=a.nd=a.d0=a.d1=a.d2=0;
  a.isReference=false;
  a.special=NULL;
}

/// constructor with resize
template<class T> Array<T>::Array(uint i) : Array() { resize(i); }

/// constructor with resize
template<class T> Array<T>::Array(uint i, uint j) : Array() { resize(i, j); }

/// constructor with resize
template<class T> Array<T>::Array(uint i, uint j, uint k) : Array() { resize(i, j, k); }

/// initialization via {1., 2., 3., ...} lists..
template<class T> Array<T>::Array(std::initializer_list<T> values) : Array() { operator=(values); }

/// initialization via {1., 2., 3., ...} lists, with certain dimensionality
template<class T> Array<T>::Array(std::initializer_list<uint> dim, std::initializer_list<T> values) : Array() { operator=(values); reshape(dim); }

template<class T> Array<T>::Array(const T* p, uint size, bool byReference) : Array() { if(byReference) referTo(p, size); else setCarray(p, size); }

template<class T> Array<T>::~Array() {
#if 0
  clear();
#else //faster (leaves members non-zeroed..)
  if(special) { delete special; special=NULL; }
  if(d!=&d0) { delete[] d; }
  if(M) {
    globalMemoryTotal -= M*sizeT;
    if(memMove==1) free(p); else delete[] p;
  }
#endif
}

/// frees all memory; this becomes an empty array
template<class T> Array<T>&  Array<T>::clear() {
  if(special) { delete special; special=NULL; }
  freeMEM();
  return *this;
}

/// resize 1D array, discard the previous contents
template<class T> Array<T>& Array<T>::resize(uint D0) { nd=1; d0=D0; resetD(); resizeMEM(d0, false); return *this; }

/// resize but copy the previous contents
template<class T> Array<T>& Array<T>::resizeCopy(uint D0) { nd=1; d0=D0; resetD(); resizeMEM(d0, true); return *this; }

/// reshape the dimensionality (e.g. from 2D to 1D); throw an error if this actually requires to resize the memory
template<class T> Array<T>& Array<T>::reshape(int D0) {
  if(D0<0) D0=N;
  CHECK_EQ((int)N, D0, "reshape must preserve total memory size");
  nd=1; d0=D0; d1=d2=0; resetD();
  return *this;
}

/// same for 2D ...
template<class T> Array<T>& Array<T>::resize(uint D0, uint D1) { nd=2; d0=D0; d1=D1; resetD(); resizeMEM(d0*d1, false); return *this; }

/// ...
template<class T> Array<T>& Array<T>::resizeCopy(uint D0, uint D1) { nd=2; d0=D0; d1=D1; resetD(); resizeMEM(d0*d1, true); return *this; }

/// ...
template<class T> Array<T>& Array<T>::reshape(int D0, int D1) {
  if(D0<0) D0=N/D1; else if(D1<0) D1=N/D0;
  CHECK_EQ((int)N, D0*D1, "reshape must preserve total memory size");
  nd=2; d0=D0; d1=D1; d2=0;
  resetD();
  return *this;
}

/// same for 3D ...
template<class T> Array<T>& Array<T>::resize(uint D0, uint D1, uint D2) { nd=3; d0=D0; d1=D1; d2=D2; resetD(); resizeMEM(d0*d1*d2, false); return *this; }

/// ...
template<class T> Array<T>& Array<T>::resizeCopy(uint D0, uint D1, uint D2) { nd=3; d0=D0; d1=D1; d2=D2; resetD(); resizeMEM(d0*d1*d2, true); return *this; }

/// ...
template<class T> Array<T>& Array<T>::reshape(int D0, int D1, int D2) {
  if(D0<0) D0=N/(D1*D2); else if(D1<0) D1=N/(D0*D2); else if(D2<0) D2=N/(D0*D1);
  CHECK_EQ((int)N, D0*D1*D2, "reshape must preserve total memory size");
  nd=3; d0=D0; d1=D1; d2=D2;
  resetD();
  return *this;
}

/// resize to multi-dimensional tensor
template<class T> Array<T>& Array<T>::resize(uint ND, uint* dim) {
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
template<class T> Array<T>& Array<T>::resizeCopy(uint ND, uint* dim) {
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
template<class T> Array<T>& Array<T>::reshape(uint ND, uint* dim) {
  nd=ND; d0=d1=d2=0; resetD();
  if(nd>0){
    d0=dim[0];
    if(nd>1){
      d1=dim[1];
      if(nd>2){
        d2=dim[2];
        if(nd>3) { d=new uint[nd];  memmove(d, dim, nd*sizeof(uint)); }
      }
    }
  }
  //for(uint j=0; j<nd && j<3; j++) {(&d0)[j]=dim[j]; }
  uint S=(nd>0?1:0);
  for(uint j=0; j<nd; j++) S*=dim[j];
  CHECK_EQ(N, S, "reshape must preserve total memory size");
  return *this;
}

/// resize to multi-dimensional tensor
template<class T> Array<T>& Array<T>::resize(const Array<uint>& newD) { resize(newD.N, newD.p); return *this; }

/// resize to multi-dimensional tensor
template<class T> Array<T>& Array<T>::resizeCopy(const Array<uint>& newD) { resizeCopy(newD.N, newD.p); return *this; }

/// resize to multi-dimensional tensor
template<class T> Array<T>& Array<T>::reshape(const Array<uint>& newD) { reshape(newD.N, newD.p); return *this; }

template<class T> Array<T>& Array<T>::reshape(std::initializer_list<uint> dim) { reshape(dim.size(), (uint*)dim.begin()); return *this; }

template<class T> Array<T>& Array<T>::resizeAs(const Array<T>& a) {
  CHECK(this!=&a, "never do this!!!");
  if(isReference) CHECK_EQ(N, a.N, "resize of a reference (e.g. subarray) is not allowed! (only a resize without changing memory size)");
  nd=a.nd; d0=a.d0; d1=a.d1; d2=a.d2;
  resetD();
  if(nd>3) { d=new uint[nd];  memmove(d, a.d, nd*sizeof(uint)); }
  resizeMEM(a.N, false);
  return *this;
}

/// make it the same size as \c a and copy previous content
template<class T> Array<T>& Array<T>::resizeCopyAs(const Array<T>& a) {
  CHECK(this!=&a, "never do this!!!");
  nd=a.nd; d0=a.d0; d1=a.d1; d2=a.d2; resetD();
  if(nd>3) { d=new uint[nd];  memmove(d, a.d, nd*sizeof(uint)); }
  resizeMEM(a.N, true);
  return *this;
}

template<class T> Array<T>& Array<T>::reshapeAs(const Array<T>& a) {
  CHECK(this!=&a, "never do this!!!");
  CHECK_EQ(N, a.N, "reshape must preserve total memory size");
  nd=a.nd; d0=a.d0; d1=a.d1; d2=a.d2; resetD();
  if(nd>3) { d=new uint[nd];  memmove(d, a.d, nd*sizeof(uint)); }
  return *this;
}

/// return the k-th dimensionality
template<class T> uint Array<T>::dim(uint k) const {
  CHECK(k<nd, "dimensionality range check error: " <<k <<"!<" <<nd);
  if(!d && k<3) return (&d0)[k]; else return d[k];
}

#ifdef RAI_CLANG
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wdynamic-class-memaccess"
#endif
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#pragma GCC diagnostic ignored "-Wmismatched-dealloc"

/// allocate memory (maybe using \ref flexiMem)
template<class T> void Array<T>::resizeMEM(uint n, bool copy, int Mforce) {
  if(n==N) return;
  CHECK(!isReference, "resize of a reference (e.g. subarray) is not allowed! (only a resize without changing memory size)");

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
#if 0 //non flexi mem
    Mnew=n;
#else //flexi mem
    if(Mold==0 && n>0) {
      Mnew=n;      //first time: exact allocation
    } else if(n>Mold || 10+2*n<Mold/4) {
      Mnew=20+2*n; //big down-or-up-resize: allocate with some extra space
    } else {
      Mnew=Mold;   //small down-size: don't really resize memory
    }
#endif
  }

#ifdef RAI_USE_STDVEC
  if(Mnew!=Mold) { vec_type::resize(Mnew); }
//  vec_type::reserve(Mnew);
//  vec_type::resize(Mnew);
  p = vec_type::data();
#else
  CHECK_GE(Mnew, n, "");
  CHECK((p && M) || (!p && !M), "");
  if(Mnew!=Mold) {  //if M changed, allocate the memory
    globalMemoryTotal -= Mold*sizeT;
    globalMemoryTotal += Mnew*sizeT;
    if(globalMemoryTotal>globalMemoryBound) {
      if(globalMemoryStrict) {
        globalMemoryTotal -= Mnew*sizeT;
        HALT("out of memory: " <<((globalMemoryTotal+Mnew)>>20) <<"MB");
      }
      LOG(0) <<"using massive memory: " <<(globalMemoryTotal>>20) <<"MB";
    }
    if(Mnew) {
      if(memMove==1) {
        if(p) {
          p=(T*)realloc(p, Mnew*sizeT);
        } else {
          p=(T*)malloc(Mnew*sizeT);
          //memset(p, 0, Mnew*sizeT);
        }
        if(!p) { HALT("memory allocation failed! Wanted size = " <<Mnew*sizeT <<"bytes"); }
      } else {
        T* pold = p;
        p=new T [Mnew];
        if(!p) { HALT("memory allocation failed! Wanted size = " <<Mnew*sizeT <<"bytes"); }
        if(copy) for(uint i=N<n?N:n; i--;) p[i]=pold[i];
        if(pold) delete[] pold;
      }
      M=Mnew;
    } else {
      if(p) {
        if(memMove==1) {
          free(p);
        } else {
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
template<class T> void Array<T>::freeMEM() {
#ifdef RAI_USE_STDVEC
  vec_type::clear();
#else
  if(M) {
    globalMemoryTotal -= M*sizeT;
    if(memMove==1) {
      free(p);
    } else {
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

///this was a reference; becomes a copy
template<class T> Array<T>& Array<T>::dereference() {
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
template<class T> void Array<T>::resetD() {
  if(d && d!=&d0) { delete[] d; d=NULL; }
  d=&d0;
}

//***** append, insert & remove

/// append an (uninitialized) element to the array and return its reference -- the array becomes 1D!
template<class T> T& Array<T>::append() {
  if(nd==2 && d1==1)
    resizeCopy(d0+1, d1);
  else
    resizeCopy(N+1);
  return p[N-1];
}

/// append an element to the array -- the array becomes 1D!
template<class T> Array<T>& Array<T>::append(const T& x) {
#if 0
  reshape(N);
  vec_type::push_back(x);
  p = vec_type::data();
  d0 = N = vec_type::size();
#else
  resizeCopy(N+1);
  p[N-1]=x;
#endif
  return *this;
}

/// append an element to the array -- the array becomes 1D!
template<class T> Array<T>& Array<T>::append(const T& x, uint multiple) {
  uint i=N;
  resizeCopy(N+multiple);
  for(; i<N; i++) p[i]=x;
  return *this;
}

/// append another array to the array (by copying it) -- the array might become 1D!
template<class T> Array<T>& Array<T>::append(const Array<T>& x, bool asRow) {
  uint oldN=N, xN=x.N, i;
  if(!xN) return *this;
  if(nd==2 && x.nd==1 && d1==x.d0)
    resizeCopy(d0+1, d1);
  else if(nd==2 && x.nd==2 && d1==x.d1)
    resizeCopy(d0+x.d0, d1);
  else if(!N){
    resizeAs(x);
    if(asRow && x.nd!=2) reshape(1, x.N);
  }else{
    resizeCopy(N+xN);
    if(asRow) reshape(N/x.N, x.N);
  }
  if(memMove==1) memmove(p+oldN, x.p, sizeT*xN);
  else for(i=0; i<xN; i++) p[oldN+i]=x.p[i];
  return *this;
}

/// append a C array to the array (by copying it) -- the array might become 1D!
template<class T> Array<T>& Array<T>::append(const T* q, uint n) {
  uint oldN=N, i;
  if(nd==2 && d1==n)
    resizeCopy(d0+1, d1);
  else
    resizeCopy(N+n);
  if(memMove==1) memmove(p+oldN, q, sizeT*n);
  else for(i=0; i<n; i++) p[n+i]=q[i];
  return *this;
}

/// append an element to the array if it is not included yet -- the array becomes 1D! [TL]
template<class T> void Array<T>::setAppend(const T& x) {
  if(findValue(x) < 0)
    append(x);
}

/// append elements of another array to the array which are not included yet -- the array might become 1D! [TL]
template<class T> void Array<T>::setAppend(const Array<T>& x) {
  for(const T& i:x) setAppend(i);
}

/// remove and return the first element of the array (must have size>1)
template<class T> T Array<T>::popFirst() { T x; x=elem(0); remove(0);   return x; }

/// remove and return the last element of the array (must have size>1)
template<class T> T Array<T>::popLast() { T x=elem(N-1); CHECK_EQ(nd, 1, ""); d0--; N--;/*resizeCopy(N-1);*/ return x; }

/// remove and return the last element of the array (must have size>1)
template<class T> void Array<T>::removeLast() { resizeCopy(N-1); }

/// reverse this array
template<class T> void Array<T>::reverse() {
  Array<T> L2;
  uint i;
  for(i=this->N; i--;) L2.append(this->elem(i));
  *this = L2;
}

/// reverse the rows of this array
template<class T> void Array<T>::reverseRows() {
  CHECK_EQ(this->nd, 2, "Can only reverse rows of 2 dim arrays. nd=" << this->nd);
  Array<T> L2;
  uint i;
  for(i=this->d0; i--;) L2.append(this->operator[](i));
  L2.reshape(this->d0, this->d1);
  *this = L2;
}

/// the array contains `copies' copies of the old one
template<class T> void Array<T>::replicate(uint copies) {
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
template<class T> void Array<T>::insert(uint i, const T& x) {
  CHECK(memMove, "only with memMove");
  uint Nold=N;
  resizeCopy(Nold+1);
  if(i<Nold) memmove(p+i+1, p+i, sizeT*(Nold-i));
  p[i]=x;
}

template<class T> void Array<T>::insert(uint i, const Array<T>& x) {
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
template<class T> void Array<T>::remove(int i, uint n) {
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
template<class T> void Array<T>::removePerm(uint i) {
  p[i]=p[N-1];
  resizeCopy(N-1);
}

/// remove (delete) a subsequence of the array -- the array becomes 1D!  [only with memMove!] (throws error if value does not exist)
template<class T> bool Array<T>::removeValue(const T& x, bool errorIfMissing) {
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
template<class T> void Array<T>::removeAllValues(const T& x) {
  CHECK(memMove, "only with memMove");
  uint i;
  for(i=0; i<N; i++) if(p[i]==x) { remove(i, 1); i--; }
}

/// replace n elements at pos i by the sequence x -- the array becomes 1D!  [only with memMove!]
template<class T> void Array<T>::replace(uint i, uint n, const Array<T>& x) {
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
template<class T> void Array<T>::delRows(int i, uint k) {
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
template<class T> void Array<T>::insRows(int i, uint k) {
  CHECK(memMove, "only with memMove");
  CHECK_EQ(nd, 2, "only for matricies");
  if(i<0) i+=d0+1;
  CHECK_LE(i, (int)d0, "range error (" <<i <<">=" <<d0 <<")");
  int n=d0;
  resizeCopy(d0+k, d1);
  if(n>i) memmove(p+(i+k)*d1, p+i*d1, sizeT*d1*(n-i));
  if(k)   memset(p+ i   *d1, 0, sizeT*d1*k);
}

/// deletes k columns starting from the i-th (i==d1 -> deletes the last k columns)
template<class T> void Array<T>::delColumns(int i, uint k) {
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
template<class T> Array<T>& Array<T>::insColumns(int i, uint k) {
  CHECK(memMove, "only with memMove");
  CHECK_EQ(nd, 2, "only for matricies");
  if(!k) return *this;
  if(i<0) i+=d1+1;
  CHECK_LE(i, (int)d1, "range check error");
  uint n=d1;
  resizeCopy(d0, n+k);
  for(uint j=d0; j--;) {
    if(i<(int)n) memmove(p+j*d1+(i+k), p+j*n+i, sizeT*(n-i));
    memset(p+j*d1+i, 0, sizeT*k);
    if(i) memmove(p+j*d1, p+j*n, sizeT*i);
  }
  return *this;
}

/// changes the range of one dimension (generalization of ins/delColumn to arbitrary tensors)
template<class T> void Array<T>::resizeDim(uint k, uint dk) {
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
template<class T> Array<uint> Array<T>::dim() const {
  Array<uint> dims;
  dims.setCarray(d, nd);
  return dims;
}

/// scalar reference (valid only for a 0-dim or 1-dim array of size 1)
template<class T> T& Array<T>::elem() const {
  CHECK(nd<=2 && N==1, "scalar range error (nd=" <<nd <<"), N=" <<N <<")");
  return *p;
}

/// the \c ith element
template<class T> T& Array<T>::elem(int i) const {
  if(i<0) i+=N;
  CHECK(i>=0 && i<(int)N, "range error (" <<i <<">=" <<N <<")");
  return p[i];
}

/// access that invariantly works for sparse and non-sparse matrices
template<class T> T& Array<T>::elem(int i, int j) {
  if(i<0) i += d0;
  if(j<0) j += d1;
  CHECK(nd==2 && (uint)i<d0 && (uint)j<d1,
        "2D range error (" <<nd <<"=2, " <<i <<"<" <<d0 <<", " <<j <<"<" <<d1 <<")");
  if constexpr(std::is_same_v<T, double>){
    if(isSparseMatrix(*this)) {
      return sparse().addEntry(i, j);
    }
    if(isRowShifted(*this)) {
      return rowShifted().elemNew(i, j);
    }
  }
  return p[i*d1+j];
}

/// multi-dimensional (tensor) access
template<class T> T& Array<T>::elem(const Array<uint>& I) const {
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
  return elem(rndInt(N));
}

/// 1D reference access (throws an error if not 1D or out of range)
template<class T> T& Array<T>::operator()(int i) const {
  if(i<0) i += d0;
  CHECK(nd==1 && (uint)i<d0,
        "1D range error (" <<nd <<"=1, " <<i <<"<" <<d0 <<")");
  return p[i];
}

/// 2D reference access
template<class T> T& Array<T>::operator()(int i, int j) const {
  if(i<0) i += d0;
  if(j<0) j += d1;
  CHECK(nd==2 && (uint)i<d0 && (uint)j<d1 && !special,
        "2D range error (" <<nd <<"=2, " <<i <<"<" <<d0 <<", " <<j <<"<" <<d1 <<")");
  return p[i*d1+j];
}

/// 3D reference access
template<class T> T& Array<T>::operator()(int i, int j, int k) const {
  if(i<0) i += d0;
  if(j<0) j += d1;
  if(k<0) k += d2;
  CHECK(nd==3 && (uint)i<d0 && (uint)j<d1 && (uint)k<d2 && !special,
        "3D range error (" <<nd <<"=3, " <<i <<"<" <<d0 <<", " <<j <<"<" <<d1 <<", " <<k <<"<" <<d2 <<")");
  return p[(i*d1+j)*d2+k];
}

template<class T> Array<T> Array<T>::ref() const {
  Array<T> x;
  x.referTo(*this);
  return x;
}

template<class T> Array<T> Array<T>::operator()(std::pair<int, int> I) const {
  Array<T> z;
  z.referToRange(*this, I);
  //  if(I.size()==2) z.referToRange(*this, I.begin()[0], I.begin()[1]);
  //  else if(I.size()==0) z.referTo(*this);
  //  else if(I.size()==1) z.referToDim(*this, I.begin()[0]);
  //  else HALT("range list needs 0,1, or 2 entries exactly");
  return z;
}

/// range reference access
template<class T> Array<T> Array<T>::operator()(int i, std::pair<int, int> J) const {
  Array<T> z;
  z.referToRange(*this, i, J);
//  if(J.size()==2)
//  else if(J.size()==0) z.referToDim(*this, i);
//  else if(J.size()==1) z.referToDim(*this, i, J.begin()[0]);
//  else HALT("range list needs 0,1, or 2 entries exactly");
  return z;
}

/// range reference access
template<class T> Array<T> Array<T>::operator()(int i, int j, std::initializer_list<int> K) const {
  Array<T> z;
  if(K.size()==2) z.referToRange(*this, i, j, {K.begin()[0], K.begin()[1]});
  else if(K.size()==0) z.referToDim(*this, i, j);
  else if(K.size()==1) z.referToDim(*this, i, j, K.begin()[0]);
  else HALT("range list needs 0,1, or 2 entries exactly");
  return z;
}

/// get a subarray (e.g., row of a matrix); use in conjuction with operator()() to get a reference
template<class T> Array<T> Array<T>::operator[](int i) const {
  CHECK(!special, "");
  Array<T> z;
  z.referToDim(*this, i);
  return z;
}

/// convert a subarray into a reference (e.g. a[3]()+=.123)
//template<class T> T& Array<T>::operator()() const { return scalar(); } //return (*this); }

/// return the index of an entry equal to x, or -1
template<class T> int Array<T>::findValue(const T& x) const { uint i; for(i=0; i<N; i++) if(p[i]==x) return i; return -1; }

/// return all indices to entries equal to x
template<class T> void Array<T>::findValues(Array<uint>& indices, const T& x) const {
  indices.clear();
  uint i;
  for(i=0; i<N; i++)
    if(p[i]==x) indices.append(i);
}

/// non-reference copy (to apply followup operators, like x.copy().reshape(3,5))
template<class T> Array<T> Array<T>::copy() const { return Array<T>(*this); }

/** @brief a sub array of a 1D Array (corresponds to matlab [i:I]); when
  the upper limit I is -1, it is replaced by the max limit (like
  [i:]) */
template<class T> Array<T> Array<T>::sub(std::pair<int, int> _I) const {
  CHECK_EQ(nd, 1, "1D range error ");
  int i=_I.first, I=_I.second-1;
  Array<T> x;
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
template<class T> Array<T> Array<T>::sub(std::pair<int, int> _I, std::pair<int, int> _J) const {
  CHECK_EQ(nd, 2, "2D range error ");
  int i=_I.first, I=_I.second-1, j=_J.first, J=_J.second-1;
  Array<T> x;
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
template<class T> Array<T> Array<T>::sub(std::pair<int, int> _I, std::pair<int, int> _J, std::pair<int, int> _K) const {
  CHECK_EQ(nd, 3, "3D range error ");
  int i=_I.first, I=_I.second-1, j=_J.first, J=_J.second-1, k=_K.first, K=_K.second-1;
  Array<T> x;
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
template<class T> Array<T> Array<T>::pick(std::pair<int, int> _I, Array<uint> cols) const {
  CHECK_EQ(nd, 2, "2D range error ");
  int i=_I.first, I=_I.second-1;
  Array<T> x;
  if(i<0) i+=d0;
  if(I<0) I+=d0;
  CHECK(i>=0 && I>=0 && i<=I, "lower limit higher than upper!");
  x.resize(I-i+1, cols.N);
  for(int ii=i; ii<=I; ii++) for(int l=0; l<(int)cols.N; l++) x(ii-i, l)=operator()(ii, cols(l));
  return x;
}

template<class T> Array<T> Array<T>::pick(Array<uint> elems) const {
  Array<T> x;
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
  //x.reshape(elems.dim());
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
Array<T> Array<T>::row(uint row_index) const {
  return sub({row_index, row_index+1},{ 0, d1 - 1+1});
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
Array<T> Array<T>::col(uint col_index) const {
  return sub({0, d0 - 1+1},{ col_index, col_index+1}).reshape(d0);
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
Array<T> Array<T>::rows(uint start_row, uint end_row) const {
  return sub({start_row, end_row - 1+1},{ 0, d1 - 1+1});
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
Array<T> Array<T>::cols(uint start_col, uint end_col) const {
  return sub({0, d0 - 1+1},{ start_col, end_col - 1+1});
}

/// makes this array a reference to the C buffer
template<class T> Array<T>& Array<T>::referTo(const T* buffer, uint n) {
  freeMEM();
  isReference=true;
  nd=1; d0=N=n; d1=d2=0;
  p=(T*)buffer;
  return *this;
}

template<class T> Array<T>& Array<T>::operator=(std::initializer_list<T> values) {
  resize(values.size());
  uint i=0;
  for(T t : values) elem(i++)=t;
  return *this;
}

/// set all elements to value \c v
template<class T> Array<T>& Array<T>::operator=(const T& v) {
  if(N) {
    T* x=p, *xstop=p+N;
    for(; x!=xstop; x++) *x = v;
  }
  return *this;
}

/// copy operator
template<class T> Array<T>& Array<T>::operator=(const Array<T>& a) {
  CHECK(this!=&a, "never do this!!!");
  //if(a.temp){ takeOver(*((Array<T>*)&a)); return *this; }
  resizeAs(a);
  if(memMove) memmove(p, a.p, sizeT*N);
  else for(uint i=0; i<N; i++) p[i]=a.p[i];
  if(special) { delete special; special=NULL; }
  if constexpr(std::is_same_v<T, double>){
    if(isSpecial(a)) special_copy(*this, a);
    if(a.jac) jac = std::make_unique<arr>(*a.jac);
  }
  return *this;
}

/// copy operator
//template<class T> Array<T>& Array<T>::operator=(const std::vector<T>& a) {
//  setCarray(&a.front(), a.size());
//  return *this;
//}

/** @brief same as memset(p, 0, sizeT*N); precondition: memMove is
  true! */
template<class T> Array<T>& Array<T>::setZero(byte zero) {
  CHECK(memMove, "can set array's memory to zero only if memMove option is true");
  memset(p, zero, sizeT*N);
  return *this;
}

/// concatenate 2D matrices (or vectors) column-wise
template<class T> Array<T> catCol(const Array<Array<T>*>& X) {
  uint d0=X(0)->d0, d1=0;
  for(const Array<T>* x:X) { CHECK((x->nd==2 || x->nd==1) && x->d0==d0, ""); d1+=x->nd==2?x->d1:1; }
  Array<T> z;
  if(isSparse(*X.first())) {
    NIY;
//      z.sparse().resize(d0, d1, 0);
//      d1=0;
//      for(const Array<T>* x:  X) {
//        arr* xx = dynamic_cast<arr*>(x);
//          CHECK(xx->isSparse(), "");
//          CHECK(xx->nd==2,"");
//          z.sparse().add(x->sparse(), 0, d1);
//          d1+=x->d1;
//      }
  } else {
    z.resize(d0, d1).setZero();
    d1=0;
    for(const Array<T>* x:  X) { z.setMatrixBlock(*x, 0, d1); d1+=x->nd==2?x->d1:1; }
  }
  return z;
}

template<class T> Array<T> catCol(std::initializer_list<rai::Array<T>> X) {
  Array<Array<T>*> Xp;
  for(const Array<T>& x:  X) Xp.append((Array<T>*)&x);
  return catCol(Xp);
}

/// concatenate 2D matrices (or vectors) column-wise
template<class T> Array<T> catCol(const Array<Array<T>>& X) {
  Array<const Array<T>*> Xp;
  for(const Array<T>& x:  X) Xp.append(&x);
  return catCol(Xp);
}

/// set all entries to same value x [default: don't change dimension]
template<class T> void Array<T>::setConst(const T& x, int d) {
  if(d!=-1) resize(d);
  uint i;
  for(i=0; i<N; i++) elem(i)=x;
}

/** @brief becomes the n-dim identity matrix [default:
  don't change dimension (already has to be squared matrix)] */
template<class T> void Array<T>::setId(int d) {
  CHECK(d!=-1 || (nd==2 && d0==d1), "need squared matrix to set to identity");
  if(d!=-1) resize(d, d);
  setZero();
  if constexpr(std::is_scalar_v<T>){
    for(uint i=0; i<d0; i++) operator()(i, i)=(T)1;
  }else NIY;
}

template<class T> void Array<T>::setDiag(const T& x, int d) {
  CHECK(d!=-1 || nd==2, "need squared matrix to set to diagonal");
  if(d!=-1) resize(d, d);
  if(d==-1) d=(int)(d0<d1?d0:d1);
  setZero();
  for(uint i=0; i<(uint)d; i++) p[i*d+i]=x; //operator()(i, i)=x;
  //mtype=diagMT;
}

/// sets x to be the diagonal matrix with diagonal v
template<class T> void Array<T>::setDiag(const Array<T>& v) {
  CHECK_EQ(v.nd, 1, "can only give diagonal of 1D array");
  resize(v.d0, v.d0);
  setZero();
  uint i;
  for(i=0; i<v.d0; i++) operator()(i, i)=v(i);
  //mtype=diagMT;
}

/// constructs the block matrix X=[A, B ; C, D]
template<class T> void Array<T>::setBlockMatrix(const Array<T>& A, const Array<T>& B, const Array<T>& C, const Array<T>& D) {
  CHECK(A.nd==2 && B.nd==2 && C.nd==2 && D.nd==2, "");
  CHECK(A.d0==B.d0 && A.d1==C.d1 && B.d1==D.d1 && C.d0==D.d0, "");
  resize(A.d0+C.d0, A.d1+B.d1);
  setMatrixBlock(A, 0, 0);
  setMatrixBlock(B, 0, A.d1);
  setMatrixBlock(C, A.d0, 0);
  setMatrixBlock(D, A.d0, A.d1);
}

/// constructs the block matrix X=[A; B]
template<class T> void Array<T>::setBlockMatrix(const Array<T>& A, const Array<T>& B) {
  CHECK(A.nd==2 && B.nd==2, "");
  CHECK(A.d1==B.d1, "");

  if constexpr(std::is_same_v<T, double>){
    if(isSparse(A)){
      CHECK(isSparse(B), "");
      sparse().resize(A.d0+B.d0, A.d1, 0);
    }else{
      resize(A.d0+B.d0, A.d1).setZero();
    }
  }else{
    resize(A.d0+B.d0, A.d1).setZero();
  }

  setMatrixBlock(A, 0, 0);
  setMatrixBlock(B, A.d0, 0);
}

/// constructs a vector x=[a, b]
template<class T> void Array<T>::setBlockVector(const Array<T>& a, const Array<T>& b) {

  if constexpr(std::is_same_v<T, double>){
    if(a.jac || b.jac) {
      const Array<T>& A=*a.jac;
      const Array<T>& B=*b.jac;
      if(isSparse(A)){
        CHECK(isSparse(B), "");
        J().sparse().resize(A.d0+B.d0, A.d1, 0);
      }else{
        CHECK(!isSparse(B), "");
        J().resize(A.d0+B.d0, A.d1).setZero();
      }
    }
  }
  CHECK(a.nd==1 && b.nd==1, "");
  resize(a.N+b.N);
  setVectorBlock(a, 0);   //for(i=0;i<a.N;i++) operator()(i    )=a(i);
  setVectorBlock(b, a.N); //for(i=0;i<b.N;i++) operator()(i+a.N)=b(i);
}

/// write the matrix B into 'this' matrix at location lo0, lo1
template<class T> void Array<T>::setMatrixBlock(const Array<T>& B, uint lo0, uint lo1) {
  if constexpr(std::is_same_v<T, double>){
    if(isSparse(*this)){
      sparse().add(B, lo0, lo1);
      return;
    }
  }

  CHECK(!special && !B.special, "");
  CHECK(B.nd==1 || B.nd==2, "");
  if(B.nd==2) {
    CHECK(nd==2 && lo0+B.d0<=d0 && lo1+B.d1<=d1, "");
    uint i, j;
    if(memMove) {
      for(i=0; i<B.d0; i++) memmove(p+(lo0+i)*d1+lo1, B.p+i*B.d1, B.d1*sizeT);
    } else {
      for(i=0; i<B.d0; i++) for(j=0; j<B.d1; j++) p[(lo0+i)*d1+lo1+j] = B.p[i*B.d1+j];   // operator()(lo0+i, lo1+j)=B(i, j);
    }
  } else { //N.nd==1
    CHECK(nd==2 && lo0+B.d0<=d0 && lo1+1<=d1, "");
    uint i;
    for(i=0; i<B.d0; i++) p[(lo0+i)*d1+lo1] = B.p[i];  // operator()(lo0+i, lo1+j)=B(i, j);
  }
}

/// write the vector B into 'this' vector at location lo0
template<class T> void Array<T>::setVectorBlock(const Array<T>& B, uint lo) {
  CHECK(!special && !B.special, "");
  CHECK(nd==1 && B.nd==1 && lo+B.N<=N, "");
  uint i;
  for(i=0; i<B.N; i++) elem(lo+i)=B.elem(i);
  if constexpr(std::is_same_v<T, double>){
    if(B.jac) {
      CHECK(jac && jac->d1==B.jac->d1, "Jacobian needs to be pre-sized");
      CHECK(!B.jac->jac, "NOT HANDLED YET");
      jac->setMatrixBlock(*B.jac, lo, 0);
    }
  }
}

/// sorted permutation of length \c n
template<class T> void Array<T>::setStraightPerm(int n) {
  if(n!=-1) resize(n);
  if constexpr(std::is_arithmetic_v<T>){
    for(uint i=0; i<N; i++) elem(i)=static_cast<T>(i);
  } else NIY;
}

/// reverse sorted permutation of lenth \c N
template<class T> void Array<T>::setReversePerm(int n) {
  if(n!=-1) resize(n);
  if constexpr(std::is_arithmetic_v<T>){
    for(uint i=0; i<N; i++) elem(N-1-i)=static_cast<T>(i);
  } else NIY;
}

/// permute all elements randomly
template<class T> void Array<T>::setRandomPerm(int n) {
  setStraightPerm(n);
  int j, r;
  for(j=N-1; j>=1; j--) {
    r=rndInt(j+1);
    permute(r, j);
  }
}

/// 'this' becomes a copy (not reference to!) of the 1D C array
template<class T> Array<T>& Array<T>::setCarray(const T* buffer, uint D0) {
  if(N!=D0) resize(D0);
  uint i;
  if(memMove && typeid(T)==typeid(T))
    memmove(p, buffer, sizeT*d0);
  else for(i=0; i<d0; i++) operator()(i)=(T)buffer[i];
  return *this;
}

/// 'this' becomes a copy (not reference to!) of the 2D C array
template<class T> Array<T>& Array<T>::setCarray(const T** buffer, uint D0, uint D1) {
  resize(D0, D1);
  uint i, j;
  for(i=0; i<d0; i++) {
    if(memMove && typeid(T)==typeid(T))
      memmove(p+i*d1, buffer[i], sizeT*d1);
    else for(j=0; j<d1; j++) operator()(i, j)=(T)buffer[i][j];
  }
  return *this;
}

/// make this array a reference to the array \c a
template<class T> void Array<T>::referTo(const Array<T>& a) {
  CHECK(!a.special, "");
  referTo(a.p, a.N);
  reshapeAs(a);
}

/// make this array a subarray reference to \c a
template<class T> void Array<T>::referToRange(const Array<T>& a, std::pair<int, int> I) {
  CHECK_LE(a.nd, 3, "not implemented yet");
  int i_lo=I.first, i_up=I.second-1;
  if(i_lo<0) i_lo+=a.d0;
  if(i_up<0) i_up+=a.d0;
  if(i_lo>i_up) { clear(); return; }
  CHECK((uint)i_lo<a.d0 && (uint)i_up<a.d0, "SubRange range error (" <<i_lo <<"<" <<a.d0 <<", " <<i_up <<"<" <<a.d0 <<")");

  if(a.nd==1) {
    referTo(a.p+i_lo, i_up+1-i_lo);
  } else if(a.nd==2) {
    referTo(a.p+i_lo*a.d1, (i_up+1-i_lo)*a.d1);
    nd=2;  d0=i_up+1-i_lo;  d1=a.d1;
  } else if(a.nd==3) {
    referTo(a.p+i_lo*a.d1*a.d2, (i_up+1-i_lo)*a.d1*a.d2);
    nd=3;  d0=i_up+1-i_lo;  d1=a.d1;  d2=a.d2;
  }
}

/// make this array a subarray reference to \c a
template<class T> void Array<T>::referToRange(const Array<T>& a, int i, std::pair<int, int> J) {
  CHECK(a.nd>1, "does not make sense");
  CHECK_LE(a.nd, 3, "not implemented yet");
  int j_lo=J.first, j_up=J.second-1;
  if(i<0) i+=a.d0;
  if(j_lo<0) j_lo+=a.d1;
  if(j_up<0) j_up+=a.d1;
  if(j_lo>j_up) return;
  CHECK((uint)i<a.d0, "SubRange range error (" <<i <<"<" <<a.d0 <<")");
  CHECK((uint)j_lo<a.d1 && (uint)j_up<a.d1, "SubRange range error (" <<j_lo <<"<" <<a.d1 <<", " <<j_up <<"<" <<a.d1 <<")");

  if(a.nd==2) {
    referTo(&a(i, j_lo), (j_up+1-j_lo));
  } else if(a.nd==3) {
    referTo(&a(i, j_lo, 0), (j_up+1-j_lo)*a.d2);
    nd=2;  d0=j_up+1-j_lo;  d1=a.d2;
  }
}

/// make this array a subarray reference to \c a
template<class T> void Array<T>::referToRange(const Array<T>& a, int i, int j, std::pair<int, int> K) {
  CHECK(a.nd>2, "does not make sense");
  CHECK_LE(a.nd, 3, "not implemented yet");
  int k_lo=K.first, k_up=K.second-1;
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
template<class T> void Array<T>::referToDim(const Array<T>& a, int i) {
  CHECK(a.nd>1, "can't create subarray of array less than 2 dimensions");
  CHECK(!special, "can't refer to row of sparse matrix");
  if(i<0) i+=a.d0;
  CHECK(i>=0 && i<(int)a.d0, "SubDim range error (" <<i <<"<" <<a.d0 <<")");

  if(a.nd==2) {
    referTo(a.p+i*a.d1, a.d1);
  } else if(a.nd==3) {
    referTo(a.p+i*a.d1*a.d2, a.d1*a.d2);
    nd=2;  d0=a.d1;  d1=a.d2;
  } else if(a.nd>3) {
    uint n=a.N/a.d0;
    referTo(a.p+i*n, n);
    nd=a.nd-1;  d0=a.d1;  d1=a.d2;  d2=a.d[3];
    if(nd>3) { d=new uint[nd];  memmove(d, a.d+1, nd*sizeof(uint)); }
  }
}

/// make this array a subarray reference to \c a
template<class T> void Array<T>::referToDim(const Array<T>& a, uint i, uint j) {
  CHECK(a.nd>2, "can't create subsubarray of array less than 3 dimensions");
  CHECK(i<a.d0 && j<a.d1, "SubDim range error (" <<i <<"<" <<a.d0 <<", " <<j <<"<" <<a.d1 <<")");

  if(a.nd==3) {
    referTo(&a(i, j, 0), a.d2);
  } else {
    NIY // TODO
  }
}

/// make this array a subarray reference to \c a
template<class T> void Array<T>::referToDim(const Array<T>& a, uint i, uint j, uint k) {
  CHECK(a.nd>3, "can't create subsubarray of array less than 3 dimensions");
  CHECK(i<a.d0 && j<a.d1 && k<a.d2, "SubDim range error (" <<i <<"<" <<a.d0 <<", " <<j <<"<" <<a.d1 <<", " <<k <<"<" <<a.d2 << ")");

  if(a.nd==4) {
    referTo(&a(i, j, k), a.d[3]);
  } else if(a.nd==5) {
    NIY;
//    nd=2; d0=a.d[3]; d1=a.d[4]; d2=0; N=d0*d1;
  } else if(a.nd>5) {
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
template<class T> void Array<T>::takeOver(Array<T>& a) {
  freeMEM();
  memMove=a.memMove;
  N=a.N; nd=a.nd; d0=a.d0; d1=a.d1; d2=a.d2;
  p=a.p; M=a.M;
  special=a.special;
#if 0 //a remains reference on this
  a.isReference=true;
  a.M=0;
#else //a is cleared
  a.p=NULL;
  a.M=a.N=a.nd=a.d0=a.d1=a.d2=0;
  if(a.d && a.d!=&a.d0) { delete[] a.d; a.d=NULL; }
  a.special=0;
  a.isReference=false;
#endif
}

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
template<class T> Array<T>& Array<T>::sort(ElemCompare comp) {
  std::sort(p, p+N, comp);
  return *this;
}

/// check whether list is sorted
template<class T> bool Array<T>::isSorted(ElemCompare comp) const {
  uint i;
  for(i=0; i<N-1; i++) {
    if(!comp(elem(i), elem(i+1))) return false;
  }
  return true;
}

/// fast find method in a sorted array, returns index where x would fit into array
template<class T> uint Array<T>::rankInSorted(const T& x, ElemCompare comp, bool rankAfterIfEqual) const {
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
template<class T> int Array<T>::findValueInSorted(const T& x, ElemCompare comp) const {
  uint cand_pos = rankInSorted(x, comp);
  if(cand_pos == N) return -1;
  else if(elem(cand_pos) != x) return -1;
  else return cand_pos;
}

/// fast insert method in a sorted array, the array remains sorted
template<class T> uint Array<T>::insertInSorted(const T& x, ElemCompare comp, bool insertAfterIfEqual) {
  uint cand_pos = rankInSorted(x, comp, insertAfterIfEqual);
  insert(cand_pos, x);
//  if(true){
//      for(uint i=0;i<N-1;i++) CHECK(comp(elem(i),elem(i+1)), "this is not sorted!");
//  }
  return cand_pos;
}

template<class T> uint Array<T>::setAppendInSorted(const T& x, ElemCompare comp) {
  CHECK(memMove, "");
  uint cand_pos = rankInSorted(x, comp);
  if(cand_pos<N && elem(cand_pos)==x) return cand_pos;
  if(cand_pos>0 && elem(cand_pos-1)==x) return cand_pos-1;
  insert(cand_pos, x);
  return cand_pos;
}

/// fast remove method in a sorted array, the array remains sorted
template<class T> void Array<T>::removeValueInSorted(const T& x, ElemCompare comp) {
  uint i=findValueInSorted(x, comp);
  CHECK_EQ(elem(i), x, "value not found");
  remove(i);
}

template<class T> Array<T>& Array<T>::removeDoublesInSorted() {
  for(int i=N-1; i>0; i--) if(elem(i)==elem(i-1)) remove(i);
  return *this;
}

//***** permutations

/// permute the elements \c i and \c j
template<class T> void Array<T>::permute(uint i, uint j) { T x=p[i]; p[i]=p[j]; p[j]=x; }

/// permute the entries according to the given permutation
template<class T> void Array<T>::permute(const Array<uint>& permutation) {
  CHECK_LE(permutation.N, N, "array smaller than permutation (" <<N <<"<" <<permutation.N <<")");
  Array<T> b=(*this);
  for(uint i=0; i<N; i++) elem(i)=b.elem(permutation(i));
}

/// permute the rows (operator[]) according to the given permutation
template<class T> void Array<T>::permuteRows(const Array<uint>& permutation) {
  CHECK_LE(permutation.N, d0, "array smaller than permutation ("<<N<<"<"<<permutation.N<<")");
  Array<T> b=(*this);
  for(uint i=0; i<d0; i++) operator[](i)=b[permutation(i)];
}

/// apply the given 'permutation' on 'this'
template<class T> void Array<T>::permuteInv(const Array<uint>& permutation) {
  CHECK_LE(permutation.N, N, "array smaller than permutation (" <<N <<"<" <<permutation.N <<")");
  Array<T> b=(*this);
  for(uint i=0; i<N; i++) elem(permutation(i))=b.elem(i);
}

/// permute the rows (operator[]) according to the given permutation
template<class T> void Array<T>::permuteRowsInv(const Array<uint>& permutation) {
  CHECK_LE(permutation.N, d0, "array smaller than permutation ("<<N<<"<"<<permutation.N<<")");
  Array<T> b=(*this);
  for(uint i=0; i<d0; i++) operator[](permutation(i))=b[i];
}

/// randomly permute all entries of 'this'
template<class T> void Array<T>::permuteRandomly() {
  uintA perm;
  perm.setRandomPerm(N);
  permute(perm);
}

/// push all elements forward or backward (depending on sign of offset)
template<class T> void Array<T>::shift(int offset, bool wrapAround) {
  Array<T> tmp;
  CHECK(memMove, "pushing only works with memMove enabled");
  uint m=offset>0?offset:-offset;
  if(!m) return;
  if(m==N && wrapAround) return; //nothing to be done;
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

//==================================================================================

/// return fraction of non-zeros in the array
template<class T> double Array<T>::sparsity() {
  uint i, m=0;
  for(i=0; i<N; i++) if(elem(i)) m++;
  return ((double)m)/N;
}

//==================================================================================

/** @brief prototype for operator<<, writes the array by separating elements with ELEMSEP, separating rows with LINESEP, using BRACKETS[0] and BRACKETS[1] to brace the data, optionally writs a dimensionality tag before the data (see below), and optinally in binary format */
template<class T> void Array<T>::write(std::ostream& os, const char* ELEMSEP, const char* LINESEP, const char* BRACKETS, bool dimTag, bool binary) const {

  if constexpr(std::is_same_v<T, double>){
    if(special){
      special_write(os, *this);
      if(jac) os <<" -- JACOBIAN:\n" <<*jac <<endl;
      return;
    }
  }else{
    CHECK(!special, "");
  }

  CHECK(!binary || memMove, "binary write works only for memMoveable data");
  uint i, j, k;
  if(!ELEMSEP) ELEMSEP=arrayElemsep;
  if(!LINESEP) LINESEP=arrayLinesep;
  if(!BRACKETS) BRACKETS=arrayBrackets;

  if(binary) {
    writeJson(os);
#if 0
    writeDim(os);
    os <<endl;
    os.put(0);
    os.write((char*)p, sizeT*N);
    os.put(0);
    os <<endl;
#endif
  } else {
    if(BRACKETS[0]) os <<BRACKETS[0];
    if(dimTag || nd>3) { os <<' '; writeDim(os); if(nd==2) os <<'\n'; else os <<' '; }
    if(nd>=3) os <<'\n';
    if(nd==0 && N==1) {
      os <<(const T&)elem();
    }
    if(nd==1) {
      for(i=0; i<N; i++) os <<(i?ELEMSEP:"")  <<operator()(i);
    }
    if(nd==2) for(j=0; j<d0; j++) {
        if(j) os <<LINESEP;
        for(i=0; i<d1; i++) os <<(i?ELEMSEP:"") <<operator()(j, i);
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

  if constexpr(std::is_same_v<T, double>){
    if(jac) os <<" -- JACOBIAN:\n" <<*jac <<endl;
  }
}

/** @brief prototype for operator>>, if there is a dimensionality tag: fast reading of ascii (if there is brackets[]) or binary (if there is \\0\\0 brackets) data; otherwise slow ascii read */
template<class T> Array<T>& Array<T>::read(std::istream& is) {
  bool expectBracket=false;

#define PARSERR(x) HALT("Error in parsing Array of type '" <<typeid(T).name() <<"' (line=" <<lineCount <<"):\n" <<x)

  char c=peerNextChar(is, " \n\r\t", true);
  if(c=='[') {
    is >>PARSE("[");
    expectBracket=true;
    c=peerNextChar(is, " \n\r\t", true);
  }

  if(c=='<') { //fast pre-sized read
    readDim(is);
    c=peerNextChar(is, " \n\r\t", true);
    if(c==0) {  //binary read
      c=is.get();  if(c!=0) PARSERR("couldn't read \0 before binary data block :-(");
      is.read((char*)p, sizeT*N);
      if(is.fail()) PARSERR("could not binary data");
      c=is.get();  if(c!=0) PARSERR("couldn't read \0 after binary data block :-(");
    } else { //fast ascii read
      for(uint i=0; i<N; i++) {
        if(is.fail()) PARSERR("could not read " <<i <<"-th element of an array");
        if constexpr(!std::is_pointer_v<T>){
          is >>p[i];
        } else NIY;
      }
    }
    if(expectBracket) {
      is >>PARSE("]");
      if(is.fail()) PARSERR("could not read array end tag");
    }
  } else { //slow ascii read (inferring size from formatting)
    uint i=0;
    uint d=0;
    T x;
    for(;;) {
      skip(is, " ,\r\t", NULL, true);
      is.get(c);
      if(is.eof()) {
        if(expectBracket) LOG(-1) <<"closing bracket is missing";
        is.clear();
        break;
      }
      if(expectBracket && c==']') { is.clear(); break; }
      if(c==';' || c=='\n') {  //set an array width
        if(!d) d=i; else if(d && i%d) PARSERR("mis-structured array in row " <<i/d);
        continue;
      }
      if(c!=',') is.putback(c);
      if constexpr(!std::is_pointer_v<T>){
        is >>x;
      }else NIY;
      if(!is.good()) {
        if(!expectBracket) is.clear(); //ok
        else PARSERR("failed reading ending bracket ]");
        break;
      }
      if(i>=N) resizeCopy(i+1000/sizeT);
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

  return *this;
}

/// write data with a name tag (convenient to write multiple data arrays into one file)
template<class T> void Array<T>::writeTagged(std::ostream& os, const char* tag, bool binary) const {
  os <<tag <<' ';
  write(os, " ", "\n ", "[]", true, binary);
  os <<endl;
}

/// read data with a name tag (convenient to read multiple data arrays from one file)
template<class T> bool Array<T>::readTagged(std::istream& is, const char* tag) {
  if(tag) parse(is, tag, false);
  skip(is, " :\r\t", NULL, true);
  read(is);
  return true;
}

/// write a dimensionality tag of format <d0 d1 d2 ...>
template<class T> void Array<T>::writeDim(std::ostream& os) const {
  uint i;
  os <<'<';
  os <<typeid(T).name();
  for(i=0; i<nd; i++) os <<' ' <<dim(i);
  os <<'>';
}

/// read a dimensionality tag of format <d0 d1 d2 ...> and resize this array accordingly
template<class T> void Array<T>::readDim(std::istream& is) {
  char c;
  uint ND, dim[10];
  is >>PARSE("<");
  is.get(c);
  if(c==typeid(T).name()[0] && 0==typeid(T).name()[1]) { //c is a type indicator - swallow it
    is.get(c); //eat c
  }
  if(c=='>') {
    clear();
    return;
  } else {
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

template<class T> void Array<T>::writeBase64(std::ostream& os) const {
  int code_len = b64_codeLen(N*sizeT);
  char* code = (char*) malloc(code_len+1);
  b64_encode(code, code_len, (const char*)p, N*sizeT);
  os.write(code, code_len);
  free(code);
}

template<class T> void Array<T>::readBase64(std::istream& is) {
  int code_len = b64_codeLen(N*sizeT);
  char* code = (char*) malloc(code_len+1);
  is.read(code, code_len);
  code[code_len] = '\0';
  if(is.fail()) LOG(-2) <<"could not base64 data";
  b64_decode((char*)p, N*sizeT, code, code_len);
  free(code);
}

/// write a json dict
template<class T> void Array<T>::writeJson(std::ostream& os) const {
  os <<"[ \"" <<rai::atomicTypeidName(typeid(T)) <<"\", [";
  for(uint i=0; i<nd; i++) { os <<dim(i); if(i+1<nd) os <<", "; }
  os <<"], \"";
  writeBase64(os);
  os <<"\" ]";
}

/// read a json dict
template<class T> void Array<T>::readJson(std::istream& is, bool skipType) {
  if(!skipType) {
    parse(is, "[", false);
    { char c=getNextChar(is, " \n\r\t", true); if(c!='"') is.putback(c); }
    parse(is, rai::atomicTypeidName(typeid(T)), false);
    { char c=getNextChar(is, " \n\r\t", true); if(c!='"') is.putback(c); }
  }
  parse(is, ",", false);
  parse(is, "[", false);
  {
    char c;
    uint ND, dim[10];
    is.get(c);
    if(c==']') {
      clear();
      return;
    } else {
      is.putback(c);
    }
    for(ND=0;; ND++) {
      is >>dim[ND];
      is.get(c);
      if(c==']') break;
      CHECK_EQ(c, ',', "error in reading dimensionality");
    }
    resize(ND+1, dim);
  }
  parse(is, ",", false);
  parse(is, "\"", false);
  readBase64(is);
  parse(is, "\"", false);
  parse(is, "]", false);
}

template<class T> uint Array<T>::serial_size() {
  return 6+6*sizeof(uint)+N*sizeT;
}

template<class T> uint Array<T>::serial_encode(char* data, uint data_size) {
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

template<class T> uint Array<T>::serial_decode(char* data, uint data_size) {
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

//===============================================================================

#ifdef RAI_CLANG
#  pragma clang diagnostic pop
#endif
#pragma GCC diagnostic pop

template<class T> void writeConsecutiveConstant(std::ostream& os, const Array<T>& x) {
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

/// contatenation of two arrays
template<class T> Array<T> operator, (const Array<T>& y, const Array<T>& z) { Array<T> x(y); x.append(z); return x; }

/// calls Array<T>::read
template<class T> std::istream& operator>>(std::istream& is, Array<T>& x) { x.read(is); return is; }

/// calls Array<T>::write
template<class T> std::ostream& operator<<(std::ostream& os, const Array<T>& x) { x.write(os); return os; }

/// equal in size and all elements
template<class T> bool operator==(const Array<T>& v, const Array<T>& w) {
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

//core for matrix-matrix (elem-wise) update
#define UpdateOperator_MM( op ) \
if constexpr(std::is_same_v<T, double>){ \
      if(isNoArr(x)){ return; } \
      if(isSparseMatrix(x) && isSparseMatrix(y)){ x.sparse() op y.sparse(); return; }  \
      if(isRowShifted(x) && isRowShifted(y)){ x.rowShifted() op y.rowShifted(); return; }  \
} \
    CHECK(!x.special, "");  \
    CHECK(!y.special, "");  \
    CHECK_EQ(x.N, y.N, "update operator on different array dimensions (" <<x.N <<", " <<y.N <<")"); \
    T *xp=x.p, *xstop=xp+x.N; \
    const T *yp=y.p; \
    for(; xp!=xstop; xp++, yp++) *xp op *yp;

//core for matrix-scalar update
#define UpdateOperator_MS( op ) \
if constexpr(std::is_same_v<T, double>){ \
      if(isNoArr(x)){ return; } \
      if(isSparseMatrix(x)){ x.sparse() op y; return; }  \
      if(isRowShifted(x)){ x.rowShifted() op y; return; }  \
} \
    CHECK(!x.special, "");  \
    T *xp=x.p, *xstop=xp+x.N; \
    for(; xp!=xstop; xp++) *xp op y;


template<class T> void operator+=(Array<T>& x, const Array<T>& y) {
  UpdateOperator_MM(+=);
  if constexpr(std::is_same_v<T, double>){
    if(y.jac) {
      if(x.jac) *x.jac += *y.jac;
      else x.J() = *y.jac;
    }
  }
  // CHECK_EQ(x.N, y.N, "update operator on different array dimensions (" <<x.N <<", " <<y.N <<")");
  // T* xp=x.p, *xstop=xp+x.N;
  // const T* yp=y.p;
  // for(; xp!=xstop; xp++, yp++) *xp += *yp;
}
template<class T> void operator+=(Array<T>& x, const T& y) {
  UpdateOperator_MS(+=);
  // T* xp=x.p, *xstop=xp+x.N;
  // for(; xp!=xstop; xp++) *xp += y;
}
template<class T> void operator-=(Array<T>& x, const Array<T>& y) {
  UpdateOperator_MM(-=);
  if constexpr(std::is_same_v<T, double>){
    if(y.jac) {
      if(x.jac) *x.jac -= *y.jac;
      else x.J() = -(*y.jac);
    }
  }
  // CHECK_EQ(x.N, y.N, "update operator on different array dimensions (" <<x.N <<", " <<y.N <<")");
  // T* xp=x.p, *xstop=xp+x.N;
  // const T* yp=y.p;
  // for(; xp!=xstop; xp++, yp++) *xp -= *yp;
}
template<class T> void operator-=(Array<T>& x, const T& y) {
  UpdateOperator_MS(-=);
  // T* xp=x.p, *xstop=xp+x.N;
  // for(; xp!=xstop; xp++) *xp -= y;
}
template<class T> void operator*=(Array<T>& x, const T& y) {
  if constexpr(std::is_same_v<T, double>){
    if(x.jac) *x.jac *= y;
  }
  UpdateOperator_MS(*=);
  // T* xp=x.p, *xstop=xp+x.N;
  // for(; xp!=xstop; xp++) *xp *= y;
}

#undef UpdateOperator_MM
#undef UpdateOperator_MS

//===========================================================================

/** @brief return a `dim'-dimensional grid with `steps' intervals
  filling the range [lo, hi] in each dimension. Note: returned array is
  `flat', rather than grid-shaped. */
template<class T> Array<T> grid(uint dim, T lo, T hi, uint steps) {
  Array<T> x;
  CHECK(steps, "steps needs to be >0");
  uint i, j, k;
  if(dim==1) {
    x.resize(steps+1, 1);
    for(i=0; i<x.d0; i++) x.elem(i)=lo+(hi-lo)*i/steps;
    return x;
  }
  if(dim==2) {
    x.resize(steps+1, steps+1, 2);
    for(i=0; i<x.d0; i++) for(j=0; j<x.d1; j++) {
        x(i, j, 0)=lo+(hi-lo)*i/steps;
        x(i, j, 1)=lo+(hi-lo)*j/steps;
      }
    x.reshape(x.d0*x.d1, 2);
    return x;
  }
  if(dim==3) {
    x.resize(uintA{steps+1, steps+1, steps+1, 3});
    T dx = (hi-lo)/steps;
    for(i=0; i<x.d0; i++) for(j=0; j<x.d1; j++) {
        T* p = &x.elem(uintA{i, j, 0, 0});
        for(k=0; k<x.d2; k++) {
          *(p++) = lo+dx*i;
          *(p++) = lo+dx*j;
          *(p++) = lo+dx*k;
          //        elem(uintA{i, j, k, 0}) = lo+dx*i;
          //        elem(uintA{i, j, k, 1}) = lo+dx*j;
          //        elem(uintA{i, j, k, 2}) = lo+dx*k;
        }
      }
    x.reshape(x.d0*x.d1*x.d2, 3);
    return x;
  }
  NIY;
  return x;
}

} //namespace
