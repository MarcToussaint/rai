/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIdouble License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "array.h"
#include "util.h"

#include <cmath>
#include <algorithm>
#include <sstream>

#define maxRank 30
/** @brief if flexiMem is true (which is default!) the resize method will
  (1) at the first call allocate the exact amount of memory, (2)
  at further calls of increasing memory allocate twice the memory
  needed or (3) at further calls of decreasing memory only free
  the memory if the new size is smaller than a fourth */
#define ARRAY_flexiMem true

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wclass-memaccess"

//#ifdef RAI_MSVC
//#define RAI_NO_VEC_IMPL
//#endif

//===========================================================================
//
// Array class
//

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

arr& arr::operator=(std::initializer_list<double> values) {
  resize(values.size());
  uint i=0;
  for(const double& t : values) elem(i++)=t;
  return *this;
}

/// set all elements to value \c v
arr& arr::operator=(const double& v) {
  if(N){
    double *x=p, *xstop=p+N;
    for(; x!=xstop; x++) *x = v;
  }
  return *this;
}

/// copy operator
arr& arr::operator=(const Array<double>& a) {
  CHECK(this!=&a, "never do this!!!");
  resizeAs(a);
  if(memMove) memmove(p, a.p, sizeT*N);
  else for(uint i=0; i<N; i++) p[i]=a.p[i];
  if(special) { delete special; special=0; }
  return *this;
}

/// copy operator
arr& arr::operator=(const arr& a) {
  operator=((const Array<double>&)a);
#if 0
  CHECK(this!=&a, "never do this!!!");
  //if(a.temp){ takeOver(*((arr*)&a)); return *this; }
  resizeAs(a);
  if(memMove) memmove(p, a.p, sizeT*N);
  else for(uint i=0; i<N; i++) p[i]=a.p[i];
  if(special) { delete special; special=0; }
#endif
  if(isSpecial(a)) {
    if(isRowShifted(a)) {
      special = new RowShifted(*((arr*)this), *dynamic_cast<RowShifted*>(a.special));
    } else if(isSparseVector(a)) {
      special = new SparseVector(*((arr*)this), *dynamic_cast<SparseVector*>(a.special));
    } else if(isSparseMatrix(a)) {
      special = new SparseMatrix(*((arr*)this), *dynamic_cast<SparseMatrix*>(a.special));
    } else if(isNoArr(a)){
      setNoArr();
    } else NIY;
  }
  if(a.jac) jac = make_unique<arr>(*a.jac);
  return *this;
}


/// access that invariantly works for sparse and non-sparse matrices
double& arr::elem(int i, int j) {
  if(i<0) i += this->d0;
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

/// range reference access
arr arr::operator()(int i, int j, std::initializer_list<int> K) const {
  arr z;
  if(K.size()==2) z.referToRange(*this, i, j, K.begin()[0], K.begin()[1]);
  else if(K.size()==0) z.referToDim(*this, i, j);
  else if(K.size()==1) z.referToDim(*this, i, j, K.begin()[0]);
  else HALT("range list needs 0,1, or 2 entries exactly");
  return z;
}

/** @brief a sub array of a 1D Array (corresponds to matlab [i:I]); when
  the upper limit I is -1, it is replaced by the max limit (like
  [i:]) */
arr arr::sub(int i, int I) const {
  CHECK_EQ(nd, 1, "1D range error ");
  arr x;
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
arr arr::sub(int i, int I, int j, int J) const {
  CHECK_EQ(nd, 2, "2D range error ");
  arr x;
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
arr arr::sub(int i, int I, int j, int J, int k, int K) const {
  CHECK_EQ(nd, 3, "3D range error ");
  arr x;
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
arr arr::sub(int i, int I, Array<uint> cols) const {
  CHECK_EQ(nd, 2, "2D range error ");
  arr x;
  if(i<0) i+=d0;
  if(I<0) I+=d0;
  CHECK(i>=0 && I>=0 && i<=I, "lower limit higher than upper!");
  x.resize(I-i+1, cols.N);
  for(int ii=i; ii<=I; ii++) for(int l=0; l<(int)cols.N; l++) x(ii-i, l)=operator()(ii, cols(l));
  return x;
}

arr arr::sub(Array<uint> elems) const {
  arr x;
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

void rai::ArrayDouble::setMatrixBlock(const rai::ArrayDouble& B, uint lo0, uint lo1){
  if(isSparse(*this))
    sparse().add(B, lo0, lo1);
  else
    Array<double>::setMatrixBlock(B, lo0, lo1);
}

void rai::ArrayDouble::setVectorBlock(const rai::ArrayDouble& B, uint lo){
  Array<double>::setVectorBlock(B, lo);
  if(B.jac){
    CHECK(jac && jac->d1==B.jac->d1, "Jacobian needs to be pre-sized");
    CHECK(!B.jac->jac, "NOT HANDLED YET");
    jac->setMatrixBlock(*B.jac, lo, 0);
  }
}

void rai::ArrayDouble::setBlockVector(const rai::ArrayDouble& a, const rai::ArrayDouble& b) {
  CHECK(a.nd==1 && b.nd==1, "");
  resize(a.N+b.N);
  setVectorBlock(a.noJ(), 0);
  setVectorBlock(b.noJ(), a.N);
  if(a.jac || b.jac){
    if(a.jac && b.jac){
      J().setBlockMatrix(*a.jac, *b.jac);
    } else NIY;
  }
}

void rai::ArrayDouble::setBlockMatrix(const rai::ArrayDouble& A, const rai::ArrayDouble& B) {
    if(!A.special){
        Array<double>::setBlockMatrix(A, B);
    }else if(isSparse(A)){
        CHECK(isSparse(B), "");
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

void rai::ArrayDouble::write(std::ostream& os, const char* ELEMSEP, const char* LINESEP, const char* BRACKETS, bool dimTag, bool binary) const {
  if(!special){
    Array<double>::write(os, ELEMSEP, LINESEP, BRACKETS, dimTag, binary);
  } else if(isSparseVector(*this)) {
    intA& elems = dynamic_cast<SparseVector*>(special)->elems;
    for(uint i=0; i<N; i++) os <<"( " <<elems(i) <<" ) " <<elem(i) <<endl;
  } else if(isSparseMatrix(*this)) {
    intA& elems = dynamic_cast<SparseMatrix*>(special)->elems;
    for(uint i=0; i<N; i++) os <<'(' <<elems[i] <<") " <<elem(i) <<endl;
  }
  if(jac){
    os <<" -- JACOBIAN:\n" <<*jac <<endl;
  }
}

/// x = y^T
void op_transpose(arr& x, const arr& y) {
  CHECK(&x!=&y, "can't transpose matrix into itself");
  CHECK_LE(y.nd, 3, "can only transpose up to 3D arrays");
  if(y.nd==3) {
    uint i, j, k, d0=y.d2, d1=y.d1, d2=y.d0;
    x.resize(d0, d1, d2);
    for(i=0; i<d0; i++) for(j=0; j<d1; j++) for(k=0; k<d2; k++)
          x(i, j, k) = y(k, j, i);
    //x.p[(i*d1+j)*d2+k]=y.p[(k*d1+j)*d0+i];
    if(y.jac){ NIY }
    return;
  }
  if(y.nd==2) {
    if(isSparseMatrix(y)) {
      x = y;
      x.sparse().transpose();
      if(y.jac){ NIY }
      return;
    }
    x.resize(y.d1, y.d0);
//    for(i=0; i<d0; i++)
    double* xp=x.p;
    for(uint i=0; i<x.d0; i++) {
      double* yp=y.p+i, *xstop=xp+x.d1;
      uint ystep=y.d1;
      for(; xp!=xstop; xp++, yp+=ystep) *xp = *yp;
//      for(j=0; j<d1; j++) x.p[i*d1+j]=y.p[j*d0+i];
    }
    if(y.jac){ NIY }
    return;
  }
  if(y.nd==1) {
    x=y;
    x.reshape(1, y.N);
    if(y.jac){
      // don't do anything
    }
    return;
  }
  HALT("transpose not implemented for this dims");
}

/// returns the diagonal x = diag(y) (the diagonal-vector of the symmetric 2D matrix y)
arr getDiag(const arr& y) {
  CHECK(y.nd==2 && y.d0==y.d1, "can only give diagonal of symmetric 2D matrix");
  arr x;
  x.resize(y.d0);
  uint i;
  for(i=0; i<x.d0; i++) x(i)=y(i, i);
  return x;
}

/// inverse of a 2d matrix
arr inverse2d(const arr& A) {
  arr Ainv(2, 2);
  Ainv(0, 0)=A(1, 1); Ainv(1, 1)=A(0, 0); Ainv(0, 1)=-A(0, 1); Ainv(1, 0)=-A(1, 0);
  Ainv/=(double)(A(0, 0)*A(1, 1)-A(0, 1)*A(1, 0));
  return Ainv;
}

/// sets x to be the diagonal matrix with diagonal v
arr skew(const arr& v) {
  arr y;
  CHECK(v.nd==1 && v.N==3, "can only give diagonal of 1D array");
  y.resize(3, 3);
  y.p[0]=   0.; y.p[1]=-v(2); y.p[2]= v(1);
  y.p[3]= v(2); y.p[4]=   0.; y.p[5]=-v(0);
  y.p[6]=-v(1); y.p[7]= v(0); y.p[8]=   0.;
  return y;
}

/// check for Nans in the array (checks x.elem(i)==x.elem(i) for all elements)
void checkNan(const arr& x) {
  for(uint i=0; i<x.N; i++) {
    //CHECK(x.elem(i)!=NAN, "found a NaN" <<x.elem(i) <<'[' <<i <<']');
    CHECK_EQ(x.elem(i), x.elem(i), "inconsistent number: " <<x.elem(i) <<'[' <<i <<']');
  }
}

arr replicate(const arr& A, uint d0) {
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
arr integral(const arr& x) {
  if(x.nd==1) {
    double s(0);
    arr y(x.N);
    for(uint i=0; i<x.N; i++) { s+=x.elem(i); y.elem(i)=s; }
    return y;
  }
  if(x.nd==2) {
    arr y = x;
    //first pass: sum rows
    for(uint i=0; i<y.d0; i++) for(uint j=1; j<y.d1; j++) y(i,j) += y(i,j-1);
    //first pass: sum columns
    for(uint j=0; j<y.d1; j++) for(uint i=1; i<y.d0; i++) y(i,j) += y(i-1,j);
    return y;
  }
  if(x.nd==3) {
    arr y = x;
    for(uint i=1; i<y.d0; i++) for(uint j=0; j<y.d1; j++) for(uint k=0; k<y.d2; k++) y(i,j,k) += y(i-1,j  ,k  );
    for(uint i=0; i<y.d0; i++) for(uint j=1; j<y.d1; j++) for(uint k=0; k<y.d2; k++) y(i,j,k) += y(i  ,j-1,k  );
    for(uint i=0; i<y.d0; i++) for(uint j=0; j<y.d1; j++) for(uint k=1; k<y.d2; k++) y(i,j,k) += y(i  ,j  ,k-1);
    return y;
  }
  NIY;
  return arr();
}

/// return the integral image, or vector
arr differencing(const arr& x, uint w) {
  if(x.nd==1) {
    arr y(x.N);
    if(x.N) y.elem(0) = x.elem(0);
    for(uint i=1; i<x.N; i++) { y.elem(i)=x.elem(i)-x.elem(i-1); }
    return y;
  }
  if(x.nd==2) {
    arr y = x;
    for(uint i=y.d0; i--;) for(uint j=y.d1; j--;){
      double& v = y(i,j);
      if(i>=w) v -= y(i-w,j);
      if(j>=w) v -= y(i,j-w);
      if(i>=w && j>=w) v += y(i-w,j-w);
    }
    return y;
  }
  if(x.nd==3) {
    arr y = x;
    for(uint i=y.d0; i--;) for(uint j=y.d1; j--;) for(uint k=y.d2; k--;){
      double& v = y(i,j,k);
//      if(i<w || j<w || k<w) continue;
      if(i>=w) v -= y(i-w,j,k);
      if(j>=w) v -= y(i,j-w,k);
      if(k>=w) v -= y(i,j,k-w);
      if(i>=w && j>=w) v += y(i-w,j-w,k);
      if(i>=w && k>=w) v += y(i-w,j,k-w);
      if(j>=w && k>=w) v += y(i,j-w,k-w);
      if(i>=w && j>=w && k>=w) v -= y(i-w,j-w,k-w);

      uint scale=1;
      if(i>=w) scale *= w; else scale *= i+1;
      if(j>=w) scale *= w; else scale *= j+1;
      if(k>=w) scale *= w; else scale *= k+1;
      v /= (double)scale;
    }
    return y;
  }
  NIY;
  return arr();
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
double entropy(const arr& v) {
  double t(0);
  for(uint i=v.N; i--;) if(v.p[i]) t-=(double)(v.p[i]*std::log((double)v.p[i]));
  return (double)(t/RAI_LN2);
}

/// v = v / sum(v)
double normalizeDist(arr& v) {
  double Z=sum(v);
  if(Z>1e-100) v/=Z; else v=(double)1./(double)v.N;
  return Z;
}

/// v = v / sum(v)
void makeConditional(arr& P) {
  RAI_MSG("makeConditional: don't use this anymore because it normalizes over the second index!!!, rather use tensorCondNormalize and condition on _later_ indices");
  CHECK_EQ(P.nd, 2, "");
  uint i, j;
  double pi;
  for(i=0; i<P.d0; i++) {
    pi=(double)0;
    for(j=0; j<P.d1; j++) pi+=P(i, j);
    for(j=0; j<P.d1; j++) P(i, j) /= pi;
  }
}

//inline uintA uintA{uint i, uint j, uint k, uint l}{                      uintA z(4); z(0)=i; z(1)=j; z(2)=k; z(3)=l; return z; }

/// check whether this is a distribution over the first index w.r.t. the later indices
void checkNormalization(arr& v, double tol) {
  double p;
  uint i, j, k, l;
  switch(v.nd) {
    case 1:
      for(p=0, i=0; i<v.d0; i++) p+=v(i);
      CHECK(std::fabs(1.-p)<tol, "distribution is not normalized: " <<v);
      break;
    case 2:
      for(j=0; j<v.d1; j++) {
        for(p=0, i=0; i<v.d0; i++) p+=v(i, j);
        CHECK(std::fabs(1.-p)<tol, "distribution is not normalized: " <<v);
      }
      break;
    case 3:
      for(j=0; j<v.d1; j++) for(k=0; k<v.d2; k++) {
          for(p=0, i=0; i<v.d0; i++) p+=v(i, j, k);
          CHECK(std::fabs(1.-p)<tol, "distribution is not normalized: " <<v);
        }
      break;
    case 4:
      for(j=0; j<v.d1; j++) for(k=0; k<v.d2; k++) {
          for(l=0; l<v.N/(v.d0*v.d1*v.d2); l++) {
            for(p=0, i=0; i<v.d0; i++) p+=v.elem(uintA{i, j, k, l});
            CHECK(std::fabs(1.-p)<tol, "distribution is not normalized: " <<v <<" " <<p);
          }
        }
      break;
    default:
      CHECK(false, "Not implemented yet");
      break;
  }
}

void eliminate(arr& x, const arr& y, uint d) {
  CHECK_EQ(y.nd, 2, "only implemented for 2D yet");
  uint i, j;
  if(d==1) {
    x.resize(y.d0); x=(double)0;
    for(i=0; i<y.d0; i++) for(j=0; j<y.d1; j++) x(i)+=y(i, j);
  }
  if(d==0) {
    x.resize(y.d1); x=(double)0;
    for(i=0; i<y.d0; i++) for(j=0; j<y.d1; j++) x(j)+=y(i, j);
  }
}

void eliminate(arr& x, const arr& y, uint d, uint e) {
  CHECK_EQ(y.nd, 3, "only implemented for 3D yet");
  uint i, j, k;
  if(d==1 && e==2) {
    x.resize(y.d0); x=(double)0;
    for(i=0; i<y.d0; i++) for(j=0; j<y.d1; j++) for(k=0; k<y.d2; k++) x(i)+=y(i, j, k);
  }
  if(d==0 && e==2) {
    x.resize(y.d1); x=(double)0;
    for(i=0; i<y.d0; i++) for(j=0; j<y.d1; j++) for(k=0; k<y.d2; k++) x(j)+=y(i, j, k);
  }
  if(d==0 && e==1) {
    x.resize(y.d2); x=(double)0;
    for(i=0; i<y.d0; i++) for(j=0; j<y.d1; j++) for(k=0; k<y.d2; k++) x(k)+=y(i, j, k);
  }
}

// Eliminates one-dimension, d, from a 3D-tensor, y, and puts the result in x.
void eliminatePartial(arr& x, const arr& y, uint d) {
  CHECK_EQ(y.nd, 3, "only implemented for 3D yet");
  uint i, j, k;
  if(d==2) {
    x.resize(y.d0, y.d1); x=(double)0;
    for(i=0; i<y.d0; i++) for(j=0; j<y.d1; j++) for(k=0; k<y.d2; k++) x(i, j)+=y(i, j, k);
  }
  if(d==1) {
    x.resize(y.d0, y.d2); x=(double)0;
    for(i=0; i<y.d0; i++) for(j=0; j<y.d1; j++) for(k=0; k<y.d2; k++) x(i, k)+=y(i, j, k);
  }
  if(d==0) {
    x.resize(y.d1, y.d2); x=(double)0;
    for(i=0; i<y.d0; i++) for(j=0; j<y.d1; j++) for(k=0; k<y.d2; k++) x(j, k)+=y(i, j, k);
  }
}

//===========================================================================
//
/// @name distances
//

/// \f$\sum_i (v^i-w^i)^2\f$
double sqrDistance(const arr& v, const arr& w) {
  CHECK_EQ(v.N, w.N,
           "sqrDistance on different array dimensions (" <<v.N <<", " <<w.N <<")");
  double d, t(0);
  for(uint i=v.N; i--;) { d=v.p[i]-w.p[i]; t+=d*d; }
  return t;
}

double maxDiff(const arr& v, const arr& w, uint* im) {
  CHECK_EQ(v.N, w.N,
           "maxDiff on different array dimensions (" <<v.N <<", " <<w.N <<")");
  double d(0), t(0);
  if(!im)
    for(uint i=v.N; i--;) {
      d=(double)std::fabs((double)(v.p[i]-w.p[i]));
      if(d>t) t=d;
    } else {
    *im=0;
    for(uint i=v.N; i--;) { d=(double)std::fabs((double)(v.p[i]-w.p[i])); if(d>t) { t=d; *im=i; } }
  }
  return t;
}

double maxRelDiff(const arr& v, const arr& w, double tol) {
  CHECK_EQ(v.N, w.N,
           "maxDiff on different array dimensions (" <<v.N <<", " <<w.N <<")");
  double d, t(0), a, b, c;
  for(uint i=v.N; i--;) {
    a=(double)std::fabs((double)v.p[i]) + tol;
    b=(double)std::fabs((double)w.p[i]) + tol;
    if(a<b) { c=a; a=b; b=c; }
    d=a/b-(double)1;
    if(d>t) t=d;
  }
  return t;
}

/// \f$\sum_{i|{\rm mask}_i={\rm true}} (v^i-w^i)^2\f$
/*template<class T>
  double sqrDistance(const arr& v, const arr& w, const rai::Array<bool>& mask){
  CHECK_EQ(v.N,w.N,
  "sqrDistance on different array dimensions (" <<v.N <<", " <<w.N <<")");
  double d, t(0);
  for(uint i=v.N;i--;) if(mask(i)){ d=v.p[i]-w.p[i]; t+=d*d; }
  return t;
  }*/

/// \f$\sqrt{\sum_{ij} g_{ij} (v^i-w^i) (v^j-w^j)}\f$
double sqrDistance(const arr& g, const arr& v, const arr& w) {
  arr d(v);
  d -= w;
  return scalarProduct(g, d, d);
}

/// \f$\sqrt{\sum_i (v^i-w^i)^2}\f$
double euclideanDistance(const arr& v, const arr& w) {
  return (double)std::sqrt((double)sqrDistance(v, w));
}

/// \f$\sqrt{\sum_i (v^i-w^i)^2}\f$
double metricDistance(const arr& g, const arr& v, const arr& w) {
  return (double)std::sqrt((double)sqrDistance(g, v, w));
}

//===========================================================================
//
/// @name running sums
//

/// \f$\sum_i x_i\f$
double sum(const arr& v) {
  double t(0);
  for(uint i=v.N; i--; t+=v.p[i]) {};
  return t;
}

/// \f$\max_i x_i\f$
double max(const arr& x) {
  CHECK(x.N, "");
  double m(x.p[0]);
  for(uint i=x.N; --i;) if(x.p[i]>m) m=x.p[i];
  return m;
}

/// \f$\min_i x_i\f$
double min(const arr& x) {
  CHECK(x.N, "");
  double m(x.p[0]);
  for(uint i=x.N; --i;) if(x.p[i]<m) m=x.p[i];
  return m;
}

/// get absolute min (using fabs)
double absMin(const arr& x) {
  CHECK(x.N, "");
  double m(std::fabs(x.p[0]));
  for(uint i=x.N; --i;) if(std::fabs(x.p[i])<m) m=std::fabs(x.p[i]);
  return m;
}

/// get absolute maximum (using fabs)
double absMax(const arr& x) {
  if(!x.N) return 0;
  double m(std::fabs(x.p[0]));
  for(uint i=x.N; --i;) if(std::fabs(x.p[i])>m) m=std::fabs(x.p[i]);
  return m;
}

uint argmin(const arr& x) { CHECK_GE(x.N, 1, ""); uint m=0; for(uint i=x.N; --i;) if(x.p[i]<x.p[m]) m=i; return m; }

uint argmax(const arr& x) { CHECK_GE(x.N, 1, ""); uint m=0; for(uint i=x.N; --i;) if(x.p[i]>x.p[m]) m=i; return m; }

void argmax(uint& i, uint& j, const arr& x) { CHECK_EQ(x.nd, 2, "needs 2D array"); j=argmax(x); i=j/x.d1; j=j%x.d1; }

void argmax(uint& i, uint& j, uint& k, const arr& x) { CHECK_EQ(x.nd, 3, "needs 3D array"); k=argmax(x); i=k/(x.d1*x.d2); k=k%(x.d1*x.d2); j=k/x.d2; k=k%x.d2; }

/// \f$\max_i x_i\f$
arr max(const arr& v, uint d) {
  CHECK(d<v.nd, "array doesn't have this dimension");
  arr x;
  x.referTo(v);
  arr M;
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
arr min(const arr& v, uint d) {
  CHECK(v.nd>d, "array doesn't have this dimension");
  arr x;
  x.referTo(v);
  arr M;
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



/// \f$\sum_i x_i\f$
arr sum(const arr& v, uint d) {
  CHECK(v.nd>d, "array doesn't have this dimension");
  arr x;
  x.referTo(v);
  arr S;
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
    IV = getIndexTuple(k, v.dim());
    for(i = 0, j = 0; i < IS.N; i++, j++) {
      if(i == d) j++;
      IS(i) = IV(j);
    }
    S.elem(IS) += x(k);
  }
//  S.reshape(S.N); //(mt: Hä? Hab ich das gemacht???)
  return S;
}

/// \f$\sum_i |x_i|\f$
double sumOfAbs(const arr& v) {
  double t(0);
  for(uint i=v.N; i--; t+=std::fabs(v.p[i])) {};
  return t;
}

/// \f$\sum_i |x_i|_+\f$
double sumOfPos(const arr& v) {
  double t(0);
  for(uint i=0;i<v.N;i++) if(v.p[i]>0) t+=v.p[i];
  return t;
}

/// \f$\sum_i x_i^2\f$
double sumOfSqr(const arr& v) {
  double t(0);
  for(uint i=v.N; i--; t+=v.p[i]*v.p[i]) {};
  return t;
}

/// \f$\sqrt{\sum_i x_i^2}\f$
double length(const arr& x) { return std::sqrt(sumOfSqr(x)); }

double var(const arr& x) { double m=sum(x)/x.N; return sumOfSqr(x)/x.N-m*m; }

arr mean(const arr& X) { CHECK_EQ(X.nd, 2, ""); return sum(X, 0)/double(X.d0); }

arr covar(const arr& X) { arr m=mean(X); return ((~X)*X)/double(X.d0)-m*~m; }

arr stdDev(const arr& v) {
  CHECK(v.d0 > 1, "empirical standard deviation makes sense only for N>1")
  arr m = sum(v, 0);
  arr vX;
  vX.referTo(v);
  vX.reshape(vX.d0, vX.N/vX.d0);
  arr x = zeros(vX.d1);
  for(uint i = 0; i < v.d0; i++) {
    for(uint j = 0; j < vX.d1; j++) {
      x(j) += rai::sqr(vX(i, j)-m(j)/vX.d0)/(vX.d0-1);
    }
  }
  x = sqrt(x);
  return x;
}

/// \f$\sum_i x_{ii}\f$
double trace(const arr& v) {
  CHECK(v.nd==2 && v.d0==v.d1, "only for squared matrix");
  double t(0);
  for(uint i=0; i<v.d0; i++) t+=v(i, i);
  return t;
}

double minDiag(const arr& v) {
  CHECK(v.nd==2 && v.d0==v.d1, "only for squared matrix");
  double t=v(0, 0);
  for(uint i=1; i<v.d0; i++) if(v(i, i)<t) t=v(i, i);
  return t;
}

/// get absolute min (using fabs)
void clip(const arr& x, double lo, double hi) {
  for(uint i=0; i<x.N; i++) rai::clip(x.p[i], lo, hi);
}

//===========================================================================
//
/// @name products
//

/// \f$\prod_i x_i\f$
double product(const arr& v) {
  double t(1);
  for(uint i=v.N; i--; t *= v.p[i]);
  return t;
}

/** @brief inner product (also ordinary matrix or scalar product):
  \f$\forall_{ik}:~ x_{ik} = \sum_j v_{ij}\, w_{jk}\f$ but also:
  \f$\forall_{i}:~ x_{i} = \sum_j v_{ij}\, w_{j}\f$*/
void op_innerProduct(arr& x, const arr& y, const arr& z) {
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
    if(y.d0==1){ //row vector -> scalar product}
      x.resize(1);
      x.p[0] = scalarProduct(y,z);
      if(y.jac || z.jac){
        if(y.jac && !z.jac) x.J() = ~z.noJ() * (*y.jac);
        else if(!y.jac && z.jac) x.J() = y.noJ() * (*z.jac);
        else x.J() = y.noJ() * (*z.jac) + ~z.noJ() * (*y.jac);
      }
    }else{
      if(rai::useLapack && typeid(double)==typeid(double)) {
        blas_Mv(x, y, z);
      }else{
        uint i, d0=y.d0, dk=y.d1;
        double* a, *astop, *b, *c;
        x.resize(d0); x.setZero();
        c=x.p;
        for(i=0; i<d0; i++) {
          //for(s=0., k=0;k<dk;k++) s+=y.p[i*dk+k]*z.p[k];
          //this is faster:
          a=y.p+i*dk; astop=a+dk; b=z.p;
          for(; a!=astop; a++, b++)(*c)+=(*a) * (*b);
          c++;
        }
      }
      if(y.jac || z.jac){
        if(y.jac) NIY;
        x.J() = y.noJ() * (*z.jac);
      }
    }
    return;
  }
  if(y.nd==2 && z.nd==2) {  //plain matrix multiplication
    CHECK_EQ(y.d1, z.d0, "wrong dimensions for inner product");
    uint i, j, d0=y.d0, d1=z.d1, dk=y.d1;
#if 0
    if(y.mtype==arr::diagMT) {
      x.resize(d0, d1);
      for(i=0; i<d0; i++) for(j=0; j<d1; j++) x(i, j) = y(i, i) * z(i, j);
      return;
    }
    if(z.mtype==arr::diagMT) {
      x.resize(d0, d1);
      for(i=0; i<d0; i++) for(j=0; j<d1; j++) x(i, j) *= y(i, j) * z(j, j);
      return;
    }
#endif
    if(typeid(double)==typeid(double)) {
      if(isSparseMatrix(y)) { x = y.sparse().A_B(z); return; }
      if(isSparseMatrix(z)) { x = z.sparse().B_A(y); return; }
      if(isRowShifted(y)) { x = y.rowShifted().A_B(z); return; }
      if(isRowShifted(z)) { x = z.rowShifted().B_A(y); return; }
      if(rai::useLapack){ blas_MM(x, y, z); return; }
    }
    double* a, *astop, *b, *c;
    x.resize(d0, d1); x.setZero();
    c=x.p;
    for(i=0; i<d0; i++) for(j=0; j<d1; j++) {
        //for(s=0., k=0;k<dk;k++) s+=y.p[i*dk+k]*z.p[k*d1+j];
        //this is faster:
        a=y.p+i*dk; astop=a+dk; b=z.p+j;
        for(; a!=astop; a++, b+=d1)(*c)+=(*a) * (*b);
        c++;
      }
    if(y.jac || z.jac){
      if(y.jac && !z.jac){
        CHECK_EQ(y.d0, 1, "");
        x.J().resize(z.d1, y.jac->d1);
        tensorEquation(x.J(), *y.jac, uintA{2,1}, z, uintA{2,0}, 1);
      }else NIY;
    }
    return;
  }
  if(y.nd==1 && z.nd==1 && z.N==1) {  //vector multiplied with scalar (disguised as 1D vector)
    x = y;
    x *= z.p[0];
    if(y.jac || z.jac){
      if(y.jac && z.jac) x.J() += y.noJ() * (*z.jac);
      else NIY;
    }
    return;
  }
  if(y.nd==1 && z.nd==1 && y.N==1) {  //vector multiplied with scalar (disguised as 1D vector)
    x = z;
    x *= y.p[0];
    if(y.jac || z.jac){
      if(y.jac && z.jac) x.J() += z.noJ() * (*y.jac);
      else NIY;
    }
    return;
  }
  if(y.nd==1 && z.nd==2 && z.d0==1) {  //vector x vector^double -> matrix (outer product)
    if(typeid(double)==typeid(double) && isSparse(z)) {
      arr _y;
      _y.referTo(y);
      _y.reshape(y.N, 1);
      x = z.sparse().B_A(_y);
      return;
    }
    uint i, j, d0=y.d0, d1=z.d1;
    x.resize(d0, d1);
    for(i=0; i<d0; i++) for(j=0; j<d1; j++) x(i, j)=y(i)*z(0, j);
    if(y.jac || z.jac){
      if(y.jac && !z.jac){
        x.J().resize(y.N, z.N, y.jac->d1);
        tensorEquation(x.J(), *y.jac, uintA{0,2}, z, uintA{3,1}, 1);
      }else NIY;
    }
    return;
  }
  if(y.nd==1 && z.nd==2) {  //vector^double x matrix -> vector^T
    HALT("This is a stupid old inconsistent case of matrix multiplication! Never use this convention! Change your code: transpose first term explicitly.");
    /*
    arr zt;
    transpose(zt, z);
    x = zt*y;
    */
  }
  if(y.nd==2 && z.nd==3) {
    arr zz; zz.referTo(z);
    zz.reshape(z.d0, z.d1*z.d2);
    op_innerProduct(x, y, zz);
    x.reshape(y.d0, z.d1, z.d2);
    if(y.jac || z.jac){ NIY }
    return;
  }
  if(y.nd==3 && z.nd==2) {
    arr yy; yy.referTo(y);
    yy.reshape(y.d0*y.d1, y.d2);
    op_innerProduct(x, yy, z);
    x.reshape(y.d0, y.d1, z.d1);
    if(y.jac || z.jac){ NIY }
    return;
  }
  if(y.nd==3 && z.nd==1) {
    arr yy; yy.referTo(y);
    yy.reshape(y.d0*y.d1, y.d2);
    op_innerProduct(x, yy, z);
    x.reshape(y.d0, y.d1);
    if(y.jac || z.jac){ NIY }
    return;
  }
  if(y.nd==1 && z.nd==3) {
    arr zz; zz.referTo(z);
    zz.reshape(z.d0, z.d1*z.d2);
    op_innerProduct(x, y, zz);
    x.reshape(z.d1, z.d2);
    if(y.jac || z.jac){ NIY }
    return;
  }
  if(y.nd==1 && z.nd==1) {  //should be scalar product, but be careful
    HALT("what do you want? scalar product or element wise multiplication?");
    CHECK_EQ(y.d0, z.d0, "wrong dimensions for inner product");
    uint k, dk=y.d0;
    x.resize(1);
    double s;
    for(s=0, k=0; k<dk; k++) s+=y.p[k]*z.p[k];
    x.p[0]=s;
    if(y.jac || z.jac){ NIY }
    return;
  }
  HALT("inner product - not yet implemented for these dimensions: " <<y.nd <<" " <<z.nd);
}

/** @brief outer product (also exterior or tensor product): \f$\forall_{ijk}:~
  x_{ijk} = v_{ij}\, w_{k}\f$ */
void op_outerProduct(arr& x, const arr& y, const arr& z) {
  if(y.nd==1 && z.nd==1) {
#if 1
    uint i, j, d0=y.d0, d1=z.d0;
    x.resize(d0, d1);
    for(i=0; i<d0; i++) for(j=0; j<d1; j++) x.p[i*d1+j] = y.p[i] * z.p[j];
#else
    x.resize(y.N, z.N);
    double yi, *zp=z.p, *zstop=zp+z.N, *xp;
    for(uint i=0; i<y.N; i++) {
      yi=y.p[i];
      xp=&x(i, 0);
      zp=z.p;
      for(; zp!=zstop; zp++, xp++) *xp = yi * *zp;
    }
#endif
    if(y.jac || z.jac){ NIY }
    return;
  }
  if(y.nd==2 && z.nd==1) {
    uint i, j, k, d0=y.d0, d1=y.d1, d2=z.d0;
    x.resize(d0, d1, d2);
    for(i=0; i<d0; i++) for(j=0; j<d1; j++) for(k=0; k<d2; k++) x.p[(i*d1+j)*d2+k] = y.p[i*d1+j] * z.p[k];
    if(y.jac || z.jac){ NIY }
    return;
  }
  HALT("outer product - not yet implemented for these dimensions");
}

/** @brief element wise product */
void op_elemWiseProduct(arr& x, const arr& y, const arr& z) {
  CHECK_EQ(y.N, z.N, "");
  x = y;
  x *= z;
}

/** @brief index wise (element-wise for vectors and matrices) product (also ordinary matrix or scalar product).:
  \f$\forall_{ik}:~ x_{ik} = \sum_j v_{ij}\, w_{jk}\f$ but also:
  \f$\forall_{i}:~ x_{i} = \sum_j v_{ij}\, w_{j}\f$
  \f$\forall_{i}:~ x_{ij} = v_{ij} w_{ij}\f$*/
void op_indexWiseProduct(arr& x, const arr& y, const arr& z) {
  if(y.N==1) { //scalar x any -> ..
    x=z;
    x*=y.elem();
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
      CHECK(typeid(double)==typeid(double), "only for double!");
      x.sparse().rowWiseMult(y);
      if(y.jac || z.jac){ NIY }
      return;
    }
    if(isRowShifted(z)){
      CHECK(typeid(double)==typeid(double), "only for double!");
      uint rowSize = x.rowShifted().rowSize;
      for(uint i=0; i<x.d0; i++) {
        double yi=y.p[i];
        double *xp=&x.rowShifted().entry(i,0);
        double *xstop=xp+rowSize;
        for(; xp!=xstop; xp++) *xp *= yi;
      }
      if(y.jac || z.jac){ NIY }
      return;
    }
    for(uint i=0; i<x.d0; i++) {
      double yi=y.p[i];
      double* xp=&x(i, 0), *xstop=xp+x.d1;
      for(; xp!=xstop; xp++) *xp *= yi;
    }
    if(y.jac || z.jac){
      if(y.jac && !z.jac){
        x.J().resize(z.d0, z.d1, y.jac->d1);
        tensorEquation(x.J(), *y.jac, uintA{0,2}, z, uintA{0,1});
      }else NIY;
    }
    return;
  }
  if(y.nd==2 && z.nd==1) {  //matrix x vector -> index-wise
    CHECK_EQ(y.d1, z.N, "wrong dims for indexWiseProduct:" <<y.d1 <<"!=" <<z.N);
    x=y;
    for(uint i=0; i<x.d0; i++) for(uint j=0; j<x.d1; j++) x(i, j) *= z(j);
    if(y.jac || z.jac){ NIY }
    return;
  }
  if(y.dim() == z.dim()) { //matrix x matrix -> element-wise
//    HALT("THIS IS AMBIGUOUS!");
    x = y;
    double* xp=x.p, *xstop=x.p+x.N, *zp=z.p;
    for(; xp!=xstop; xp++, zp++) *xp *= *zp;
    if(y.jac || z.jac){
      NIY;
      //if(!y.jac && z.jac) x.J() = y % (*z.jac);
      //else if(y.jac && !z.jac) x.J() = z % (*y.jac);
    }
    return;
  }
  HALT("operator% not implemented for dimensions "<<y.dim() <<"%" <<z.dim())
}

/** @brief outer product (also exterior or tensor product): \f$\forall_{ijk}:~
  x_{ijk} = v_{ij}\, w_{k}\f$ */
void op_crossProduct(arr& x, const arr& y, const arr& z) {
  if(!y || !z){ x.setNoArr(); return; }
  if(y.nd==1 && z.nd==1) {
    CHECK(y.N==3 && z.N==3, "cross product only works for 3D vectors!");
    x.resize(3);
    x.p[0]=y.p[1]*z.p[2]-y.p[2]*z.p[1];
    x.p[1]=y.p[2]*z.p[0]-y.p[0]*z.p[2];
    x.p[2]=y.p[0]*z.p[1]-y.p[1]*z.p[0];
    if(y.jac || z.jac){
      if(!y.jac && z.jac) x.J() = skew(y) * (*z.jac);
      else if(y.jac && !z.jac) x.J() = -skew(z) * (*y.jac);
      else x.J() = skew(y.noJ()) * (*z.jac) - skew(z.noJ()) * (*y.jac);
    }
    return;
  }
  if(y.nd==2 && z.nd==1) { //every COLUMN of y is cross-product'd with z!
    CHECK(y.d0==3 && z.N==3, "cross product only works for 3D vectors!");
    x = skew(-z) * y;
    if(y.jac || z.jac){
      //above should do autoDiff;
    }
    return;
  }
  HALT("cross product - not yet implemented for these dimensions");
}

/// \f$\sum_i v_i\, w_i\f$, or \f$\sum_{ij} v_{ij}\, w_{ij}\f$, etc.
double scalarProduct(const arr& v, const arr& w) {
  double t(0);
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
      double* vp=v.p, *wp=w.p;
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
double scalarProduct(const arr& g, const arr& v, const arr& w) {
  CHECK(v.N==w.N && g.nd==2 && g.d0==v.N && g.d1==w.N,
        "scalar product on different array dimensions (" <<v.N <<", " <<w.N <<")");
  CHECK(!v.jac && !w.jac, "you're loosing the jacobians with this method");
  double t(0);
  uint i, j;
  double* gp=g.p, *vp=v.p;
  for(i=0; i<g.d0; i++) {
    for(j=0; j<g.d1; j++) {
      t+=(*gp)*(*vp)*w.p[j];
      gp++;
    }
    vp++;
  }
  return t;
}

arr elemWiseMin(const arr& v, const arr& w) {
  arr z;
  z.resizeAs(v);
  for(uint i=0; i<v.N; i++) z.elem(i) = v.elem(i)<w.elem(i)?v.elem(i):w.elem(i);
  return z;
}

arr elemWiseMax(const arr& v, const arr& w) {
  arr z;
  z.resizeAs(v);
  for(uint i=0; i<v.N; i++) z.elem(i) = v.elem(i)>w.elem(i)?v.elem(i):w.elem(i);
  return z;
}

arr elemWiseMax(const arr& v, const double& w) {
  arr z;
  z.resizeAs(v);
  for(uint i=0; i<v.N; i++) z.elem(i) = v.elem(i)>w?v.elem(i):w;
  return z;
}

arr elemWiseMax(const double& v, const arr& w) {
  arr z;
  z.resizeAs(w);
  for(uint i=0; i<w.N; i++) z.elem(i) = v>w.elem(i)?v:w.elem(i);
  return z;
}

arr elemWiseHinge(const arr& x) {
  arr z = x;
  for(double& v:z) if(v<0.) v=0.;
  return z;
}

void writeConsecutiveConstant(std::ostream& os, const arr& x) {
  if(!x.N) return;
  uint yi=0;
  double y=x.elem(yi);
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
void tensorCondNormalize(arr& X, int left) {
  uint i, j, dl=1, dr;
  double sum;
  if(left>=0) {  //normalize over the left variables
    for(j=0; j<(uint)left; j++) dl*=X.dim(j);
    dr=X.N/dl;
    CHECK_EQ(dl*dr, X.N, "");
    for(i=0; i<dr; i++) {
      sum=(double)0;
      for(j=0; j<dl; j++)  sum += X.p[j*dr + i];
      if(sum) for(j=0; j<dl; j++) X.p[j*dr + i] /= sum;
      else    for(j=0; j<dl; j++) X.p[j*dr + i] = (double)1/dl;
    }
  } else { //normalize over the right variables
    for(j=0; j<(uint)-left; j++) dl*=X.dim(j);
    dr=X.N/dl;
    CHECK_EQ(dl*dr, X.N, "");
    for(i=0; i<dl; i++) {
      sum=(double)0;
      for(j=0; j<dr; j++)  sum += X.p[i*dr + j];
      if(sum) for(j=0; j<dr; j++) X.p[i*dr + j] /= sum;
      else    for(j=0; j<dr; j++) X.p[i*dr + j] = (double)1/dr;
    }
  }
}

/** makes X to be a distribution over the left leftmost-indexed
  variables and normalizes it */
void tensorCondMax(arr& X, uint left) {
  uint i, j, dl=1, dr, jmax;
  double pmax;
  for(j=0; j<left; j++) dl*=X.dim(j);
  dr=X.N/dl;
  CHECK_EQ(dl*dr, X.N, "");
  for(i=0; i<dr; i++) {
    jmax=0;
    pmax=X.p[jmax*dr + i];
    X.p[i]=(double)0;
    for(j=1; j<dl; j++) {
      if(X.p[j*dr + i]>pmax) {  jmax=j;  pmax=X.p[jmax*dr + i];  }
      X.p[j*dr + i]=(double)0;
    }
    X.p[jmax*dr + i]=(double)1;
  }
}

/** makes X to be a distribution over the left leftmost-indexed
  variables and normalizes it */
void tensorCondSoftMax(arr& X, uint left, double beta) {
  uint i;
  for(i=0; i<X.N; i++) X.p[i] = (double)::exp(beta*X.p[i]);
  tensorCondNormalize(X, left);
}

/** @brief checks whether X is a normalized distribution over the left leftmost-indexed
  variables */
void tensorCheckCondNormalization(const arr& X, uint left, double tol) {
  uint i, j, dl=1, dr;
  double sum;
  for(j=0; j<left; j++) dl*=X.dim(j);
  dr=X.N/dl;
  CHECK_EQ(dl*dr, X.N, "");
  for(i=0; i<dr; i++) {
    sum=0.;
    for(j=0; j<dl; j++) sum += X.p[j*dr + i];
    CHECK(std::fabs(1.-sum)<tol, "distribution is not normalized: " <<X);
  }
}

void tensorCheckCondNormalization_with_logP(const arr& X, uint left, double logP, double tol) {
  uint i, j, dl=1, dr;
  double sum, coeff=::exp(logP);
  for(j=0; j<left; j++) dl*=X.dim(j);
  dr=X.N/dl;
  CHECK_EQ(dl*dr, X.N, "");
  for(i=0; i<dr; i++) {
    sum=0.;
    uintA checkedIds;
    for(j=0; j<dl; j++) { sum += X.p[j*dr + i]*coeff; checkedIds.append(j*dr + i); }
    CHECK(std::fabs(1.-sum)<tol, "distribution is not normalized for parents-config#" <<i <<endl <<checkedIds<<endl <<" " <<X);
  }
}

/** X becomes a tensor product (maybe with some variables summed out)
  of A and B. pickA and pickB indicate which slots of A and B are
  associated with which slots of C and the summation sign. More
  precisely, if we have \f$C_{i_0i_1i_2} = \sum_{i_3i_4}
  A_{i_4i_2i_1}~ B_{i_3i_0}\f$ then you should call
  tensor(C, A, uintA{4, 2, 1}, B, uintA{3, 0}, 2); Here, the `2` indicates that
  the last two indices of i_0, .., i_4 are summed over, and C only
  becomes a 3rd rank instead of 5th rank tensor */
void tensorEquation(arr& X, const arr& A, const uintA& pickA, const arr& B, const uintA& pickB, uint sum) {
  CHECK(&X!=&A && &X!=&B, "output tensor must be different from input tensors");
  CHECK(A.nd==pickA.N && B.nd==pickB.N, "miss-sized tensor references: " <<A.nd <<"!=" <<pickA.N <<" " <<B.nd <<"!=" <<pickB.N);

  uint n=1+rai::MAX(max(pickA), max(pickB));
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

void tensorEquation_doesntWorkLikeThat(arr& X, const arr& A, const uintA& pickA, const arr& B, const uintA& pickB, uint sum) {
  CHECK(A.nd==pickA.N && B.nd==pickB.N, "miss-sized tensor references: " <<A.nd <<"!=" <<pickA.N <<" " <<B.nd <<"!=" <<pickB.N);

  uint n=1+rai::MAX(max(pickA), max(pickB));
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
  op_innerProduct(X, Aperm, Bperm);

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
void tensorMarginal(arr& Y, const arr& X, const uintA& Yid) {
  uint Xcount, Ycount;
  CHECK_LE(Yid.N, X.nd, "can't take slots " <<Yid <<" from " <<X.nd <<"D tensor");

  //handle scalar case
  if(!Yid.N) {  Y.resize(1);  Y.nd=0;  Y.elem()=sum(X);  return;  }

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
void tensorPermutation(arr& Y, const arr& X, const uintA& Yid) {
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
void tensorMaxMarginal(arr& Y, const arr& X, const uintA& Yid) {
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
void tensorAdd_old(arr& X, const arr& Y, const uintA& Yid) {
  uint Xcount, Ycount;
  CHECK_EQ(Yid.N, Y.nd, "need to specify " <<Y.nd <<" slots, not " <<Yid.N);
  CHECK_LE(Yid.N, X.nd, "can't take slots " <<Yid <<" from " <<X.nd <<"D tensor");

  //handle scalar case
  if(!Yid.N) { CHECK_EQ(Y.N, 1, "");  X+=Y.elem();  return; } //Y is only a scalar

  //initialize looping
  uint I[maxRank];     memset(I, 0, sizeof(uint)*maxRank);  //index on X
  uint Ydim[maxRank], Yinc[maxRank], Ydec[maxRank];
  getMultiDimIncrement(X.dim(), Yid, Ydim, Yinc, Ydec);
  X.resize(Yid.N, Ydim);
  X.setZero();

  //loop
  for(Xcount=0, Ycount=0; Xcount<X.N; Xcount++) {
    X.p[Xcount] += Y.p[Ycount];
    multiDimIncrement(Ycount, I, X.d, Yinc, Ydec, X.nd);
  }
}

void tensorMarginal_old(arr& y, const arr& x, const uintA& xd, const uintA& ids) {
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
void tensorMultiply(arr& X, const arr& Y, const uintA& Yid) {
  uint Xcount, Ycount;
  CHECK_EQ(Yid.N, Y.nd, "need to specify " <<Y.nd <<" slots, not " <<Yid.N);
  CHECK_LE(Yid.N, X.nd, "can't take slots " <<Yid <<" from " <<X.nd <<"D tensor");

  //handle scalar case
  if(!Yid.N) { CHECK_EQ(Y.N, 1, "");  X*=Y.elem();  return; } //Y is only a scalar

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
void tensorDivide(arr& X, const arr& Y, const uintA& Yid) {
  uint Xcount, Ycount;
  CHECK_EQ(Yid.N, Y.nd, "need to specify " <<Y.nd <<" slots, not " <<Yid.N);
  CHECK_LE(Yid.N, X.nd, "can't take slots " <<Yid <<" from " <<X.nd <<"D tensor");

  //handle scalar case
  if(!Yid.N) { CHECK_EQ(Y.N, 1, "");  X/=Y.elem();  return; } //Y is only a scalar

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

void tensorAdd(arr& X, const arr& Y, const uintA& Yid) {
  uint Xcount, Ycount;
  CHECK_EQ(Yid.N, Y.nd, "need to specify " <<Y.nd <<" slots, not " <<Yid.N);
  CHECK_LE(Yid.N, X.nd, "can't take slots " <<Yid <<" from " <<X.nd <<"D tensor");

  //handle scalar case
  if(!Yid.N) { CHECK_EQ(Y.N, 1, "");  X+=Y.elem();  return; } //Y is only a scalar

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
void tensorMultiply_old(arr& x, const arr& y, const uintA& d, const uintA& ids) {
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
/// @name randomizations
//

/// Assign all elements of \c a to a uniformly distributed discrete value in {low, .., hi}
void rndInteger(arr& a, int low, int high, bool add) {
  if(!add) for(uint i=0; i<a.N; i++) a.p[i] =(double)(low+(int)rnd.num(1+high-low));
  else     for(uint i=0; i<a.N; i++) a.p[i]+=(double)(low+(int)rnd.num(1+high-low));
}

/// Assign all elements of \c a to a uniformly distributed continuous value in [low, hi]
void rndUniform(arr& a, double low, double high, bool add) {
  if(!add) for(uint i=0; i<a.N; i++) a.p[i] =(double)rnd.uni(low, high);
  else     for(uint i=0; i<a.N; i++) a.p[i]+=(double)rnd.uni(low, high);
}

void rndNegLogUniform(arr& a, double low, double high, bool add) {
  if(!add) for(uint i=0; i<a.N; i++) a.p[i] =(double)(-::log(rnd.uni(low, high)));
  else     for(uint i=0; i<a.N; i++) a.p[i]+=(double)(-::log(rnd.uni(low, high)));
}

/** Assign all elements of x to a Gaussian random variable where
    _each_ elelemt has the given stdDev -- devide by sqrt(N) if you
    want the multivariate Gaussian to have a given standard deviation!
    If add is true, the Gaussian noise is added to the existing
    value */
void rndGauss(arr& x, double stdDev, bool add) {
  if(!add) for(uint i=0; i<x.N; i++) x.p[i] =(double)(stdDev*rnd.gauss());
  else     for(uint i=0; i<x.N; i++) x.p[i]+=(double)(stdDev*rnd.gauss());
}

/// a gaussian random vector with Id covariance matrix (sdv = sqrt(dimension))
/*void rndGauss(arr& a, bool add){
  if(!add) for(uint i=0;i<a.N;i++) a.p[i]=rnd.gauss();
  else     for(uint i=0;i<a.N;i++) a.p[i]+=rnd.gauss();
  }*/

/// returns an array with \c dim Gaussian noise elements
/*arr& rndGauss(double stdDev, uint dim){
  static arr z;
  stdDev/=std::sqrt(dim);
  z.resize(dim);
  rndGauss(z, stdDev);
  return z;
  }*/

/** @brief from a vector of numbers, calculates the softmax distribution
  soft(i) = exp(beta*a(i)), and returns a random sample from
  this distribution (an index in {0, .., a.N-1}) */
uint softMax(const arr& a, arr& soft, double beta) {
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
arr operator+(const arr& y, const arr& z) { arr x(y); x+=z; return x; }
arr operator+(double y, const arr& z){                arr x; x.resizeAs(z); x=y; x+=z; return x; }
arr operator+(const arr& y, double z){                arr x(y); x+=z; return x; }

//subtraction
arr operator-(const arr& y, const arr& z) { arr x(y); x-=z; return x; }
arr operator-(double y, const arr& z){                arr x; x.resizeAs(z); x=y; x-=z; return x; }
arr operator-(const arr& y, double z){                arr x(y); x-=z; return x; }

/// transpose
arr operator~(const arr& y) { arr x; op_transpose(x, y); return x; }
/// negative
arr operator-(const arr& y) { arr x; op_negative(x, y);  return x; }

/// outer product (notation borrowed from the wedge product, though not anti-symmetric)
arr operator^(const arr& y, const arr& z) { arr x; op_outerProduct(x, y, z); return x; }

/// inner product
arr operator*(const arr& y, const arr& z) { arr x; op_innerProduct(x, y, z); return x; }
/// scalar multiplication
arr operator*(const arr& y, double z) {             arr x(y); x*=z; return x; }
/// scalar multiplication
arr operator*(double y, const arr& z) {             arr x(z); x*=y; return x; }

/// index-wise (elem-wise) product (x_i = y_i z_i   or  X_{ij} = y_i Z_{ij}  or  X_{ijk} = Y_{ij} Z_{jk}   etc)
arr operator%(const arr& y, const arr& z) { arr x; op_indexWiseProduct(x, y, z); return x; }

/// inverse
arr operator/(int y, const arr& z) {  CHECK_EQ(y, 1, ""); arr x; x = inverse(z); return x; }
/// scalar division
arr operator/(const arr& y, double z) {             arr x(y); x/=z; return x; }
/// element-wise division
arr operator/(const arr& y, const arr& z) { arr x(y); x/=z; return x; }

/// contatenation of two arrays
arr operator, (const arr& y, const arr& z) { arr x(y); x.append(z); return x; }

/// x.append(y)
arr& operator<<(arr& x, const double& y) { x.append(y); return x; }

/// x.append(y)
arr& operator<<(arr& x, const arr& y) { x.append(y); return x; }


//core for matrix-matrix (elem-wise) update
#define UpdateOperator_MM( op )        \
    if(isNoArr(x)){ return x; } \
    if(isSparseMatrix(x) && isSparseMatrix(y)){ x.sparse() op y.sparse(); return x; }  \
    if(isRowShifted(x) && isRowShifted(y)){ x.rowShifted() op y.rowShifted(); return x; }  \
    CHECK(!isSpecial(x), "");  \
    CHECK(!isSpecial(y), "");  \
    CHECK_EQ(x.N, y.N, "update operator on different array dimensions (" <<x.N <<", " <<y.N <<")"); \
    double *xp=x.p, *xstop=xp+x.N;              \
    const double *yp=y.p;              \
    for(; xp!=xstop; xp++, yp++) *xp op *yp;

//core for matrix-scalar update
#define UpdateOperator_MS( op ) \
  if(isNoArr(x)){ return x; } \
  if(isSparseMatrix(x)){ x.sparse() op y; return x; }  \
  if(isRowShifted(x)){ x.rowShifted() op y; return x; }  \
  CHECK(!isSpecial(x), "");  \
  double *xp=x.p, *xstop=xp+x.N;              \
  for(; xp!=xstop; xp++) *xp op y;


arr& operator+=(arr& x, const arr& y){
  UpdateOperator_MM(+=);
  if(y.jac){
    if(x.jac) *x.jac += *y.jac;
    else x.J() = *y.jac;
  }
  return x;
}
arr& operator+=(arr& x, double y){
  UpdateOperator_MS(+=);
  return x;
}
arr& operator+=(arr&& x, const arr& y){
  UpdateOperator_MM(+=);
  if(y.jac){
    if(x.jac) *x.jac += *y.jac;
    else x.J() = *y.jac;
  }
  return x;
}
arr& operator+=(arr&& x, double y){
  UpdateOperator_MS(+=);
  return x;
}

arr& operator-=(arr& x, const arr& y){
  UpdateOperator_MM(-=);
  if(y.jac){
    if(x.jac) *x.jac -= *y.jac;
    else x.J() = -(*y.jac);
  }
  return x;
}
arr& operator-=(arr& x, double y){
  UpdateOperator_MS(-=);
  return x;
}
  arr& operator-=(arr&& x, const arr& y){
    UpdateOperator_MM(-=);
    if(y.jac){
      if(x.jac) *x.jac -= *y.jac;
      else x.J() = -(*y.jac);
    }
    return x;
  }
  arr& operator-=(arr&& x, double y){
    UpdateOperator_MS(-=);
    return x;
  }

arr& operator*=(arr& x, const arr& y){
  if(x.jac || y.jac){
    CHECK_EQ(x.nd, 1, "");
    CHECK_EQ(y.nd, 1, "");
    if(x.jac && !y.jac) *x.jac = y % (*x.jac);
    else if(!x.jac && y.jac) x.J() = x % (*y.jac);
    else{ *x.jac = y.noJ() % (*x.jac); *x.jac += x.noJ() % (*y.jac); }
  }
  UpdateOperator_MM(*=);
  return x;
}
arr& operator*=(arr& x, double y){
  if(x.jac) *x.jac *= y;
  UpdateOperator_MS(*=);
  return x;
}
  arr& operator*=(arr&& x, const arr& y){
    if(x.jac || y.jac){
      CHECK_EQ(x.nd, 1, "");
      if(x.jac && !y.jac) *x.jac = y.noJ() % (*x.jac);
      else if(!x.jac && y.jac) x.J() = x.noJ() % (*y.jac);
      else NIY;
    }
    UpdateOperator_MM(*=);
    return x;
  }
  arr& operator*=(arr&& x, double y){
    if(x.jac) *x.jac *= y;
    UpdateOperator_MS(*=);
    return x;
  }

arr& operator/=(arr& x, const arr& y){
  UpdateOperator_MM(/=);
  if(x.jac || y.jac){
    if(x.jac && !y.jac){
      arr yinv(y.N);
      for(uint i=0;i<y.N;i++) yinv.p[i] = 1./y.p[i];
      *x.jac = yinv % (*x.jac);
    }else if(!x.jac && y.jac){
      arr coeff(y.N);
      for(uint i=0;i<y.N;i++) coeff.p[i] = -x.p[i]/y.p[i]; //NOTE: x(i) is already divided by y(i)!
      x.J() = coeff % (*y.jac);
    }else{
      arr coeff(y.N);
      for(uint i=0;i<y.N;i++) coeff.p[i] = 1./y.p[i];
      *x.jac = coeff % (*x.jac);
      for(uint i=0;i<y.N;i++) coeff.p[i] = -x.p[i]/y.p[i];
      *x.jac += coeff % (*y.jac);
    }
  }
  return x;
}
arr& operator/=(arr& x, double y){
  UpdateOperator_MS(/=);
  if(x.jac) *x.jac /= y;
  return x;
}
  arr& operator/=(arr&& x, const arr& y){
    UpdateOperator_MM(/=);
    if(x.jac || y.jac){
      NIY;
    }
    return x;
  }
  arr& operator/=(arr&& x, double y){
    UpdateOperator_MS(/=);
    if(x.jac) *x.jac /= y;
    return x;
  }



//UpdateOperator(|=)
//UpdateOperator(^=)
//UpdateOperator(&=)
//UpdateOperator(%=)
#undef UpdateOperator_MM
#undef UpdateOperator_MS


/// allows a notation such as x <<"[0 1; 2 3]"; to initialize an array x
//arr& operator<<(arr& x, const char* str) { std::istringstream ss(str); ss >>x; return x; }

/// calls arr::read
std::istream& operator>>(std::istream& is, arr& x) { x.read(is); return is; }

/// calls arr::write
std::ostream& operator<<(std::ostream& os, const arr& x) { x.write(os);  return os; }

/// equal in size and all elements
bool operator==(const arr& v, const arr& w) {
  if(!w) return !v; //if w==NoArr
  if(!samedim(v, w)) return false;
  const double* vp=v.p, *wp=w.p, *vstop=vp+v.N;
  for(; vp!=vstop; vp++, wp++)
    if(*vp != *wp) return false;
  return true;
}

/// element-wise equal to constant
Array<byte> operator==(const arr& v, const double& w) {
  Array<byte> x;
  resizeAs(x, v);
  x.setZero();
  const double* vp=v.p, *vstop=vp+v.N;
  byte* xp=x.p;
  for(; vp!=vstop; vp++, xp++)
    if(*vp == w) *xp=1;
  return x;
}

///// equal in size and all elements
//bool operator==(const arr& v, const double *w) {
//  const double *vp=v.p, *wp=w, *vstop=vp+v.N;
//  for(; vp!=vstop; vp++, wp++)
//    if(*vp != *wp) return false;
//  return true;
//}

/// not equal
bool operator!=(const arr& v, const arr& w) {
  return !(v==w);
}

/// lexical comparison
bool operator<(const arr& v, const arr& w) {
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

void op_negative(arr& x, const arr& y) {
  x=y;
  double* xp=x.p, *xstop=xp+x.N;
  for(; xp!=xstop; xp++) *xp = - (*xp);
  if(y.jac) op_negative(x.J(), *y.jac);
}

//---------- unary functions

inline double sigm(double x) {  return 1./(1.+::exp(-x)); }

inline double sign(double x) {  return (x > 0) - (x < 0); }

#define UnaryFunction( func )         \
            \
  arr func (const arr& y){    \
    arr x;           \
    if(&x!=&y) x.resizeAs(y);         \
    double *xp=x.p, *xstop=xp+x.N, *yp=y.p;            \
    for(; xp!=xstop; xp++, yp++) *xp = (double)::func( (double) *yp );  \
    CHECK(!y.jac, "AutoDiff NIY"); \
    return x;         \
  }

// trigonometric functions
UnaryFunction(acos)
UnaryFunction(asin)
UnaryFunction(atan)
UnaryFunction(cos)
UnaryFunction(sin)
UnaryFunction(tan)

// hyperbolic functions
UnaryFunction(cosh)
UnaryFunction(sinh)
UnaryFunction(tanh)
UnaryFunction(acosh)
UnaryFunction(asinh)
UnaryFunction(atanh)

// exponential and logarithmic functions
UnaryFunction(exp)
UnaryFunction(log)
UnaryFunction(log10)

//roots
UnaryFunction(sqrt)
UnaryFunction(cbrt)

// nearest integer and absolute value
UnaryFunction(ceil)
UnaryFunction(fabs)
UnaryFunction(floor)
UnaryFunction(sigm)

UnaryFunction(sign)

#undef UnaryFunction

//---------- binary functions

#define BinaryFunction( func )            \
              \
  arr func(const arr& y, const arr& z){ \
    CHECK_EQ(y.N,z.N,             \
             "binary operator on different array dimensions (" <<y.N <<", " <<z.N <<")"); \
    arr x;             \
    x.resizeAs(y);              \
    for(uint i=x.N;i--; ) x.p[i]= func(y.p[i], z.p[i]);      \
    return x;           \
  }                 \
  \
              \
  arr func(const arr& y, double z){     \
    arr x;             \
    x.resizeAs(y);              \
    for(uint i=x.N;i--; ) x.p[i]= func(y.p[i], z);     \
    return x;           \
  }                 \
  \
              \
  arr func(double y, const arr& z){     \
    arr x;             \
    x.resizeAs(z);              \
    for(uint i=x.N;i--; ) x.p[i]= func(y, z.p[i]);     \
    return x;           \
  }

BinaryFunction(atan2)
#ifndef RAI_MSVC
BinaryFunction(pow)
#endif
BinaryFunction(fmod)
#undef BinaryFunction

#ifndef RAI_doxy // exclude these macros when generating the documentation
// (doxygen can't handle them...)
#endif //(doxygen exclusion)

/*

/// element-wise linear combination (plus with scalar factors for each array)
void plusSASA(arr& x, double a, const arr& y, double b, const arr& z){
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
#define double double
#  include "array_instantiate.cpp"
#define double int
#  include "array_instantiate.cpp"
#endif


//===========================================================================
//
// differentiation
//

void arr::J_setId() {
  CHECK(!jac, "");
  CHECK(nd==1,"");
  jac = make_unique<arr>();
  jac->setId(N);
}

#pragma GCC diagnostic pop
