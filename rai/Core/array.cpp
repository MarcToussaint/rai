/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

/* Because of some stupid thing in old Eigen libs (that come with
   Ubuntu 16.04), the following includes have to be BEFORE others
   (which is against usual convention) */
#ifdef RAI_EIGEN
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>
#endif

#include "array.h"
#include "util.h"

#ifdef RAI_LAPACK
extern "C" {
#include "cblas.h"
#ifdef RAI_MSVC
#  include "lapack/blaswrap.h"
#endif
#include "f2c.h"
#undef small
#undef large
#ifndef ATLAS
#  include "lapack/clapack.h"
#endif
#undef double
#undef max
#undef min
#undef abs
}

#ifdef ATLAS
#include <complex>
#define lapack_complex_float std::complex<float>
#define lapack_complex_double std::complex<double>

#include "lapack/lapacke.h"
#define integer int
#undef MAX
#undef MIN
#endif
#endif //RAI_LAPACK

namespace rai {
//===========================================================================

bool useLapack=true;
#ifdef RAI_LAPACK
const bool lapackSupported=true;
#else
const bool lapackSupported=false;
#endif
uint64_t globalMemoryTotal=0, globalMemoryBound=1ull<<30; //this is 1GB
bool globalMemoryStrict=false;
const char* arrayElemsep=", ";
const char* arrayLinesep=",\n ";
const char* arrayBrackets="[]";

//===========================================================================
}

arr __NoArr(new SpecialArray(SpecialArray::ST_NoArr));
arr& NoArr = __NoArr;
arrA __NoArrA(new SpecialArray(SpecialArray::ST_NoArr));
arrA& NoArrA = __NoArrA;
uintA __NoUintA(new SpecialArray(SpecialArray::ST_NoArr));
uintA& NoUintA = __NoUintA;
uint16A __NoUint16A(new SpecialArray(SpecialArray::ST_NoArr));
uint16A& NoUint16A = __NoUint16A;
byteA __NoByteA(new SpecialArray(SpecialArray::ST_NoArr));
byteA& NoByteA = __NoByteA;
intAA __NoIntAA(new SpecialArray(SpecialArray::ST_NoArr));
intAA& NoIntAA = __NoIntAA;
uintAA __NoUintAA(new SpecialArray(SpecialArray::ST_NoArr));
uintAA& NoUintAA = __NoUintAA;
StringA __NoStringA(new SpecialArray(SpecialArray::ST_NoArr));
StringA& NoStringA = __NoStringA;

/* LAPACK notes
Use the documentation at
  http://www.netlib.org/lapack/double/
  http://www.netlib.org/lapack/individualroutines.html
to find the right function! Also use the man tools with Debian package lapack-doc installed.

I've put the clapack.h directly into the rai directory - one only has to link to the Fortran lib
*/

namespace rai {

/// make sparse: create the \ref sparse index
template<> SparseVector& Array<double>::sparseVec() {
  SparseVector* s;
  if(!special) {
    s = new SparseVector(*this);
    if(N) {
      CHECK_EQ(nd, 1, "");
      arr copy;
      copy.swap(*this);
      s->setFromDense(copy);
    } else {
      nd=1;
    }
  } else {
    s = dynamic_cast<SparseVector*>(special);
    CHECK(s, "");
  }
  return *s;
}

template<> const SparseVector& Array<double>::sparseVec() const {
  CHECK(isSparseVector(*this), "");
  SparseVector* s = dynamic_cast<SparseVector*>(special);
  CHECK(s, "");
  return *s;
}

/// make sparse: create the \ref sparse index
template<> SparseMatrix& Array<double>::sparse() {
  SparseMatrix* s;
  if(!special) {
    s = new SparseMatrix(*this);
    if(N) {
      CHECK_EQ(nd, 2, "");
      arr copy;
      copy.swap(*this);
      s->setFromDense(copy);
    } else {
      nd=2;
    }
  } else {
    s = dynamic_cast<SparseMatrix*>(special);
    CHECK(s, "");
  }
  return *s;
}

/// make sparse: create the \ref sparse index
template<> const SparseMatrix& Array<double>::sparse() const {
  CHECK(isSparseMatrix(*this), "");
  SparseMatrix* s = dynamic_cast<SparseMatrix*>(special);
  CHECK(s, "");
  return *s;
}

/// make sparse: create the \ref sparse index
template<> RowShifted& Array<double>::rowShifted() {
  rai::RowShifted* r;
  if(!special) {
    r = new rai::RowShifted(*this);
    if(N) {
      CHECK_EQ(nd, 2, "");
      r->rowSize = d1;
      r->rowShift.resize(d0).setZero();
      r->rowLen.resize(d0) = d1;
    } else {
      nd=2;
    }
  } else {
    r = dynamic_cast<RowShifted*>(special);
    CHECK(r, "");
  }
  return *r;
}

  /// make sparse: create the \ref sparse index
template<> const RowShifted& Array<double>::rowShifted() const{
  CHECK(isRowShifted(*this), "");
  rai::RowShifted* r = dynamic_cast<RowShifted*>(special);
  CHECK(r, "");
  return *r;
}

#define NONSENSE( type ) \
  template<> SparseMatrix& Array<type>::sparse() { NIY; return *(new SparseMatrix(NoArr)); } \
  template<> const SparseMatrix& Array<type>::sparse() const{ NIY; return *(new SparseMatrix(NoArr)); } \
  template<> RowShifted& Array<type>::rowShifted() { NIY; return *(new RowShifted(NoArr)); } \
  template<> const RowShifted& Array<type>::rowShifted() const{ NIY; return *(new RowShifted(NoArr)); }
NONSENSE(float)
NONSENSE(uint)
NONSENSE(int)
NONSENSE(byte)
#undef NONSENSE

}

//===========================================================================
//
/// @name matrix operations
//

arr grid(const arr& lo, const arr& hi, const uintA& steps) {
  CHECK(lo.N==hi.N && lo.N==steps.N, "");
  arr X;
  uint i, j, k;
  if(lo.N==1) {
    X.resize(steps(0)+1, 1);
    for(i=0; i<X.d0; i++) X.operator()(i, 0)=lo(0)+(hi(0)-lo(0))*i/steps(0);
    return X;
  }
  if(lo.N==2) {
    X.resize(steps(0)+1, steps(1)+1, 2);
    for(i=0; i<X.d0; i++) for(j=0; j<X.d1; j++) {
        X.operator()(i, j, 0)=lo(0)+(i?(hi(0)-lo(0))*i/steps(0):0.);
        X.operator()(i, j, 1)=lo(1)+(j?(hi(1)-lo(1))*j/steps(1):0.);
      }
    X.reshape(X.d0*X.d1, 2);
    return X;
  }
  if(lo.N==3) {
    arr delta = zeros(3);
    for(uint i=0; i<3; i++) if(steps(i)) delta(i) = (hi(i)-lo(i))/steps(i);

    X.resize(TUP(steps(2)+1, steps(1)+1, steps(0)+1, 3));
    for(i=0; i<X.d0; i++) for(j=0; j<X.d1; j++) for(k=0; k<X.d2; k++) {
          X.elem(TUP(i, j, k, 0))=lo(0)+delta(0)*k;
          X.elem(TUP(i, j, k, 1))=lo(1)+delta(1)*j;
          X.elem(TUP(i, j, k, 2))=lo(2)+delta(2)*i;
        }
    X.reshape(X.d0*X.d1*X.d2, 3);
    return X;
  }
  HALT("not implemented yet");

}

arr repmat(const arr& A, uint m, uint n) {
  CHECK(A.nd==1 || A.nd==2, "");
  arr B;
  B.referTo(A);
  if(B.nd==1) B.reshape(B.N, 1);
  arr z;
  z.resize(B.d0*m, B.d1*n);
  for(uint i=0; i<m; i++)
    for(uint j=0; j<n; j++)
      z.setMatrixBlock(B, i*B.d0, j*B.d1);
  return z;
}
arr rand(const uintA& d) {  arr z;  z.resize(d);  rndUniform(z, false); return z;  }
arr randn(const uintA& d) {  arr z;  z.resize(d);  rndGauss(z, 1., false);  return z;  }

arr diag(double d, uint n) {
  arr z;
  z.setDiag(d, n);
  return z;
}

void addDiag(arr& A, double d) {
  if(isRowShifted(A)) {
    rai::RowShifted* Aaux = (rai::RowShifted*) A.special;
    if(!Aaux->symmetric) HALT("this is not a symmetric matrix");
    for(uint i=0; i<A.d0; i++) A(i, 0) += d;
  } else {
    for(uint i=0; i<A.d0; i++) A(i, i) += d;
  }
}

/// make symmetric \f$A=(A+A^T)/2\f$
void makeSymmetric(arr& A) {
  CHECK(A.nd==2 && A.d0==A.d1, "not symmetric");
  uint n=A.d0, i, j;
  for(i=1; i<n; i++) for(j=0; j<i; j++) A(j, i) = A(i, j) = .5 * (A(i, j) + A(j, i));
}

/// make its transpose \f$A \gets A^T\f$
void transpose(arr& A) {
  CHECK(A.nd==2 && A.d0==A.d1, "not symmetric");
  uint n=A.d0, i, j;
  double z;
  for(i=1; i<n; i++) for(j=0; j<i; j++) { z=A(j, i); A(j, i)=A(i, j); A(i, j)=z; }
}

arr oneover(const arr& A) {
  arr B = A;
  for(double& b:B) b=1./b;
  return B;
}

namespace rai {
/// use this to turn on Lapack routines [default true if RAI_LAPACK is defined]
extern bool useLapack;
}

void normalizeWithJac(arr& y, arr& J) {
  double l = length(y);
  if(l<1e-10) {
    LOG(-1) <<"can't normalize vector of length " <<l;
  } else {
    y /= l;
    if(!!J && J.N) {
      J -= (y^y)*J;
      J /= l;
    }
  }
}

//===========================================================================
//
/// @name SVD etc
//

/// called from svd if RAI_LAPACK is not defined
uint own_SVD(
  arr& U,
  arr& w,
  arr& V,
  const arr& A,
  bool sort2Dpoints);

/** @brief Singular Value Decomposition (from Numerical Recipes);
  computes \f$U, D, V\f$ with \f$A = U D V^T\f$ from \f$A\f$ such that
  \f$U\f$ and \f$V\f$ are orthogonal and \f$D\f$ diagonal (the
  returned array d is 1-dimensional) -- uses LAPACK if RAI_LAPACK is
  defined */
uint svd(arr& U, arr& d, arr& V, const arr& A, bool sort) {
  uint r;
#ifdef RAI_LAPACK
  if(rai::useLapack) {
    r=lapack_SVD(U, d, V, A);
    V=~V;
  } else {
    r=own_SVD(U, d, V, A, sort);
  }
#else
  r=own_SVD(U, d, V, A, sort);
#endif

#ifdef RAI_CHECK_SVD
  bool uselapack=rai::useLapack;
  rai::useLapack=false;
  double err;
  arr dD, I;
  setDiagonal(dD, d);
  //cout <<U <<dD <<Vt;
  //Atmp = V * D * U;
  arr Atmp;
  Atmp = U * dD * ~V;
  //cout <<"\nA=" <<A <<"\nAtmp=" <<Atmp <<"U=" <<U <<"W=" <<dD <<"~V=" <<~V <<endl;
  std::cout <<"SVD is correct:  " <<(err=maxDiff(Atmp, A)) <<' ' <<endl;    CHECK(err<RAI_CHECK_SVD, "");
  if(A.d0<=A.d1) {
    I.setId(U.d0);
    std::cout <<"U is orthogonal: " <<(err=maxDiff(U * ~U, I)) <<' ' <<endl;  CHECK(err<RAI_CHECK_SVD, "");
    I.setId(V.d1);
    std::cout <<"V is orthogonal: " <<(err=maxDiff(~V * V, I)) <<endl;        CHECK(err<RAI_CHECK_SVD, "");
  } else {
    I.setId(U.d1);
    std::cout <<"U is orthogonal: " <<(err=maxDiff(~U * U, I)) <<' ' <<endl;  CHECK(err<RAI_CHECK_SVD, "");
    I.setId(V.d0);
    std::cout <<"V is orthogonal: " <<(err=sqrDistance(V * ~V, I)) <<endl;        CHECK(err<1e-5, "");
  }
  rai::useLapack=uselapack;
#endif

  return r;
}

/// gives a decomposition \f$A = U V^T\f$
void svd(arr& U, arr& V, const arr& A) {
  arr d, D;
  ::svd(U, d, V, A);
  D.resize(d.N, d.N); D=0.;
  for(uint i=0; i<d.N; i++) D(i, i)=::sqrt(d(i));
  U=U*D;
  V=V*D;
  //CHECK(maxDiff(A, U*~V) <1e-4, "");
}

void pca(arr& Y, arr& v, arr& W, const arr& X, uint npc) {
  CHECK(X.nd == 2 && X.d0 > 0 && X.d1 > 0, "Invalid data matrix X.");
  CHECK_LE(npc,  X.d1, "More principal components than data matrix X can offer.");

  if(npc == 0)
    npc = X.d1;

  // centering around the mean
  arr m = sum(X, 0) / (double)X.d0;
  arr D = X;
  for(uint i = 0; i < D.d0; i++)
    D[i]() -= m;

  arr U;
  svd(U, v, W, D, true);
  v = v % v;
  /*
  cout << "X: " << X << endl;
  cout << "D: " << D << endl;
  cout << "~D*D: " << ~D*D << endl;
  cout << "UU: " << U << endl;
  cout << "vv: " << v << endl;
  cout << "WW: " << W << endl;
  */

  W = W.cols(0, npc);
  Y = D * W;

  v *= 1./sum(v);
  v.sub(0, npc-1);
}

void check_inverse(const arr& Ainv, const arr& A) {
#ifdef RAI_CHECK_INVERSE
  arr D, _D; D.setId(A.d0);
  uint me;
  _D=A*Ainv;
  double err=maxDiff(_D, D, &me);
  cout <<"inverse is correct: " <<err <<endl;
  if(A.d0<10) {
    CHECK(err<RAI_CHECK_INVERSE, "inverting failed, error=" <<err <<" " <<_D.elem(me) <<"!=" <<D.elem(me) <<"\nA=" <<A <<"\nAinv=" <<Ainv <<"\nA*Ainv=" <<_D);
  } else {
    CHECK(err<RAI_CHECK_INVERSE, "inverting failed, error=" <<err <<" " <<_D.elem(me) <<"!=" <<D.elem(me));
  }
#endif
}

uint inverse(arr& Ainv, const arr& A) {
  uint r=inverse_SVD(Ainv, A);
  //rai::inverse_LU(inverse, A); return A.d0;
  return r;
}

/// calls inverse(B, A) and returns B
arr inverse(const arr& A) { arr B; inverse(B, A); return B; }

/// Pseudo Inverse based on SVD; computes \f$B\f$ such that \f$ABA = A\f$
uint inverse_SVD(arr& Ainv, const arr& A) {
  CHECK_EQ(A.nd, 2, "requires a matrix");
  unsigned i, j, k, m=A.d0, n=A.d1, r;
  arr U, V, w, winv;
  Ainv.resize(n, m);
  if(m==0 || n==0) return 0;
  if(m==n && m==1) { Ainv(0, 0)=1./A(0, 0); return 0; }
  if(m==n && m==2) { inverse2d(Ainv, A); return 0; }

  r=svd(U, w, V, A, true);

  //arr W;
  //setDiagonal(W, w);
  //CHECK(fabs(maxDiff(A, U*W*~V))<1e-10, "");

  winv.resizeAs(w);
  for(i=0; i<r; i++) {
    if(w(i)>1e-10) winv(i) = 1./w(i); else winv(i) = 1e10;
  }
  for(; i<w.N; i++) winv(i) = 0.;

#if 0
  //arr W;
  setDiagonal(W, winv);
  Ainv = V * W * ~U;
#else
  double* Ainvij=&Ainv(0, 0);
  for(i=0; i<n; i++) for(j=0; j<m; j++) {
      double* vi = &V(i, 0);
      double* uj = &U(j, 0);
      double  t  = 0.;
      for(k=0; k<w.N; k++) t += vi[k] * winv.p[k] * uj[k];
      *Ainvij = t;
      Ainvij++;
    }
#endif

#ifdef RAI_CHECK_INVERSE
  check_inverse(Ainv, A);
#endif
  return r;
}

void mldivide(arr& X, const arr& A, const arr& b) {
#ifdef RAI_LAPACK
  lapack_mldivide(X, A, b);
#else
  NIY;
#endif
}

void inverse_LU(arr& Xinv, const arr& X) {
  NIY;
#if 0
  CHECK(X.nd==2 && X.d0==X.d1, "");
  uint n=X.d0, i, j;
  Xinv.resize(n, n);
  if(n==0) return;
  if(n==n && n==1) { Xinv(0, 0)=1./X(0, 0); return; }
  if(n==n && n==2) { inverse2d(Xinv, X); return; }
  arr LU, piv;
  lapackLU(X, LU, piv);
  arr col(n);
  for(j=0; j<n; j++) {
    col.setZero();
    col(j)=1.0;
    lubksb(LU.pp, n, idx, col.p);
    for(i=0; i<n; i++) Xinv(i, j)=col(i);
  }

  delete[] idx;
  delete[] d;

#ifdef RAI_CHECK_INVERSE
  check_inverse(Xinv, X);
#endif
#endif
}

void inverse_SymPosDef(arr& Ainv, const arr& A) {
  CHECK_EQ(A.d0, A.d1, "");
#ifdef RAI_LAPACK
  lapack_inverseSymPosDef(Ainv, A);
#else
  inverse_SVD(Ainv, A);
#endif
#ifdef RAI_CHECK_INVERSE
  check_inverse(Ainv, A);
#endif
}

arr pseudoInverse(const arr& A, const arr& Winv, double eps) {
  arr AAt;
  arr At = ~A;
  if(!!Winv) {
    if(Winv.nd==1) AAt = A*(Winv%At); else AAt = A*Winv*At;
  } else AAt = A*At;
  if(eps) for(uint i=0; i<AAt.d0; i++) AAt(i, i) += eps;
  arr AAt_inv = inverse_SymPosDef(AAt);
  arr Ainv = At * AAt_inv;
  if(!!Winv) { if(Winv.nd==1) Ainv = Winv%Ainv; else Ainv = Winv*Ainv; }
  return Ainv;
}

/// the determinant of a 2D squared matrix
double determinant(const arr& A);

/** @brief the cofactor is the determinant of a 2D squared matrix after removing
  the ith row and the jth column */
double cofactor(const arr& A, uint i, uint j);

void gaussFromData(arr& a, arr& A, const arr& X) {
  CHECK_EQ(X.nd, 2, "");
  uint N=X.d0, n=X.d1;
  arr ones(N); ones=1.;
  a = ones*X/(double)N; a.reshape(n);
  A = (~X*X)/(double)N - (a^a);
}

/* compute a rotation matrix that rotates a onto v in arbitrary dimensions */
void rotationFromAtoB(arr& R, const arr& a, const arr& v) {
  CHECK_EQ(a.N, v.N, "");
  CHECK(fabs(1.-length(a))<1e-10 && fabs(1.-length(v))<1e-10, "");
  uint n=a.N, i, j;
  if(maxDiff(a, v)<1e-10) { R.setId(n); return; }  //nothing to rotate!!
  R.resize(n, n);
  //-- compute b orthogonal to a such that (a, b) span the rotation plane
  arr b;
  b = v - a*scalarProduct(a, v);
  b /= length(b);
  //-- compute rotation coefficients within the (a, b) plane, namely, R_2D=(v_a  -v_b ;  v_b  v_a)
  double v_a, v_b;
  v_a=scalarProduct(v, a);     //component along a
  v_b=scalarProduct(v, b);     //component along b
  //-- compute columns of R:
  arr x(n), x_res;
  double x_a, x_b;
  for(i=0; i<n; i++) {
    x.setZero(); x(i)=1.;       //x=i-th unit vector
    x_a=scalarProduct(x, a);     //component along a
    x_b=scalarProduct(x, b);     //component along b
    x_res = x - x_a*a - x_b*b;  //residual (rest) of the vector
    //rotated vector = residual + rotated a-component + rotated b-component
    x = x_res + (v_a*x_a-v_b*x_b)*a + (v_b*x_a+v_a*x_b)*b;
    for(j=0; j<n; j++) R(j, i)=x(j);  //store as column of the final rotation
  }
}

inline double RAI_SIGN_SVD(double a, double b) { return b>0 ? ::fabs(a) : -::fabs(a); }
#define RAI_max_SVD(a, b) ( (a)>(b) ? (a) : (b) )
#define RAI_SVD_MINVALUE .0 //1e-10

uint own_SVD(
  arr& U,
  arr& w,
  arr& V,
  const arr& A,
  bool sort) {
  //rai::Array<double*> Apointers, Upointers, Vpointers;
  unsigned m = A.d0; /* rows */
  unsigned n = A.d1; /* cols */
  U.resize(m, n);
  V.resize(n, n);
  w.resize(n);
  rai::Array<double*> Ap, Up, Vp;
  double** a = A.getCarray(Ap); //Pointers(Apointers); /* input matrix */
  double** u = U.getCarray(Up); //Pointers(Upointers); /* left vectors */
  double** v = V.getCarray(Vp); //Pointers(Vpointers); /* right vectors */

  int flag;
  unsigned i, its, j, jj, k, l, nm(0), r;
  double anorm, c, f, g, h, s, scale, x, y, z, t;

  arr rv1(n);

  /* copy A to U */
  for(i=0; i<m; i++) for(j=0; j<n; j++) u[i][j] = a[i][j];

  /* householder reduction to pickBiagonal form */
  g = scale = anorm = 0.0;

  for(i=0; i<n; i++) {
    l = i + 1;
    rv1(i) = scale * g;
    g = s = scale = 0.0;

    if(i<m) {
      for(k=i; k<m; k++) scale += fabs(u[k][i]);

      if(scale!=0.0) {
        for(k=i; k<m; k++) {
          u[k][i] /= scale;
          s += u[k][i] * u[k][i];
        }

        f = u[i][i];
        g = -RAI_SIGN_SVD(sqrt(s), f);
        h = f * g - s;
        u[i][i] = f - g;

        for(j=l; j<n; j++) {
          s = 0.0;
          for(k=i; k<m; k++) s += u[k][i] * u[k][j];

          f = s / h;
          for(k=i; k<m; k++) u[k][j] += f * u[k][i];
        }

        for(k=i; k<m; k++) u[k][i] *= scale;
      }
    }

    w(i) = scale * g;
    g = s = scale = 0.0;

    if(i<m && i!=n-1) {
      for(k=l; k<n; k++)scale += fabs(u[i][k]);

      if(scale!=0.0) {
        for(k=l; k<n; k++) {
          u[i][k] /= scale;
          s += u[i][k] * u[i][k];
        }

        f = u[i][l];
        g = -RAI_SIGN_SVD(sqrt(s), f);
        h = f * g - s;
        u[i][l] = f - g;

        for(k=l; k<n; k++) rv1(k) = u[i][k] / h;

        for(j=l; j<m; j++) {
          s = 0.0;
          for(k=l; k<n; k++) s += u[j][k] * u[i][k];

          for(k=l; k<n; k++) u[j][k] += s * rv1(k);
        }

        for(k=l; k<n; k++) u[i][k] *= scale;
      }
    }

    anorm = RAI_max_SVD(anorm, fabs(w(i)) + fabs(rv1(i)));
  }

  /* accumulation of right-hand transformations */
  for(l=i=n; i--; l--) {
    if(l<n) {
      if(g!=0.0) {
        /* double division avoids possible underflow */
        for(j=l; j<n; j++) v[j][i] = (u[i][j] / u[i][l]) / g;

        for(j=l; j<n; j++) {
          s = 0.0;
          for(k=l; k<n; k++) s += u[i][k] * v[k][j];

          for(k=l; k<n; k++) v[k][j] += s * v[k][i];
        }
      }

      for(j=l; j<n; j++) v[i][j] = v[j][i] = 0.0;
    }

    v[i][i] = 1.0;
    g = rv1(i);
  }

  /* accumulation of left-hand transformations */
  for(l=i=(m<n?m:n); i--; l--) {
    g = w(i);

    for(j=l; j<n; j++) u[i][j] = 0.0;

    if(g!=0.0) {
      g = 1.0 / g;

      for(j=l; j<n; j++) {
        s = 0.0;
        for(k=l; k<m; k++) s += u[k][i] * u[k][j];

        /* double division avoids possible underflow */
        f = (s / u[i][i]) * g;

        for(k=i; k<m; k++) u[k][j] += f * u[k][i];
      }

      for(j=i; j<m; j++) u[j][i] *= g;
    } else {
      for(j=i; j<m; j++) u[j][i] = 0.0;
    }

    u[i][i]++;
  }

  /* diagonalization of the pickBiagonal form */
  for(k=n; k--;) {
    for(its=1; its<=30; its++) {
      flag = 1;

      /* test for splitting */
      for(l = k + 1; l--;) {
        /* rv1 [0] is always zero, so there is no exit */
        nm = l - 1;

        if(fabs(rv1(l)) + anorm == anorm) {
          flag = 0;
          break;
        }

        //if(!l) break; //(mt 07-01-16)
        if(fabs(w(nm)) + anorm == anorm) break;
      }

      if(flag) {
        /* cancellation of rv1 [l] if l greater than 0 */
        c = 0.0;
        s = 1.0;

        for(i=l; i<=k; i++) {
          f = s * rv1(i);
          rv1(i) *= c;

          if(fabs(f) + anorm == anorm) break;

          g = w(i);
          h = hypot(f, g);
          w(i) = h;
          h = 1.0 / h;
          c = g * h;
          s = -f * h;

          for(j=0; j<m; j++) {
            y = u[j][nm];
            z = u[j][i];
            u[j][nm] = y * c + z * s;
            u[j][i] = z * c - y * s;
          }
        }
      }

      /* test for convergence */
      z = w(k);

      if(l==k) {
        if(z<0.0) {
          w(k) = -z;
          for(j=0; j<n; j++) v[j][k] = -v[j][k];
        }
        break;
      }

      if(its==50) HALT("svd failed");
      //if(its==30) throw k;

      /* shift from bottom 2 by 2 minor */
      x = w(l);
      nm = k - 1;
      y = w(nm);
      g = rv1(nm);
      h = rv1(k);
      f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
      g = hypot(f, 1.0);
      f = ((x - z) * (x + z) + h * ((y / (f + RAI_SIGN_SVD(g, f))) - h)) / x;

      /* next qr transformation */
      c = s = 1.0;

      for(j=l; j<k; j++) {
        i = j + 1;
        g = rv1(i);
        y = w(i);
        h = s * g;
        g *= c;
        z = hypot(f, h);
        rv1(j) = z;
        c = f / z;
        s = h / z;
        f = x * c + g * s;
        g = g * c - x * s;
        h = y * s;
        y *= c;

        for(jj=0; jj<n; jj++) {
          x = v[jj][j];
          z = v[jj][i];
          v[jj][j] = x * c + z * s;
          v[jj][i] = z * c - x * s;
        }

        z = hypot(f, h);
        w(j) = z;

        /* rotation can be arbitrary if z is zero */
        if(z!=0.0) {
          z = 1.0 / z;
          c = f * z;
          s = h * z;
        }

        f = c * g + s * y;
        x = c * y - s * g;

        for(jj=0; jj<m; jj++) {
          y = u[jj][j];
          z = u[jj][i];
          u[jj][j] = y * c + z * s;
          u[jj][i] = z * c - y * s;
        }
      }

      rv1(l) = 0.0;
      rv1(k) = f;
      w(k) = x;
    }
  }

  //sorting:
  if(sort) {
    unsigned i, j, k;
    double   p;

    for(i=0; i<n-1; i++) {
      p = w(k=i);

      for(j=i+1; j<n; j++) if(w(j)>=p) p = w(k=j);

      if(k!=i) {
        w(k) = w(i);
        w(i) = p;

        for(j=0; j<n; j++) {
          p       = v[j][i];
          v[j][i] = v[j][k];
          v[j][k] = p;
        }

        for(j=0; j<m; j++) {
          p       = u[j][i];
          u[j][i] = u[j][k];
          u[j][k] = p;
        }
      }
    }
  }

  //rank analysis

  for(r=0; r<n && w(r)>RAI_SVD_MINVALUE; r++) {};

  t = r < n ? fabs(w(n-1)) : 0.0;
  r = 0;
  s = 0.0;
  while(r<n && w(r)>t && w(r)+s>s) s += w(r++);

  return r;
}

double determinantSubroutine(double** A, uint n) {
  if(n==1) return A[0][0];
  if(n==2) return A[0][0]*A[1][1]-A[0][1]*A[1][0];
  uint i, j;
  double d=0;
  double** B=new double*[n-1];
  for(i=0; i<n; i++) {
    for(j=0; j<n; j++) {
      if(j<i) B[j]=&A[j][1];
      if(j>i) B[j-1]=&A[j][1];
    }
    d+=((i&1)?-1.:1.) * A[i][0] * determinantSubroutine(B, n-1);
  }
  delete[] B; B=nullptr;
  return d;
}

double determinant(const arr& A) {
  CHECK(A.nd==2 && A.d0==A.d1, "determinants require a squared 2D matrix");
  rai::Array<double*> tmp;
  return determinantSubroutine(A.getCarray(tmp), A.d0);
}

double cofactor(const arr& A, uint i, uint j) {
  CHECK(A.nd==2 && A.d0==A.d1, "determinants require a squared 2D matrix");
  arr B=A;
  B.delRows(i);
  B.delColumns(j, 1);
  return ((i&1)^(j&1)?-1.:1) * determinant(B);
}

/** Given a distribution p over a discrete domain {0, .., p.N-1}
    Stochastic Universal Sampling draws n samples from this
    distribution, stored as integers in s */
uintA sampleMultinomial_SUS(const arr& p, uint n) {
  //following T. Baeck "EA in Theo. and Prac." p120
  uintA s(n);
  double sum=0, ptr=rnd.uni();
  uint i, j=0;
  for(i=0; i<p.N; i++) {
    sum+=p(i)*n;
    while(sum>ptr) { s(j)=i; j++; ptr+=1.; }
  }
  //now, 'sum' should = 'n' and 'ptr' has been 'n'-times increased -> 'j=n'
  CHECK_EQ(j, n, "error in rnd::sampleMultinomial_SUS(p, n) -> p not normalized?");
  return s;
}

uint sampleMultinomial(const arr& p) {
  double sum=0, ptr=rnd.uni();
  uint i;
  for(i=0; i<p.N; i++) {
    sum+=p(i);
    if(sum>ptr) return i;
  }
  HALT("error in rnd::sampleMultinomial(p) -> p not normalized? " <<p);
  return 0;
}

/// calls gnuplot to display the (n, 2) or (n, 3) array (n=number of points of line or surface)
void gnuplot(const arr& X, bool pauseMouse, bool persist, const char* PDFfile) {
  rai::arrayBrackets="  ";
  if(X.nd==2 && X.d1!=2) {  //assume array -> splot
    FILE("z.pltX") <<X;
    gnuplot("splot 'z.pltX' matrix with pm3d, 'z.pltX' matrix with lines", pauseMouse, persist, PDFfile);
    return;
  }
  if(X.nd==2 && X.d1==2) {  //assume curve -> plot
    FILE("z.pltX") <<X;
    gnuplot("plot 'z.pltX' us 1:2", pauseMouse, persist, PDFfile);
    return;
  }
  if(X.nd==1) {  //assume curve -> plot
    arr Y;
    Y.referTo(X);
    Y.reshape(Y.N, 1);
    FILE("z.pltX") <<Y;
    gnuplot("plot 'z.pltX' us 1", pauseMouse, persist, PDFfile);
    return;
  }
}

arr bootstrap(const arr& x) {
  arr y(x.N);
  for(uint i=0; i<y.N; i++) y(i) = x(rnd(y.N));
  return y;
}

void write(const arrL& X, const char* filename, const char* ELEMSEP, const char* LINESEP, const char* BRACKETS, bool dimTag, bool binary) {
  std::ofstream fil;
  rai::open(fil, filename);
  catCol(X).write(fil, ELEMSEP, LINESEP, BRACKETS, dimTag, binary);
  fil.close();
}

//===========================================================================
//
/// @name simple image formats
//

/** save data as ppm or pgm. Images are (height, width, [0, 2, 3, 4])-dim
  byte arrays, where the 3rd dimension determines whether it's a grey
  (0), grey-alpha (2), RGB (3), or RGBA (4) image */
void write_ppm(const byteA& img, const char* file_name, bool swap_rows) {
  if(!img.N) RAI_MSG("empty image");
  CHECK(img.nd==2 || (img.nd==3 && img.d2==3), "only rgb or gray images to ppm");
  ofstream os;
  os.open(file_name, std::ios::out | std::ios::binary);
  if(!os.good()) HALT("could not open file `" <<file_name <<"' for output");
  switch(img.d2) {
    case 0:  os <<"P5 " <<img.d1 <<' ' <<img.d0 <<" 255\n";  break; //PGM
    case 3:  os <<"P6 " <<img.d1 <<' ' <<img.d0 <<" 255\n";  break; //PPM
    default: NIY;
  }
  if(!swap_rows) {
    os.write((char*)img.p, img.N);
  } else {
    if(img.d2)
      for(uint i=img.d0; i--;) os.write((char*)&img(i, 0, 0), img.d1*img.d2);
    else
      for(uint i=img.d0; i--;) os.write((char*)&img(i, 0), img.d1);
  }
}

/** read data from an ppm or pgm file */
void read_ppm(byteA& img, const char* file_name, bool swap_rows) {
  uint mode, width, height, max;
  ifstream is;
  is.open(file_name, std::ios::in | std::ios::binary);
  if(!is.good()) HALT("could not open file `" <<file_name <<"' for input");
  if(is.get()!='P') HALT("NO PPM FILE:" <<file_name);
  is >>mode;
  if(rai::peerNextChar(is)=='#') rai::skipRestOfLine(is);
  is >>width >>height >>max;
  is.get(); //MUST be a white character if everything went ok
  switch(mode) {
    case 5:  img.resize(height, width);    break; //PGM
    case 6:  img.resize(height, width, 3);  break; //PPM
  }
  if(!swap_rows) {
    is.read((char*)img.p, img.N);
  } else {
    for(uint i=img.d0; i--;) is.read((char*)&img(i, 0, 0), img.d1*img.d2);
  }
}

/// add an alpha channel to an image array
void add_alpha_channel(byteA& img, byte alpha) {
  uint w=img.d1, h=img.d0;
  img.reshape(h*w, 3);
  img.insColumns(3, 1);
  for(uint i=0; i<img.d0; i++) img(i, 3)=alpha;
  img.reshape(h, w, 4);
}

/// add an alpha channel to an image array
void remove_alpha_channel(byteA& img) {
  uint w=img.d1, h=img.d0;
  img.reshape(h*w, 4);
  img.delColumns(3, 1);
  img.reshape(h, w, 3);
}

void image_halfResolution(byteA& img) {
  byteA org = img;
  img.resize(org.d0/2, org.d1/2, org.d2);
  for(uint i=0; i<img.d0; i++) for(uint j=0; j<img.d1; j++) for(uint k=0; k<img.d2; k++) {
        float v = (float)org(2*i, 2*j, k) + (float)org(2*i, 2*j+1, k)
                  + (float)org(2*i+1, 2*j, k) +(float)org(2*i+1, 2*j+1, k);
        v /= 4;
        img(i, j, k) = (byte)v;
      }
}

void flip_image(byteA& img) {
  if(!img.N) return;
  uint h=img.d0, n=img.N/img.d0;
  byteA line(n);
  byte* a, *b, *c;
  for(uint i=0; i<h/2; i++) {
    a=img.p+i*n;
    b=img.p+(h-1-i)*n;
    c=line.p;
    memmove(c, a, n);
    memmove(a, b, n);
    memmove(b, c, n);
  }
}

void flip_image(floatA& img) {
  if(!img.N) return;
  uint h=img.d0, n=img.N/img.d0;
  floatA line(n);
  float* a, *b, *c;
  uint s=sizeof(float);
  for(uint i=0; i<h/2; i++) {
    a=img.p+i*n;
    b=img.p+(h-1-i)*n;
    c=line.p;
    memmove(c, a, n*s);
    memmove(a, b, n*s);
    memmove(b, c, n*s);
  }
}

/// make grey scale image
void make_grey(byteA& img) {
  CHECK(img.nd==3 && (img.d2==3 || img.d1==4), "makeGray requires color image as input");
  byteA tmp;
  tmp.resize(img.d0, img.d1);
  for(uint i=0; i<img.d0; i++) for(uint j=0; j<img.d1; j++) {
      tmp(i, j) = ((uint)img(i, j, 0) + img(i, j, 1) + img(i, j, 2))/3;
    }
  img=tmp;
}

/// make a grey image and RGA image
void make_RGB(byteA& img) {
  CHECK_EQ(img.nd, 2, "make_RGB requires grey image as input");
  byteA tmp;
  tmp.resize(img.d0, img.d1, 3);
  for(uint i=0; i<img.d0; i++) for(uint j=0; j<img.d1; j++) {
      tmp(i, j, 0) = img(i, j);
      tmp(i, j, 1) = img(i, j);
      tmp(i, j, 2) = img(i, j);
    }
  img=tmp;
}

/// make a grey image and RGA image
void make_RGB2BGRA(byteA& img) {
  CHECK(img.nd==3 && img.d2==3, "make_RGB2RGBA requires color image as input");
  byteA tmp;
  tmp.resize(img.d0, img.d1, 4);
  for(uint i=0; i<img.d0; i++) for(uint j=0; j<img.d1; j++) {
      tmp(i, j, 0) = img(i, j, 2);
      tmp(i, j, 1) = img(i, j, 1);
      tmp(i, j, 2) = img(i, j, 0);
      tmp(i, j, 3) = 255;
    }
  img=tmp;
}

/// make a grey image and RGA image
void swap_RGB_BGR(byteA& img) {
  CHECK(img.nd==3 && img.d2==3, "make_RGB2RGBA requires color image as input");
  byte* b=img.p, *bstop=img.p+img.N;
  byte z;
  for(; b<bstop; b+=3) {
    z=b[0]; b[0]=b[2]; b[2]=z;
  }
}

uintA getIndexTuple(uint i, const uintA& d) {
  CHECK(i<product(d), "out of range");
  uintA I(d.N);
  I.setZero();
  for(uint j=d.N; j--;) {
    I.p[j] = i%d.p[j];
    i -= I.p[j];
    i /= d.p[j];
  }
  return I;
}

void lognormScale(arr& P, double& logP, bool force) {
#ifdef RAI_NoLognormScale
  return;
#endif
  double Z=0.;
  for(uint i=0; i<P.N; i++) Z += fabs(P.elem(i));
  if(!force && Z>1e-3 && Z<1e3) return;
  if(fabs(Z-1.)<1e-10) return;
  if(Z>1e-100) {
    logP+=::log(Z);
    P/=Z;
  } else {
    logP+=::log(Z);
    P=1.;
    RAI_MSG("ill-conditioned table factor for norm scaling");
  }
}

void sparseProduct(arr& y, arr& A, const arr& x) {
  if(!A.special && !x.special) {
    innerProduct(y, A, x);
    return;
  }
#if 0
  NIY; //replace by Eigen
#else
  if(isSparseMatrix(A) && !isSparseVector(x)) {
    uint i, j;
    int* k, *kstop;
    y.resize(A.d0); y.setZero();
    double* Ap=A.p;
    intA& A_elems = dynamic_cast<rai::SparseMatrix*>(A.special)->elems;
    for(k=A_elems.p, kstop=A_elems.p+A_elems.N; k!=kstop; Ap++) {
      i=*k; k++;
      j=*k; k++;
      y.p[i] += (*Ap) * x.p[j];
    }
    return;
  }
  if(isSparseMatrix(A) && isSparseVector(x)) {
    A.sparse().setupRowsCols();
    rai::SparseVector* sx = dynamic_cast<rai::SparseVector*>(x.special);
    CHECK(x.nd==1 && A.nd==2 && x.d0==A.d1, "not a proper matrix-vector multiplication");
    uint i, j, n;
    int* k, *kstop;
    uint* l, *lstop;
    y.sparseVec();
    y.d0 = A.d0;
    intA& y_elems= dynamic_cast<rai::SparseVector*>(y.special)->elems;
    double* xp=x.p;
    intA& x_elems = sx->elems;
    for(k=x_elems.p, kstop=x_elems.p+x_elems.N; k!=kstop; xp++) {
      j=*k; k++;
      uintA& A_col = dynamic_cast<rai::SparseMatrix*>(A.special)->cols(j);
      for(l=A_col.p, lstop=A_col.p+A_col.N; l!=lstop;) {
        i =*l; l++;
        n =*l; l++;
#if 0
        slot=&y_col(i);
        if(*slot==(uint)-1) {
          *slot=y.N;
          y.resizeMEM(y.N+1, true); y(y.N-1)=0.;
          y_elems.append(i);
          CHECK_EQ(y_elems.N, y.N, "");
        }
        i=*slot;
        y(i) += A.elem(n) * (*xp);
#else
        double a = A.elem(n) * (*xp);
        y_elems.append(i);
        y.resizeMEM(y.N+1, true);
        y.elem(y.N-1)=a;
#endif
      }
    }
    return;
  }
  if(!isSparseMatrix(A) && isSparseVector(x)) {
    uint i, j, d1=A.d1;
    int* k, *kstop;
    y.resize(A.d0); y.setZero();
    double* xp=x.p;
    intA& elems = dynamic_cast<rai::SparseMatrix*>(x.special)->elems;
    for(k=elems.p, kstop=elems.p+elems.N; k!=kstop; xp++) {
      j=*k; k++;
      for(i=0; i<A.d0; i++) {
        y.p[i] += A.p[i*d1+j] * (*xp);
      }
    }
    return;
  }
#endif
}

void scanArrFile(const char* name) {
  ifstream is(name, std::ios::binary);
  CHECK(is.good(), "couldn't open file " <<name);
  arr x;
  rai::String tag;
  for(;;) {
    tag.read(is, " \n\r\t", " \n\r\t");
    if(!is.good() || tag.N==0) return;
    x.readTagged(is, nullptr);
    x.writeTagged(cout, tag);  cout <<endl;
    if(!is.good()) return;
  }
}

#ifndef CHECK_EPS
#  define CHECK_EPS 1e-8
#endif

/// numeric (finite difference) computation of the gradient
arr finiteDifferenceGradient(const ScalarFunction& f, const arr& x, arr& Janalytic) {
  arr dx, J;
  double y, dy;
  y=f(Janalytic, NoArr, x);

  J.resize(x.N);
  double eps=CHECK_EPS;
  uint i;
  for(i=0; i<x.N; i++) {
    dx=x;
    dx.elem(i) += eps;
    dy = f(NoArr, NoArr, dx);
    dy = (dy-y)/eps;
    J(i)=dy;
  }
  return J;
}

/// numeric (finite difference) computation of the gradient
arr finiteDifferenceJacobian(const VectorFunction& f, const arr& _x, arr& Janalytic) {
  arr x=_x;
  arr y, dx, dy, J;
  f(y, Janalytic, x);
  if(isRowShifted(Janalytic)
      || isSparseMatrix(Janalytic)) {
    Janalytic = unpack(Janalytic);
  }

  J.resize(y.N, x.N);
  double eps=CHECK_EPS;
  uint i, k;
  for(i=0; i<x.N; i++) {
    dx=x;
    dx.elem(i) += eps;
    f(dy, NoArr, dx);
    dy = (dy-y)/eps;
    for(k=0; k<y.N; k++) J(k, i)=dy.elem(k);
  }
  J.reshapeAs(Janalytic);
  return J;
}

/// numeric (finite difference) check of the gradient of f at x
bool checkGradient(const ScalarFunction& f,
                   const arr& x, double tolerance, bool verbose) {
  arr J;
  arr JJ = finiteDifferenceGradient(f, x, J);
  uint i;
  double md=maxDiff(J, JJ, &i);
  if(md>tolerance && md>fabs(J.elem(i))*tolerance) {
    RAI_MSG("checkGradient -- FAILURE -- max diff=" <<md <<" |"<<J.elem(i)<<'-'<<JJ.elem(i)<<"| (stored in files z.J_*)");
    J >>FILE("z.J_analytical");
    JJ >>FILE("z.J_empirical");
    //cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
    //HALT("");
    return false;
  } else {
    cout <<"checkGradient -- SUCCESS (max diff error=" <<md <<")" <<endl;
  }
  return true;
}

bool checkHessian(const ScalarFunction& f, const arr& x, double tolerance, bool verbose) {
  arr g, H, dx, dy, Jg;
  f(g, H, x);
  if(isRowShifted(H)) H = unpack(H);

  Jg.resize(g.N, x.N);
  double eps=CHECK_EPS;
  uint i, k;
  for(i=0; i<x.N; i++) {
    dx=x;
    dx.elem(i) += eps;
    f(dy, NoArr, dx);
    dy = (dy-g)/eps;
    for(k=0; k<g.N; k++) Jg(k, i)=dy.elem(k);
  }
  Jg.reshapeAs(H);
  double md=maxDiff(H, Jg, &i);
  //   J >>FILE("z.J");
  //   JJ >>FILE("z.JJ");
  if(md>tolerance) {
    RAI_MSG("checkHessian -- FAILURE -- max diff=" <<md <<" |"<<H.elem(i)<<'-'<<Jg.elem(i)<<"| (stored in files z.J_*)");
    H >>FILE("z.J_analytical");
    Jg >>FILE("z.J_empirical");
    //cout <<"\nmeasured grad=" <<JJ <<"\ncomputed grad=" <<J <<endl;
    //HALT("");
    return false;
  } else {
    cout <<"checkHessian -- SUCCESS (max diff error=" <<md <<")" <<endl;
  }
  return true;
}

bool checkJacobian(const VectorFunction& f,
                   const arr& x, double tolerance, bool verbose) {
  arr J;
  arr JJ = finiteDifferenceJacobian(f, x, J);
  uint i;
  double md=maxDiff(J, JJ, &i);
  if(md>tolerance && md>fabs(J.elem(i))*tolerance) {
    RAI_MSG("checkJacobian -- FAILURE -- max diff=" <<md <<" |"<<J.elem(i)<<'-'<<JJ.elem(i)<<"| (stored in files z.J_*)");
    J >>FILE("z.J_analytical");
    JJ >>FILE("z.J_empirical");
    if(verbose) {
      cout <<"J_analytical = " <<J
           <<"\nJ_empirical  = " <<JJ <<endl;
    }
    return false;
  } else {
    cout <<"checkJacobian -- SUCCESS (max diff error=" <<md <<")" <<endl;
  }
  return true;
}

#define EXP ::exp //rai::approxExp

double NNinv(const arr& a, const arr& b, const arr& Cinv) {
  double d=sqrDistance(Cinv, a, b);
  double norm = ::sqrt(lapack_determinantSymPosDef((1./RAI_2PI)*Cinv));
  return norm*EXP(-.5*d);
}
double logNNinv(const arr& a, const arr& b, const arr& Cinv) {
  NIY;
  return 1;
  /*
  arr d=a-b;
  double norm = ::sqrt(fabs(rai::determinant_LU((1./RAI_2PI)*Cinv)));
  return ::log(norm) + (-.5*scalarProduct(Cinv, d, d));
  */
}
double logNNprec(const arr& a, const arr& b, double prec) {
  uint n=a.N;
  arr d=a-b;
  double norm = pow(prec/RAI_2PI, .5*n);
  return ::log(norm) + (-.5*prec*scalarProduct(d, d));
}
double logNN(const arr& a, const arr& b, const arr& C) {
  arr Cinv;
  inverse_SymPosDef(Cinv, C);
  return logNNinv(a, b, Cinv);
}
double NN(const arr& a, const arr& b, const arr& C) {
  arr Cinv;
  inverse_SymPosDef(Cinv, C);
  return NNinv(a, b, Cinv);
}
/// non-normalized!! Gaussian function (f(0)=1)
double NNNNinv(const arr& a, const arr& b, const arr& Cinv) {
  double d=sqrDistance(Cinv, a, b);
  return EXP(-.5*d);
}
double NNNN(const arr& a, const arr& b, const arr& C) {
  arr Cinv;
  inverse_SymPosDef(Cinv, C);
  return NNNNinv(a, b, Cinv);
}
double NNzeroinv(const arr& x, const arr& Cinv) {
  double norm = ::sqrt(lapack_determinantSymPosDef((1./RAI_2PI)*Cinv));
  return norm*EXP(-.5*scalarProduct(Cinv, x, x));
}
/// gradient of a Gaussian
double dNNinv(const arr& x, const arr& a, const arr& Ainv, arr& grad) {
  double y=NNinv(x, a, Ainv);
  grad = y * Ainv * (a-x);
  return y;
}
/// gradient of a non-normalized Gaussian
double dNNNNinv(const arr& x, const arr& a, const arr& Ainv, arr& grad) {
  double y=NNNNinv(x, a, Ainv);
  grad = y * Ainv * (a-x);
  return y;
}
double NNsdv(const arr& a, const arr& b, double sdv) {
  double norm = 1./(::sqrt(RAI_2PI)*sdv);
  return norm*EXP(-.5*sqrDistance(a, b)/(sdv*sdv));
}
double NNzerosdv(const arr& x, double sdv) {
  double norm = 1./(::sqrt(RAI_2PI)*sdv);
  return norm*EXP(-.5*sumOfSqr(x)/(sdv*sdv));
}

rai::String singleString(const StringA& strs) {
  rai::String s;
  for(const rai::String& str:strs) {
    if(s.N) s<<"_";
    s<<str;
  }
  return s;
}

//===========================================================================
//
// LAPACK
//

// file:///usr/share/doc/liblapack-doc/lug/index.html

#ifdef RAI_LAPACK
#if 1 //def NO_BLAS
void blas_MM(arr& X, const arr& A, const arr& B) {       rai::useLapack=false; innerProduct(X, A, B); rai::useLapack=true; };
void blas_MsymMsym(arr& X, const arr& A, const arr& B) { rai::useLapack=false; innerProduct(X, A, B); rai::useLapack=true; };
void blas_Mv(arr& y, const arr& A, const arr& x) {       rai::useLapack=false; innerProduct(y, A, x); rai::useLapack=true; };
void blas_A_At(arr& X, const arr& A) { X = A*~A; }
void blas_At_A(arr& X, const arr& A) { X = ~A*A; }
#else
void blas_MM(arr& X, const arr& A, const arr& B) {
  CHECK_EQ(A.d1, B.d0, "matrix multiplication: wrong dimensions");
  X.resize(A.d0, B.d1);
  cblas_dgemm(CblasRowMajor,
              CblasNoTrans, CblasNoTrans,
              A.d0, B.d1, A.d1,
              1., A.p, A.d1,
              B.p, B.d1,
              0., X.p, X.d1);
#if 0//test
  rai::useLapack=false;
  std::cout  <<"blas_MM error = " <<maxDiff(A*B, X, 0) <<std::endl;
  rai::useLapack=true;
#endif
}

void blas_A_At(arr& X, const arr& A) {
  uint n=A.d0;
  CHECK(n, "blas doesn't like n=0 !");
  X.resize(n, n);
  cblas_dsyrk(CblasRowMajor, CblasUpper, CblasNoTrans,
              X.d0, A.d1,
              1.f, A.p, A.d1,
              0., X.p, X.d1);
  for(uint i=0; i<n; i++) for(uint j=0; j<i; j++) X.p[i*n+j] = X.p[j*n+i]; //fill in the lower triangle
#if 0//test
  rai::useLapack=false;
  std::cout  <<"blas_MM error = " <<maxDiff(A*~A, X, 0) <<std::endl;
  rai::useLapack=true;
#endif
}

void blas_At_A(arr& X, const arr& A) {
  uint n=A.d1;
  CHECK(n, "blas doesn't like n=0 !");
  X.resize(n, n);
  cblas_dsyrk(CblasRowMajor, CblasUpper, CblasTrans,
              X.d0, A.d0,
              1.f, A.p, A.d1,
              0., X.p, X.d1);
  for(uint i=0; i<n; i++) for(uint j=0; j<i; j++) X.p[i*n+j] = X.p[j*n+i]; //fill in the lower triangle
#if 0//test
  rai::useLapack=false;
  std::cout  <<"blas_MM error = " <<maxDiff(~A*A, X, 0) <<std::endl;
  rai::useLapack=true;
#endif
}

void blas_Mv(arr& y, const arr& A, const arr& x) {
  CHECK_EQ(A.d1, x.N, "matrix multiplication: wrong dimensions");
  y.resize(A.d0);
  if(!x.N && !A.d1) { y.setZero(); return; }
  cblas_dgemv(CblasRowMajor,
              CblasNoTrans,
              A.d0, A.d1,
              1., A.p, A.d1,
              x.p, 1,
              0., y.p, 1);
#if 0 //test
  rai::useLapack=false;
  std::cout  <<"blas_Mv error = " <<maxDiff(A*x, y, 0) <<std::endl;
  rai::useLapack=true;
#endif
}

void blas_MsymMsym(arr& X, const arr& A, const arr& B) {
  CHECK_EQ(A.d1, B.d0, "matrix multiplication: wrong dimensions");
  X.resize(A.d0, B.d1);
  cblas_dsymm(CblasRowMajor,
              CblasLeft, CblasUpper,
              A.d0, B.d1,
              1., A.p, A.d1,
              B.p, B.d1,
              0., X.p, X.d1);
#if 0 //test
  arr Y(A.d0, B.d1);
  uint i, j, k;
  Y.setZero();
  for(i=0; i<Y.d0; i++) for(j=0; j<Y.d1; j++) for(k=0; k<A.d1; k++)
        Y(i, j) += A(i, k) * B(k, j);
  std::cout  <<"blas_MsymMsym error = " <<sqrDistance(X, Y) <<std::endl;
#endif
}

#endif //RAI_NOBLAS

arr lapack_Ainv_b_sym(const arr& A, const arr& b) {
  if(isSparseMatrix(A)) {
    return eigen_Ainv_b(A, b);
  }
  arr x;
  if(b.nd==2) { //b is a matrix (unusual) repeat for each col:
    RAI_MSG("TODO: directly call lapack with the matrix!")
    arr bT = ~b;
    x.resizeAs(bT);
    for(uint i=0; i<bT.d0; i++) x[i]() = lapack_Ainv_b_sym(A, bT[i]);
    x=~x;
    return x;
  }
  integer N=A.d0, KD=0, NRHS=1, LDAB=0, INFO;
  if(isRowShifted(A)) {
    rai::RowShifted* Aaux = (rai::RowShifted*) A.special;
    if(!Aaux->symmetric) HALT("this is not a symmetric matrix");
    for(uint i=0; i<A.d0; i++) if(Aaux->rowShift(i)!=i) HALT("this is not shifted as an upper triangle");
    KD=Aaux->rowSize-1;
    LDAB=Aaux->rowSize;
  }
  x=b;
  arr Acol=A;
  try {
    if(!isRowShifted(A)) {
      dposv_((char*)"L", &N, &NRHS, Acol.p, &N, x.p, &N, &INFO);
    } else {
      //assumes symmetric and upper banded
      dpbsv_((char*)"L", &N, &KD, &NRHS, Acol.p, &LDAB, x.p, &N, &INFO);
    }
  } catch(...) {
    HALT("here");
  }
  if(INFO) {
#if 0
    uint k=(N>3?3:N); //number of required eigenvalues
    rai::Array<integer> IWORK(5*N), IFAIL(N);
    arr WORK(10*(3*N)), Acopy=A;
    integer M, IL=1, IU=k, LDQ=0, LDZ=1, LWORK=WORK.N;
    double VL=0., VU=0., ABSTOL=1e-8;
    arr sig(N);
    if(!isSpecial(A)) {
//      sig.resize(N);
//      dsyev_ ((char*)"N", (char*)"L", &N, A.p, &N, sig.p, WORK.p, &LWORK, &INFO);
//      lapack_EigenDecomp(A, sig, NoArr);
      dsyevx_((char*)"N", (char*)"I", (char*)"L", &N, Acopy.p, &LDAB, &VL, &VU, &IL, &IU, &ABSTOL, &M, sig.p, (double*)nullptr, &LDZ, WORK.p, &LWORK, IWORK.p, IFAIL.p, &INFO);
    } else if(isRowShifted(A)) {
      dsbevx_((char*)"N", (char*)"I", (char*)"L", &N, &KD, Acopy.p, &LDAB, (double*)nullptr, &LDQ, &VL, &VU, &IL, &IU, &ABSTOL, &M, sig.p, (double*)nullptr, &LDZ, WORK.p, IWORK.p, IFAIL.p, &INFO);
    } else NIY;
    sig.resizeCopy(k);
//    arr sig, eig;
//    lapack_EigenDecomp(A, sig, eig);
#endif
    rai::errString <<"lapack_Ainv_b_sym error info = " <<INFO
                   <<". Typically this is because A is not pos-def.";
//    \nsmallest "<<k<<" eigenvalues=" <<sig;
    throw(rai::errString.p);
//    THROW("lapack_Ainv_b_sym error info = " <<INFO
//         <<". Typically this is because A is not pos-def.\nsmallest "<<k<<" eigenvalues=" <<sig);
  }
  return x;
}

uint lapack_SVD(
  arr& U,
  arr& d,
  arr& Vt,
  const arr& A) {
  arr Atmp, work;
  Atmp=A;
  //transpose(Atmp, A);
  integer M=A.d0, N=A.d1, D=M<N?M:N;
  U.resize(M, D);
  d.resize(D);
  Vt.resize(D, N);
  work.resize(10*(M+N));
  integer info, wn=work.N;
  dgesvd_((char*)"S", (char*)"S", &N, &M, Atmp.p, &N, d.p, Vt.p, &N, U.p, &D, work.p, &wn, &info);
  CHECK(!info, "LAPACK SVD error info = " <<info);
  return D;
}

void lapack_LU(arr& LU, const arr& A) {
  LU = A;
  integer M=A.d0, N=A.d1, D=M<N?M:N, info;
  intA piv(D);
  dgetrf_(&N, &M, LU.p, &N, (integer*)piv.p, &info);
  CHECK(!info, "LAPACK SVD error info = " <<info);
}

void lapack_RQ(arr& R, arr& Q, const arr& A) {
  transpose(Q, A);
  R.resizeAs(A); R.setZero();
  integer M=A.d0, N=A.d1, D=M<N?M:N, LWORK=M*N, info;
  arr tau(D), work(LWORK);
  dgerqf_(&N, &M, Q.p, &N, tau.p, work.p, &LWORK, &info);
  CHECK(!info, "LAPACK RQ error info = " <<info);
  for(int i=0; i<M; i++) for(int j=0; j<=i; j++) R(j, i) = Q(i, j); //copy upper triangle
  dorgrq_(&N, &M, &N, Q.p, &N, tau.p, work.p, &LWORK, &info);
  CHECK(!info, "LAPACK RQ error info = " <<info);
  Q=~Q;
  //cout <<"\nR=" <<R <<"\nQ=" <<Q <<"\nRQ=" <<R*Q <<"\nA=" <<A <<endl;
}

void lapack_EigenDecomp(const arr& symmA, arr& Evals, arr& Evecs) {
  CHECK(symmA.nd==2 && symmA.d0==symmA.d1, "not symmetric");
  arr work, symmAcopy = symmA;
  integer N=symmA.d0;
  Evals.resize(N);
  work.resize(10*(3*N));
  integer info, wn=work.N;
  if(!!Evecs) {
    dsyev_((char*)"V", (char*)"L", &N, symmAcopy.p, &N, Evals.p, work.p, &wn, &info);
    Evecs = symmAcopy;
  } else {
    dsyev_((char*)"N", (char*)"L", &N, symmAcopy.p, &N, Evals.p, work.p, &wn, &info);
  }
  CHECK(!info, "lapack_EigenDecomp error info = " <<info);
}

arr lapack_kSmallestEigenValues_sym(const arr& A, uint k) {
  if(k>A.d0) k=A.d0; //  CHECK_LE(k, A.d0,"");
  integer N=A.d0, KD=A.d1-1, LDAB=A.d1, INFO;
  rai::Array<integer> IWORK(5*N), IFAIL(N);
  arr WORK(10*(3*N)), Acopy=A;
  integer M, IL=1, IU=k, LDQ=0, LDZ=1, LWORK=WORK.N;
  double VL=0., VU=0., ABSTOL=1e-8;
  arr sig(N);
  if(!isRowShifted(A)) {
    dsyevx_((char*)"N", (char*)"I", (char*)"L", &N, Acopy.p, &LDAB, &VL, &VU, &IL, &IU, &ABSTOL, &M, sig.p, (double*)nullptr, &LDZ, WORK.p, &LWORK, IWORK.p, IFAIL.p, &INFO);
  } else {
    dsbevx_((char*)"N", (char*)"I", (char*)"L", &N, &KD, Acopy.p, &LDAB, (double*)nullptr, &LDQ, &VL, &VU, &IL, &IU, &ABSTOL, &M, sig.p, (double*)nullptr, &LDZ, WORK.p, IWORK.p, IFAIL.p, &INFO);
  }
  sig.resizeCopy(k);
  return sig;
}

bool lapack_isPositiveSemiDefinite(const arr& symmA) {
  // Check that all eigenvalues are nonnegative.
  arr d, V;
  lapack_EigenDecomp(symmA, d, V);
  // d is nondecreasing ??!??
  for(double x:d) if(x<0.) return false;
  return true;
}

/// A=C^T C (C is upper triangular!)
void lapack_cholesky(arr& C, const arr& A) {
  CHECK_EQ(A.d0, A.d1, "");
  integer n=A.d0;
  integer info;
  C=A;
  //compute cholesky
  dpotrf_((char*)"L", &n, C.p, &n, &info);
  CHECK(!info, "LAPACK Cholesky decomp error info = " <<info);
  //clear the lower triangle:
  uint i, j;
  for(i=0; i<C.d0; i++) for(j=0; j<i; j++) C(i, j)=0.;
}

const char* potrf_ERR="\n\
*  INFO    (output) INTEGER\n\
*          = 0:  successful exit\n\
*          < 0:  if INFO = -i, the i-th argument had an illegal value\n\
*          > 0:  if INFO = i, the leading minor of order i is not\n\
*                positive definite, and the factorization could not be\n\
*                completed.\n";

void lapack_mldivide(arr& X, const arr& A, const arr& B) {
  if(isSparseMatrix(A)) {
    X = eigen_Ainv_b(A, B);
    return;
  }

  CHECK_EQ(A.nd, 2, "A in Ax=b must be a NxN matrix.");
  CHECK_EQ(A.d0, A.d1, "A in Ax=b must be square matrix.");
  CHECK(B.nd==1 || B.nd==2, "b in Ax=b must be a vector or matrix.");
  CHECK_EQ(A.d0, B.d0, "b and A must have the same amount of rows in Ax=b.");

  X = ~B;
  arr LU = ~A;
  integer N = A.d0, NRHS = (B.nd==1?1:B.d1), LDA = A.d1, INFO;
  rai::Array<integer> IPIV(N);

  dgesv_(&N, &NRHS, LU.p, &LDA, IPIV.p, X.p, &LDA, &INFO);
  CHECK(!INFO, "LAPACK gaussian elemination error info = " <<INFO);

  if(B.nd==1) X.reshape(X.N);
  else X = ~X;
}

void lapack_choleskySymPosDef(arr& Achol, const arr& A) {
  if(isRowShifted(A)) {
    rai::RowShifted* Aaux = (rai::RowShifted*) A.special;
    if(!Aaux->symmetric) HALT("this is not a symmetric matrix");
    for(uint i=0; i<A.d0; i++) if(Aaux->rowShift(i)!=i) HALT("this is not shifted as an upper triangle");

    Achol=A;
    integer N=A.d0, KD=A.d1-1, LDAB=A.d1, INFO;

    dpbtrf_((char*)"L", &N, &KD, Achol.p, &LDAB, &INFO);
    CHECK(!INFO, "LAPACK Cholesky decomp error info = " <<INFO);

  } else {
    NIY;
  }

}

void lapack_inverseSymPosDef(arr& Ainv, const arr& A) {
  Ainv=A;
  integer N=A.d0, LDAB=A.d1, INFO;
  //compute cholesky
  dpotrf_((char*)"L", &N, Ainv.p, &LDAB, &INFO);
  CHECK(!INFO, "LAPACK Cholesky decomp error info = " <<INFO <<potrf_ERR);
  //invert
  dpotri_((char*)"L", &N, Ainv.p, &N, &INFO);
  CHECK(!INFO, "lapack_inverseSymPosDef error info = " <<INFO);
  //fill in the lower triangular elements
  for(uint i=0; i<(uint)N; i++) for(uint j=0; j<i; j++) Ainv.p[i*N+j]=Ainv.p[j*N+i]; //fill in the lower triangle
}

double lapack_determinantSymPosDef(const arr& A) {
  arr C;
  lapack_cholesky(C, A);
  double det=1.;
  for(uint i=0; i<C.d0; i++) det *= C(i, i)*C(i, i);
  return det;
}

void lapack_min_Ax_b(arr& x, const arr& A, const arr& b) {
  CHECK(A.d0>=A.d1 && A.d0==b.N && b.nd==1 && A.nd==2, "");
  arr At = ~A;
  x=b;
  integer M=A.d0, N=A.d1, NRHS=1, LWORK=2*M*N, info;
  arr work(LWORK);
  dgels_((char*)"N", &M, &N, &NRHS, At.p, &M, x.p, &M, work.p, &LWORK, &info);
  CHECK(!info, "dgels_ error info = " <<info);
  x.resizeCopy(A.d1);
}

arr lapack_Ainv_b_symPosDef_givenCholesky(const arr& U, const arr& b) {
  //in lapack (or better fortran) the rows and columns are switched!! (ARGH)
  integer N = U.d0, LDA = U.d1, INFO, LDB = b.d0, NRHS = 1;
  arr x;
  if(b.nd > 1) {
    NRHS = b.d1;
    x = ~b; //TODO is there a chance to remove this?
    dpotrs_((char*)"L", &N, &NRHS, U.p, &LDA, x.p, &LDB, &INFO);
    CHECK(!INFO, "lapack dpotrs error info = " << INFO);
    return ~x;
  } else {
    x = b;
    dpotrs_((char*)"L", &N, &NRHS, U.p, &LDA, x.p, &LDB, &INFO);
    CHECK(!INFO, "lapack dpotrs error info = " << INFO);
    return x;
  }
}

arr lapack_Ainv_b_triangular(const arr& L, const arr& b) {
  //DTRTRS
  integer N = L.d0, LDA = L.d0, INFO, LDB = b.d0, NRHS = 1;
  arr x = b;
  dtrtrs_((char*)"L", (char*)"N", (char*)"N", &N, &NRHS, L.p, &LDA, x.p, &LDB, &INFO);
  CHECK(!INFO, "lapack dtrtrs error info = " << INFO);
  return x;
}

/*
dpotri uses:
dtrtri = invert triangular

dlauum = multiply L'*L
*/

#else //if defined RAI_LAPACK
#if !defined RAI_MSVC && defined RAI_NOCHECK
#  warning "RAI_LAPACK undefined - using inefficient implementations"
#endif
void blas_MM(arr& X, const arr& A, const arr& B) { rai::useLapack=false; innerProduct(X, A, B); };
void blas_MsymMsym(arr& X, const arr& A, const arr& B) { rai::useLapack=false; innerProduct(X, A, B); };
void blas_Mv(arr& y, const arr& A, const arr& x) {       rai::useLapack=false; innerProduct(y, A, x); rai::useLapack=true; };
void blas_A_At(arr& X, const arr& A) { NICO }
void blas_At_A(arr& X, const arr& A) { NICO }
void lapack_cholesky(arr& C, const arr& A) { NICO }
void lapack_choleskySymPosDef(arr& Achol, const arr& A) { NICO }
uint lapack_SVD(arr& U, arr& d, arr& Vt, const arr& A) { NICO; }
void lapack_LU(arr& LU, const arr& A) { NICO; }
void lapack_RQ(arr& R, arr& Q, const arr& A) { NICO; }
void lapack_EigenDecomp(const arr& symmA, arr& Evals, arr& Evecs) { NICO; }
bool lapack_isPositiveSemiDefinite(const arr& symmA) { NICO; }
void lapack_inverseSymPosDef(arr& Ainv, const arr& A) { NICO; }
arr lapack_kSmallestEigenValues_sym(const arr& A, uint k) { NICO; }
arr lapack_Ainv_b_sym(const arr& A, const arr& b) {
  arr invA;
  inverse(invA, A);
  return invA*b;
};
double lapack_determinantSymPosDef(const arr& A) { NICO; }
void lapack_mldivide(arr& X, const arr& A, const arr& b) { NICO; }
arr lapack_Ainv_b_symPosDef_givenCholesky(const arr& U, const arr& b) { return inverse(U)*b; }
arr lapack_Ainv_b_triangular(const arr& L, const arr& b) { return inverse(L)*b; }
#endif

//===========================================================================
//
// Eigen
//

#ifdef RAI_EIGEN

Eigen::SparseMatrix<double> conv_sparseArr2sparseEigen(const rai::SparseMatrix& S) {
  arr& Z = S.Z;
  Eigen::SparseMatrix<double> E;
  E.resize(Z.d0, Z.d1);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(Z.N);
  for(uint k=0; k<Z.N; k++) triplets.push_back(Eigen::Triplet<double>(S.elems.p[2*k], S.elems.p[2*k+1], Z.p[k]));
  E.setFromTriplets(triplets.begin(), triplets.end());
  return E;
//  cout <<E <<endl;
}

arr conv_sparseEigen2sparseArr(Eigen::SparseMatrix<double>& E) {
  arr X;
  rai::SparseMatrix& Xs = X.sparse();
  Xs.resize(E.rows(), E.cols(), E.nonZeros());

  uint n=0;
  for(int k=0; k<E.outerSize(); ++k) {
    for(Eigen::SparseMatrix<double>::InnerIterator it(E, k); it; ++it) {
      Xs.entry(it.row(), it.col(), n) = it.value();
      n++;
    }
  }
  return X;
}

arr eigen_Ainv_b(const arr& A, const arr& b) {
  if(isSparseMatrix(A)) {
    rai::SparseMatrix& As = *dynamic_cast<rai::SparseMatrix*>(A.special);
    Eigen::SparseMatrix<double> Aeig = conv_sparseArr2sparseEigen(As);
    Eigen::MatrixXd beig = conv_arr2eigen(b);
    if(A.d0==A.d1) { //square matrix
      Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
      //    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
      solver.compute(Aeig);
      if(solver.info()!=Eigen::Success) {
        HALT("decomposition failed");
        return NoArr;
      }
      Eigen::MatrixXd x = solver.solve(beig);
      if(solver.info()!=Eigen::Success) {
        HALT("solving failed");
        return NoArr;
      }
      return conv_eigen2arr(x);
    } else { //non-square matrix
      Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
      solver.compute(Aeig);
      if(solver.info()!=Eigen::Success) {
        HALT("decomposition failed");
        return NoArr;
      }
      Eigen::MatrixXd x = solver.solve(beig);
      if(solver.info()!=Eigen::Success) {
        HALT("solving failed");
        return NoArr;
      }
      return conv_eigen2arr(x);
    }
  } else NIY;
  return NoArr;
}

#else //RAI_EIGEN

//Eigen::SparseMatrix<double> conv_sparseArr2sparseEigen(const rai::SparseMatrix& S){ NICO }
//arr conv_sparseEigen2sparseArr(Eigen::SparseMatrix<double>& E){ NICO }
arr eigen_Ainv_b(const arr& A, const arr& b) { NICO }

#endif //RAI_EIGEN

//===========================================================================
//
// RowShifted
//

rai::RowShifted::RowShifted(arr& X) : Z(X), rowSize(0), symmetric(false) {
  type = SpecialArray::RowShiftedST;
  Z.special = this;
}

rai::RowShifted::RowShifted(arr& X, rai::RowShifted& aux)
  : Z(X),
    rowSize(aux.rowSize),
    rowShift(aux.rowShift),
    rowLen(aux.rowLen),
    colPatches(aux.colPatches),
    symmetric(aux.symmetric) {
  type = SpecialArray::RowShiftedST;
  Z.special=this;
}

double rai::RowShifted::elem(uint i, uint j) const {
  CHECK(Z.nd==2 && i<Z.d0 && j<Z.d1,
        "2D range error (" <<Z.nd <<"=2, " <<i <<"<" <<Z.d0 <<", " <<j <<"<" <<Z.d1 <<")");
  uint rs=rowShift.p[i];;
  if(j<rs || j>=rs+rowSize) return 0.;
  return Z.p[i*rowSize+j-rs];
}

double& rai::RowShifted::elemNew(uint i, uint j){
  CHECK(i<Z.d0 && j<Z.d1,
        "2D range error (" <<Z.nd <<"=2, " <<i <<"<" <<Z.d0 <<", " <<j <<"<" <<Z.d1 <<")");
  uint rs=rowShift.p[i];;
  uint rl=rowLen.p[i];
  if(!rl){ //first element in this row!
    rowShift.p[i] = j;
    rowLen.p[i] = 1;
    return entry(i,0);
  }
  if(j<rs){ //need to shift row to the right!
    CHECK_LE(rl+rs-j, Z.d1, ""); //can't shift! (rs+rl<=Z.d1 always!)
    memmove(&entry(i,rs-j), &entry(i,0), rl*Z.sizeT);
    memset(&entry(i,0), 0, (rs-j)*Z.sizeT);
    rowLen.p[i] += rs-j;
    rowShift.p[i] = j;
    return entry(i,0);
  }
  if(j+1>rs+rl){ //need to extend rowLen
    rowLen.p[i] = j+1-rs;
    CHECK_LE(rowLen.p[i], rowSize, "rowShifted was created too small");
  }
  return entry(i,j-rs);
}

double& rai::RowShifted::entry(uint i, uint j) const {
//  CHECK(Z.nd==2 && i<Z.d0 && j<rowSize,
//        "2D range error (" <<Z.nd <<"=2, " <<i <<"<" <<Z.d0 <<", " <<j <<"<" <<rowSize <<")");
  return Z.p[i*rowSize+j];
}

void rai::RowShifted::resize(uint d0, uint d1, uint _rowSize){
  Z.nd=2; Z.d0=d0; Z.d1=d1;
  Z.resizeMEM(d0*_rowSize, false);
  Z.setZero();
  rowSize = _rowSize;
  rowShift.resize(d0).setZero();
  rowLen.resize(d0).setZero();
}

void rai::RowShifted::resizeCopy(uint d0, uint d1, uint n){ NIY; }

void rai::RowShifted::reshape(uint d0, uint d1){ NIY; }

void rai::RowShifted::reshift() {
  for(uint i=0; i<Z.d0; i++) {
    //find number of leading and trailing zeros
    double* Zp = Z.p + i*rowSize;
    double* Zbeg = Zp;
    double* Zend = Zp + rowSize-1;
    while(Zend>=Zbeg && *Zend==0.) Zend--;
    while(Zbeg<=Zend && *Zbeg==0.) Zbeg++;
    if(Zend<Zbeg) { //all zeros
      rowShift.p[i]=0;
      rowLen.p[i]=0;
    } else {
      uint rs = Zbeg-Zp;
      uint rl = 1+Zend-Zbeg;
      rowShift.p[i] += rs;
      rowLen.p[i] = rl;
      if(Zbeg!=Zp) {
        memmove(Zp, Zbeg, rl*Z.sizeT);
        memset(Zp+rl, 0, (rowSize-rl)*Z.sizeT);
      }
    }
  }
}

arr rai::RowShifted::unpack() const {
  arr X(Z.d0, Z.d1);
  CHECK(!symmetric || Z.d0==Z.d1, "cannot be symmetric!");
  X.setZero();
  for(uint i=0; i<Z.d0; i++) {
    uint rs=rowShift(i);
    uint rl=rowLen(i);
    for(uint j=0; j<rl && j+rs<X.d1; j++) {
      X(i, j+rs) = entry(i, j);
      if(symmetric) X(j+rs, i) = entry(i, j);
    }
  }
  return X;
}

void rai::RowShifted::checkConsistency() const {
  CHECK_EQ(rowShift.N, Z.d0, "");
  CHECK_EQ(rowLen.N, Z.d0, "");
  CHECK_EQ(rowSize * Z.d0, Z.N, "");
  for(uint i=0;i<Z.d0;i++){
    uint rs=rowShift(i);
    uint rl=rowLen(i);
    CHECK_LE(rl, rowSize, "");
    CHECK_LE(rs+rl, Z.d1, "");
    for(uint j=rs+rl;j<Z.d1;j++) CHECK_EQ(entry(i,j), 0., "");
  }
}

void rai::RowShifted::computeColPatches(bool assumeMonotonic) {
  colPatches.resize(Z.d1, 2);
  uint a=0, b=Z.d0;
  if(!assumeMonotonic) {
    for(uint j=0; j<Z.d1; j++) {
      a=0;
      while(a<Z.d0 && elem(a, j)==0) a++;
      b=Z.d0;
      while(b>a && elem(b-1, j)==0) b--;
      colPatches.p[2*j]=a;
      colPatches.p[2*j+1]=b;
    }
  } else {
    for(uint j=0; j<Z.d1; j++) {
      while(a<Z.d0 && j>=rowShift.p[a]+Z.d1) a++;
      colPatches.p[2*j]=a;
    }
    for(uint j=Z.d1; j--;) {
      while(b>0 && j<rowShift.p[b-1]) b--;
      colPatches.p[2*j+1]=b;
    }
  }
}

arr rai::RowShifted::At_A() {
  //TODO use blas DSYRK instead?
  CHECK_EQ(rowLen.N, rowShift.N, "");
  arr R;
  rai::RowShifted& R_ = R.rowShifted();
  R_.resize(Z.d1, Z.d1, rowSize);
  R.setZero();
  for(uint i=0; i<R.d0; i++) R_.rowShift(i) = i;
  for(uint i=0; i<R.d0; i++) R_.rowLen(i) = rowSize;
  R_.symmetric=true;
  if(!rowSize) return R; //Z is identically zero, all rows fully packed -> return zero R
  for(uint i=0; i<Z.d0; i++) {
    uint rs=rowShift.p[i];
    uint rl=rowLen.p[i];
    double* Zi = Z.p+i*rowSize;
    for(uint j=0; j<rl/*rowSize*/; j++) {
      uint real_j=j+rs;
      if(real_j>=Z.d1) break;
      double Zij=Zi[j];
      if(Zij!=0.) {
        double* Rp=R.p + real_j*R_.rowSize;
        double* Jp=Zi+j;
        double* Jpstop=Zi+rl; //rowSize;
        for(; Jp!=Jpstop; Rp++, Jp++) if(*Jp!=0.) *Rp += Zij * *Jp;
      }
    }
  }
  return R;
}

arr rai::RowShifted::A_At() {
  //-- determine pack_d1 for the resulting symmetric matrix
  uint pack_d1=1;
  for(uint i=0; i<Z.d0; i++) {
    uint rs_i=rowShift.p[i];
    for(uint j=Z.d0-1; j>=i+pack_d1; j--) {
      uint rs_j=rowShift.p[j];
      uint a, b;
      if(rs_i<rs_j) { a=rs_j; b=rs_i+rowSize; } else { a=rs_i; b=rs_j+rowSize; }
      if(b>Z.d1) b=Z.d1;
      if(a<b) if(pack_d1<j-i+1) pack_d1=j-i+1;
    }
  }

  arr R;
  rai::RowShifted& R_ = R.rowShifted();
  R_.resize(Z.d0, Z.d0, pack_d1);
  for(uint i=0; i<R.d0; i++) R_.rowShift(i) = i;
  for(uint i=0; i<R.d0; i++) R_.rowLen(i) = pack_d1;
  R_.symmetric=true;
  if(!rowSize) return R; //Z is identically zero, all rows fully packed -> return zero R
  for(uint i=0; i<Z.d0; i++) {
    uint rs_i=rowShift.p[i];
    double* Zi=&entry(i, 0);
    for(uint j=i; j<Z.d0 && j<i+pack_d1; j++) {
      uint rs_j=rowShift.p[j];
      double* Zj=&entry(j, 0);
      double* Rij=&R_.entry(i, j-i);

      uint a, b;
      if(rs_i<rs_j) { a=rs_j; b=rs_i+rowSize; } else { a=rs_i; b=rs_j+rowSize; }
      if(b>Z.d1) b=Z.d1;
      for(uint k=a; k<b; k++) *Rij += Zi[k-rs_i]*Zj[k-rs_j];
    }
  }
  return R;
}

arr rai::RowShifted::At_x(const arr& x) {
  CHECK_EQ(rowLen.N, rowShift.N, "");
  CHECK_EQ(x.N, Z.d0, "");
  arr y(Z.d1);
  y.setZero();
//  cout <<"SPARSITY = " <<Z.sparsity() <<endl;
  if(!Z.d1) return y; //Z is identically zero, all rows fully packed -> return zero y
  for(uint i=0; i<Z.d0; i++) {
    double xi = x.p[i];
    uint rs=rowShift.p[i];
#if 0
    for(uint j=0; j<Z.d1; j++) y.p[rs+j] += xi * entry(i,j); // sum += acc(i,j)*x(i);
#else //PROFILED
    double* Zp = Z.p + i*rowSize;
    double* yp = y.p + rs;
    double* ypstop = yp + rowLen.p[i]; //+ rowSize;
    for(; yp!=ypstop;) { *yp += xi * *Zp;  Zp++;  yp++; }
#endif
  }
  return y;
}

arr rai::RowShifted::A_x(const arr& x) {
  if(x.nd==2) {
    arr Y(x.d1, Z.d0);
    arr X = ~x;
    for(uint j=0; j<x.d1; j++) Y[j]() = A_x(X[j]);
    return ~Y;
  }
  CHECK_EQ(x.N, Z.d1, "");
  arr y = zeros(Z.d0);
  if(!Z.d1) return y; //Z is identically zero, all rows fully packed -> return zero y
  for(uint i=0; i<Z.d0; i++) {
    double sum=0.;
    uint rs=rowShift.p[i];
    for(uint j=0; j<rowSize && j+rs<x.N; j++) {
      sum += entry(i, j)*x(j+rs);
    }
    y(i) = sum;
  }
  return y;
}

arr rai::RowShifted::At() {
  uint width = 0;
  if(!colPatches.N) computeColPatches(false);
  for(uint i=0; i<colPatches.d0; i++) { uint a=colPatches(i, 1)-colPatches(i, 0); if(a>width) width=a; }

  arr At;
  rai::RowShifted& At_ = At.rowShifted();
  At_.resize(Z.d1, Z.d0, width);
  At.setZero();
  for(uint i=0; i<Z.d1; i++) {
    uint rs = colPatches(i, 0);
    uint rl = colPatches(i, 1)-rs;
    At_.rowLen(i) = rl;
    if(rl){
      At_.rowShift(i) = rs;
      for(uint j=0; j<rl; j++) At_.entry(i,j) = elem(rs+j, i);
    }
  }
  return At;
}

arr rai::RowShifted::A_B(const arr& B) const {
  CHECK(!isSpecial(B), "");
  arr X;
  RowShifted& Xr = X.rowShifted();
  Xr.resize(Z.d0, B.d1, B.d1);
  for(uint i=0; i<X.d0; i++) for(uint k=0;k<B.d1;k++) {
    uint rs = rowShift.p[i];
    uint rl = rowLen.p[i];
    for(uint j=0; j<rl; j++){
      Xr.elemNew(i,k) += entry(i,j) * B.p[(j+rs)*B.d1+k]; //B(j+rs,k);
    }
  }
  return X;
}

arr rai::RowShifted::B_A(const arr& B) const {
  CHECK(!isSpecial(B), "");
  arr X;
  RowShifted& Xr = X.rowShifted();
  Xr.resize(B.d0, Z.d1, rowSize);
  for(uint i=0; i<X.d0; i++) for(uint k=0;k<B.d1;k++){
    uint rs = rowShift.p[k];
    uint rl = rowLen.p[k];
    double Bik = B(i,k);
    for(uint j=0; j<rl; j++){
      Xr.elemNew(i,j+rs) += Bik*entry(k,j);
    }
  }
  return X;
}

void rai::RowShifted::rowWiseMult(const arr& a) {
  CHECK_EQ(a.N, Z.d0, "");
  for(uint i=0; i<Z.d0; i++){
    for(uint k=0;k<rowSize;k++) entry(i,k) *= a.p[i];
  }
}

void rai::RowShifted::add(const arr& B, uint lo0, uint lo1, double coeff){
  if(isRowShifted(B)){
    const RowShifted& Br = B.rowShifted();
    for(uint i=0;i<B.d0;i++){
      uint rs = Br.rowShift(i);
      uint rl = Br.rowLen(i);
      if(!lo0 && !lo1 && coeff==1.){
        for(uint j=0;j<rl;j++) elemNew(i, rs+j) += Br.entry(i, j);
      }else{
        for(uint j=0;j<rl;j++) elemNew(lo0 + i, lo1 + rs+j) += coeff*Br.entry(i, j);
      }
    }
  }else{
    for(uint i=0;i<B.d0;i++){
      if(!lo0 && !lo1 && coeff==1.){
        for(uint j=0;j<B.d1;j++) elemNew(i, j) += B(i, j);
      }else{
        for(uint j=0;j<B.d1;j++) elemNew(lo0 + i, lo1 + j) += coeff*B(i, j);
      }
    }
  }
}

void rai::RowShifted::write(ostream& os) const{
  os <<"RowShifted: real:" <<Z.d0 <<'x' <<Z.d1 <<"  packed:" <<Z.d0 <<'x' <<rowSize <<endl;
  os <<"packed numbers =\n" <<Z
      <<"\nrowShifts=" <<rowShift
     <<"\nrowLens=" <<rowLen;
  if(colPatches.N) os <<"\ncolPaches=\n" <<~colPatches;
  os  <<"\nunpacked =\n" <<unpack() <<endl;
}

//===========================================================================
//
// SparseMatrix
//

namespace rai {

SparseVector::SparseVector(arr& _Z) : Z(_Z) {
  CHECK(!isSpecial(_Z), "only once yet");
  type = sparseVectorST;
  Z.special = this;
}

SparseMatrix::SparseMatrix(arr& _Z) : Z(_Z) {
  CHECK(!isSpecial(_Z), "only once yet");
  type = sparseMatrixST;
  Z.special = this;
}

SparseVector::SparseVector(arr& _Z, const SparseVector& s) : SparseVector(_Z) {
  elems = s.elems;
}

SparseMatrix::SparseMatrix(arr& _Z, const SparseMatrix& s) : SparseMatrix(_Z) {
  elems = s.elems;
}

/// return fraction of non-zeros in the array
template<> double Array<double>::sparsity() {
  uint i, m=0;
  for(i=0; i<N; i++) if(elem(i)) m++;
  return ((double)m)/N;
}

void SparseVector::resize(uint d0, uint n) {
  Z.nd=1; Z.d0=d0;
  Z.resizeMEM(n, false);
  Z.setZero();
  elems.resize(n);
  for(int& e:elems) e=-1;
}

void SparseMatrix::resize(uint d0, uint d1, uint n) {
  Z.nd=2; Z.d0=d0; Z.d1=d1;
  Z.resizeMEM(n, false);
  Z.setZero();
  elems.resize(n, 2);
  for(int& e:elems) e=-1;
}

void SparseMatrix::resizeCopy(uint d0, uint d1, uint n) {
  Z.nd=2; Z.d0=d0; Z.d1=d1;
  uint Nold = Z.N;
  Z.resizeMEM(n, true);
  if(n>Nold) memset(Z.p+Nold, 0, Z.sizeT*(n-Nold));
  elems.resizeCopy(n, 2);
//  for(uint i=Nold; i<n; i++) elems(i, 0) = elems(i, 1) =-1;
  for(int *p=elems.p+2*Nold, *pstop=elems.p+2*n; p<pstop; p++) *p = -1;
}

void SparseMatrix::reshape(uint d0, uint d1) {
  Z.nd=2; Z.d0=d0; Z.d1=d1;
}

double& SparseVector::entry(uint i, uint k) {
  CHECK_LE(k, Z.N-1, "");
  if(elems.p[k]==-1) { //new element
    elems.p[k]=i;
  } else {
    CHECK_EQ(elems.p[k], (int)i, "");
  }
  return Z.p[k];
}

double& SparseMatrix::entry(uint i, uint j, uint k) {
  CHECK_LE(k, Z.N-1, "");
  int* elemsk = elems.p+2*k;
  if(*elemsk==-1) { //new element
    elemsk[0]=i;
    elemsk[1]=j;
    if(rows.nd){ rows.clear(); cols.clear(); }
  } else {
    CHECK_EQ(elemsk[0], (int)i, "");
    CHECK_EQ(elemsk[1], (int)j, "");
  }
  return Z.p[k];
}

double& SparseMatrix::elem(uint i, uint j) {
  if(rows.N) {
    uintA& r = rows(i);
    uintA& c = cols(j);
    if(r.N < c.N) {
      for(uint rj=0; rj<r.d0; rj++) if(r(rj, 0)==j) return Z.elem(r(rj, 1));
    } else {
      for(uint ci=0; ci<c.d0; ci++) if(c(ci, 0)==i) return Z.elem(c(ci, 1));
    }
  } else {
    for(uint k=0; k<elems.d0; k++)
      if(elems.p[2*k]==(int)i && elems.p[2*k+1]==(int)j) return Z.elem(k);
  }
  return addEntry(i, j);
}

double& SparseVector::addEntry(int i) {
  if(i<0) i += Z.d0;
  CHECK(Z.nd==1 && (uint)i<Z.d0,
        "1D range error (" <<Z.nd <<"=1, " <<i <<"<" <<Z.d0 <<")");
  uint k=Z.N;
  CHECK_EQ(elems.N, k, "");
  elems.resizeCopy(k+1);
  elems(k)=i;
  Z.resizeMEM(k+1, true);
  Z.last()=0.;
  return Z.last();
}

double& SparseMatrix::addEntry(int i, int j) {
  if(i<0) i += Z.d0;
  if(j<0) j += Z.d1;
  CHECK(Z.nd==2 && (uint)i<Z.d0 && (uint)j<Z.d1,
        "2D range error (" <<Z.nd <<"=2, " <<i <<"<" <<Z.d0 <<", " <<j <<"<" <<Z.d1 <<")");
  uint k=Z.N;
  CHECK_EQ(elems.d0, k, "");
  elems.resizeCopy(k+1, 2);
  elems(k, 0)=i;
  elems(k, 1)=j;
  if(rows.nd){ rows.clear(); cols.clear(); }
  Z.resizeMEM(k+1, true);
  Z.last()=0.;
  return Z.last();
}

arr SparseMatrix::getSparseRow(uint i) {
  arr v;
  SparseVector& vS = v.sparseVec();
  if(rows.N) {
    uintA& r = rows(i);
    uint n=r.d0;
    vS.resize(Z.d1, n);
    for(uint k=0; k<n; k++) {
      vS.entry(r(k, 0), k) = Z.elem(r(k, 1));
    }
  } else {
    NIY
  }
  return v;
}

void SparseVector::setFromDense(const arr& x) {
  CHECK_EQ(x.nd, 1, "");
  CHECK(&Z!=&x, "can't initialize from yourself");
  //count non-zeros
  uint n=0;
  for(const double& a:x) if(a) n++;
  //resize
  resize(x.d0, n);
  //set entries
  n=0;
  for(uint i=0; i<x.d0; i++) {
    double a = x.p[i];
    if(a) {
      entry(i, n) = a;
      n++;
    }
  }
}

void SparseMatrix::setFromDense(const arr& X) {
  CHECK_EQ(X.nd, 2, "");
  CHECK(&Z!=&X, "can't initialize from yourself");
  //count non-zeros
  uint n=0;
  for(const double& a:X) if(a) n++;
  //resize
  resize(X.d0, X.d1, n);
  //set entries
  n=0;
  for(uint i=0; i<X.d0; i++) for(uint j=0; j<X.d1; j++) {
      double a = X.p[i*X.d1+j];
      if(a) {
        entry(i, j, n) = a;
        n++;
      }
    }
}

void SparseMatrix::setupRowsCols() {
  rows.resize(Z.d0);
  cols.resize(Z.d1);
  for(uint k=0; k<elems.d0; k++) {
    uint i = elems(k, 0);
    uint j = elems(k, 1);
    rows(i).append(TUP(j, k));
    cols(j).append(TUP(i, k));
  }
  for(uint i=0; i<Z.d0; i++) rows(i).reshape(rows(i).N/2, 2);
  for(uint j=0; j<Z.d1; j++) cols(j).reshape(cols(j).N/2, 2);
}

void SparseMatrix::rowShift(int shift) {
  if(rows.nd){ rows.clear(); cols.clear(); }
  for(uint i=0; i<elems.d0; i++) {
    int& j = elems(i, 1);
    CHECK_GE(j+shift, 0, "");
    CHECK_LE(j+shift+1, (int)Z.d1, "");
    j += shift;
  }
}

void SparseMatrix::colShift(int shift) {
  if(rows.nd){ rows.clear(); cols.clear(); }
  for(uint i=0; i<elems.d0; i++) {
    int& j = elems.p[2*i]; //(i, 0);
    CHECK_GE(j+shift, 0, "");
    CHECK_LE(j+shift+1, (int)Z.d0, "");
    j += shift;
  }
}

#ifdef RAI_EIGEN

arr SparseMatrix::At_x(const arr& x) {
  Eigen::SparseMatrix<double> A_eig = conv_sparseArr2sparseEigen(*this);
  Eigen::MatrixXd x_eig = conv_arr2eigen(x);

  x_eig = A_eig.transpose() * x_eig;

  arr y(x_eig.rows());
  for(uint i = 0; i<y.d0; i++) y(i) = x_eig(i, 0);
  return y;
}

arr SparseMatrix::At_A() {
  Eigen::SparseMatrix<double> s = conv_sparseArr2sparseEigen(*this);

  Eigen::SparseMatrix<double> W(Z.d1, Z.d1);
  W = s.transpose() * s;

  return conv_sparseEigen2sparseArr(W);
}

arr SparseMatrix::A_B(const arr& B) const {
  if(!B.isSparse() && B.N<25){
    arr C;
    SparseMatrix &S = C.sparse();
    S.resize(B.d0, Z.d1, B.d1*Z.N); //resize to maximal possible
    uint l=0;
    for(uint k=0;k<Z.N;k++){
      uint a=elems(k,0);
      uint b=elems(k,1);
      double x = Z.elem(k);
      for(uint j=0;j<B.d1;j++) S.entry(a, j, l++) = B(b,j) * x;
    }
    CHECK_EQ(l, C.N, "");
    return C;
  }
  Eigen::SparseMatrix<double> A_eig = conv_sparseArr2sparseEigen(*this);
  Eigen::SparseMatrix<double> B_eig = conv_sparseArr2sparseEigen(B.copy().sparse());
//  Eigen::MatrixXd B_eig = conv_arr2eigen(B);

  Eigen::SparseMatrix<double> W = A_eig * B_eig;

  return conv_sparseEigen2sparseArr(W);
}

arr SparseMatrix::B_A(const arr& B) const {
  if(!B.isSparse() && B.N<25){
    arr C;
    SparseMatrix &S = C.sparse();
    S.resize(B.d0, Z.d1, B.d0*Z.N); //resize to maximal possible
    uint l=0;
    for(uint k=0;k<Z.N;k++){
      uint a=elems.p[2*k]; //(k,0);
      uint b=elems.p[2*k+1]; //(k,1);
      double x = Z.p[k]; //elem(k);
      for(uint i=0;i<B.d0;i++) S.entry(i, b, l++) = B.p[i*B.d1+a] * x; //B(i,a) * x;
    }
    CHECK_EQ(l, C.N, "");
//    S.resizeCopy(B.d0, Z.d1, l);
    return C;
  }
  Eigen::SparseMatrix<double> A_eig = conv_sparseArr2sparseEigen(*this);
  Eigen::SparseMatrix<double> B_eig = conv_sparseArr2sparseEigen(B.copy().sparse());

  Eigen::SparseMatrix<double> W = B_eig * A_eig;

  return conv_sparseEigen2sparseArr(W);
}

#else //RAI_EIGEN

arr SparseMatrix::At_x(const arr& x) { NICO }
arr SparseMatrix::At_A() { NICO }
arr SparseMatrix::A_B(const arr& B) const { NICO }
arr SparseMatrix::B_A(const arr& B) const { NICO }

#endif //RAI_EIGEN

void SparseMatrix::transpose() {
  uint d0 = Z.d0;
  Z.d0 = Z.d1;
  Z.d1 = d0;
  for(uint i=0; i<elems.d0; i++) {
    int k = elems(i, 0);
    elems(i, 0) = elems(i, 1);
    elems(i, 1) = k;
  }
  if(rows.nd){ cols.clear(); rows.clear(); }
}

void SparseMatrix::rowWiseMult(const arr& a) {
  CHECK_EQ(a.N, Z.d0, "");
  for(uint k=0; k<Z.N; k++) Z.elem(k) *= a.elem(elems.p[2*k]);
}

void SparseMatrix::rowWiseMult(const floatA& a) {
  CHECK_EQ(a.N, Z.d0, "");
  for(uint k=0; k<Z.N; k++) Z.elem(k) *= a.elem(elems.p[2*k]);
}

//void SparseMatrix::add(const SparseMatrix& a, double coeff) {
//  CHECK_EQ(a.Z.d0, Z.d0, "");
//  CHECK_EQ(a.Z.d1, Z.d1, "");
//  uint Nold=Z.N;
//  resizeCopy(Z.d0, Z.d1, Z.N + a.Z.N);
//  for(uint j=0; j<a.Z.N; j++) {
//    if(coeff==1.) {
//      entry(a.elems(j, 0), a.elems(j, 1), Nold+j) = a.Z.elem(j);
//    } else {
//      entry(a.elems(j, 0), a.elems(j, 1), Nold+j) = coeff * a.Z.elem(j);
//    }
//  }
//}

void SparseMatrix::add(const SparseMatrix& a, uint lo0, uint lo1, double coeff){
  CHECK_LE(lo0+a.Z.d0, Z.d0, "");
  CHECK_LE(lo1+a.Z.d1, Z.d1, "");
  if(!a.Z.N) return; //nothing to add
  uint Nold=Z.N;
#if 1
  Z.resizeMEM(Nold+a.Z.N, true);
  memmove(Z.p+Nold, a.Z.p, Z.sizeT*a.Z.N);
  elems.append(a.elems);
  if(coeff){
    for(double* x=&Z.elem(Nold); x!=Z.p+Z.N; x++) (*x) *= coeff;
  }
  if(lo0){
    for(int* i=&elems(Nold,0); i!=elems.p+elems.N; i+=2) (*i) += lo0;
  }
  if(lo1){
    for(int* i=&elems(Nold,1); i!=elems.p+elems.N+1; i+=2) (*i) += lo1;
  }
#else
  resizeCopy(Z.d0, Z.d1, Z.N + a.Z.N);
  for(uint j=0; j<a.Z.N; j++) {
    if(coeff==1.) {
      entry(lo0+a.elems(j, 0), lo1+a.elems(j, 1), Nold+j) = a.Z.elem(j);
    } else {
      entry(lo0+a.elems(j, 0), lo1+a.elems(j, 1), Nold+j) = coeff * a.Z.elem(j);
    }
  }
#endif
}

void SparseMatrix::add(const arr& B, uint lo0, uint lo1, double coeff){
  CHECK_LE(lo0+B.d0, Z.d0, "");
  CHECK_LE(lo1+B.d1, Z.d1, "");
  if(!B.N) return; //nothing to add
  uint Nold=Z.N;
  Z.resizeMEM(Nold+B.N, true);
  memmove(Z.p+Nold, B.p, Z.sizeT*B.N);
  if(isSparseMatrix(B)){
    elems.append(B.sparse().elems);
  }else{
    elems.resizeCopy(Nold+B.N, 2);
    int *e = &elems(Nold,0);
    for(uint i=0;i<B.d0;i++) for(uint j=0;j<B.d1;j++){
      *(e++) = i;
      *(e++) = j;
    }
  }
  if(coeff){
    for(double* x=&Z.elem(Nold); x!=Z.p+Z.N; x++) (*x) *= coeff;
  }
  if(lo0){
    for(int* i=&elems(Nold,0); i!=elems.p+elems.N; i+=2) (*i) += lo0;
  }
  if(lo1){
    for(int* i=&elems(Nold,1); i!=elems.p+elems.N+1; i+=2) (*i) += lo1;
  }
}

arr SparseVector::unsparse() {
  arr x;
  x.resize(Z.d0).setZero();
  for(uint k=0; k<Z.N; k++) x(elems(k)) += Z.elem(k);
  return x;
}

arr SparseMatrix::unsparse() {
  arr x;
  x.resize(Z.d0, Z.d1).setZero();
  for(uint k=0; k<Z.N; k++) x(elems(k, 0), elems(k, 1)) += Z.elem(k);
  return x;
}

} //namespace rai

void operator -= (rai::SparseMatrix& x, const rai::SparseMatrix& y) { x.add(y, 0, 0, -1.); }
void operator -= (rai::SparseMatrix& x, double y) { arr& X=x.Z; x.unsparse(); X -= y; }

void operator += (rai::SparseMatrix& x, const rai::SparseMatrix& y) { x.add(y); }
void operator += (rai::SparseMatrix& x, double y) { arr& X=x.Z; x.unsparse(); X += y; }

void operator *= (rai::SparseMatrix& x, const rai::SparseMatrix& y) { NIY; }
void operator *= (rai::SparseMatrix& x, double y) { x.memRef() *= y; }

void operator /= (rai::SparseMatrix& x, const rai::SparseMatrix& y) { NIY; }
void operator /= (rai::SparseMatrix& x, double y) { x.memRef() /= y; }

void operator -= (rai::RowShifted& x, const rai::RowShifted& y) { x.add(y.Z, 0, 0, -1.); }
void operator -= (rai::RowShifted& x, double y) { arr X=x.unpack(); X-=y; x.Z=X; }

void operator += (rai::RowShifted& x, const rai::RowShifted& y) { x.add(y.Z); }
void operator += (rai::RowShifted& x, double y) { arr X=x.unpack(); X+=y; x.Z=X; }

void operator *= (rai::RowShifted& x, const rai::RowShifted& y) { NIY; }
void operator *= (rai::RowShifted& x, double y) { x.memRef() *= y; }

void operator /= (rai::RowShifted& x, const rai::RowShifted& y) { NIY; }
void operator /= (rai::RowShifted& x, double y) { x.memRef() /= y; }
//void operator %= (rai::RowShifted& x, const rai::RowShifted& y){ NIY; }

//===========================================================================
//
// generic special
//

arr rai::unpack(const arr& X) {
  if(!isSpecial(X)) HALT("this is not special");
  if(isRowShifted(X)) return dynamic_cast<rai::RowShifted*>(X.special)->unpack();
  if(isSparseMatrix(X)) return dynamic_cast<rai::SparseMatrix*>(X.special)->unsparse();
  HALT("should not be here");
  return arr();
}

arr rai::comp_At_A(const arr& A) {
  if(!isSpecial(A)) { arr X; blas_At_A(X, A); return X; }
  if(isRowShifted(A)) return dynamic_cast<rai::RowShifted*>(A.special)->At_A();
  if(isSparseMatrix(A)) return dynamic_cast<rai::SparseMatrix*>(A.special)->At_A();
  return NoArr;
}

arr rai::comp_A_At(const arr& A) {
  if(!isSpecial(A)) { arr X; blas_A_At(X, A); return X; }
  if(isRowShifted(A)) return dynamic_cast<rai::RowShifted*>(A.special)->A_At();
  return NoArr;
}

//arr comp_A_H_At(arr& A, const arr& H){
//  if(!isSpecial(A)) { arr X; blas_A_At(X,A); return X; }
//  if(isRowShifted(A)) return ((rai::RowShifted*)A.aux)->A_H_At(H);
//  return NoArr;
//}

arr rai::comp_At_x(const arr& A, const arr& x) {
  if(!isSpecial(A)) { arr y; innerProduct(y, ~A, x); return y; }
  if(isRowShifted(A)) return ((rai::RowShifted*)A.special)->At_x(x);
  if(isSparseMatrix(A)) return ((rai::SparseMatrix*)A.special)->At_x(x);
  return NoArr;
}

arr rai::comp_At(const arr& A) {
  if(!isSpecial(A)) { return ~A; }
  if(isRowShifted(A)) return ((rai::RowShifted*)A.special)->At();
  return NoArr;
}

arr rai::comp_A_x(const arr& A, const arr& x) {
  if(!isSpecial(A)) { arr y; innerProduct(y, A, x); return y; }
  if(isRowShifted(A)) return ((rai::RowShifted*)A.special)->A_x(x);
  return NoArr;
}

//===========================================================================
//
// conv with Eigen
//

#ifdef RAI_EIGEN

arr conv_eigen2arr(const Eigen::MatrixXd& in) {
  if(in.cols()==1) {
    arr out(in.rows());
    for(uint i = 0; i<in.rows(); i++)
      out(i) = in(i, 0);
    return out;
  }
  arr out(in.rows(), in.cols());
  for(uint i = 0; i<in.rows(); i++)
    for(uint j = 0; j<in.cols(); j++)
      out(i, j) = in(i, j);
  return out;
}

Eigen::MatrixXd conv_arr2eigen(const arr& in) {
  if(in.nd == 1) {
    Eigen::MatrixXd out(in.d0, 1);
    for(uint i = 0; i<in.d0; i++)
      out(i, 0) = in(i);
    return out;
  } else if(in.nd == 2) {
    Eigen::MatrixXd out(in.d0, in.d1);
    for(uint i = 0; i<in.d0; i++)
      for(uint j = 0; j<in.d1; j++)
        out(i, j) = in(i, j);
    return out;
  }
  NIY;
  return Eigen::MatrixXd(1, 1);
}

#endif

//===========================================================================
//
// graphs
//

void graphRandomUndirected(uintA& E, uint n, double connectivity) {
  uint i, j;
  for(i=0; i<n; i++) for(j=i+1; j<n; j++) {
      if(rnd.uni()<connectivity) E.append(TUP(i, j));
    }
  E.reshape(E.N/2, 2);
}

void graphRandomTree(uintA& E, uint N, uint roots) {
  uint i;
  CHECK_GE(roots, 1, "");
  for(i=roots; i<N; i++) E.append(TUP(rnd(i), i));
  E.reshape(E.N/2, 2);
}

void graphRandomFixedDegree(uintA& E, uint N, uint d) {
  // --- from Joris' libDAI!!
  // Algorithm 1 in "Generating random regular graphs quickly"
  // by A. Steger and N.C. Wormald
  //
  // Draws a random graph with size N and uniform degree d
  // from an almost uniform probability distribution over these graphs
  // (which becomes uniform in the limit that d is small and N goes
  // to infinity).

  CHECK_EQ((N*d)%2, 0, "It's impossible to create a graph with " <<N<<" nodes and fixed degree " <<d);

  uint j;

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
    E.clear();
    bool finished = false;
    while(!finished) {
      U.permuteRandomly();
      uint i1, i2;
      bool suit_pair_found = false;
      for(i1=0; i1<U.N-1 && !suit_pair_found; i1++) {
        for(i2=i1+1; i2<U.N && !suit_pair_found; i2++) {
          if((U(i1)/d) != (U(i2)/d)) {  // they are suitable (refer to different nodes)
            suit_pair_found = true;
            E.append(TUP(U(i1)/d, U(i2)/d));
            U.remove(i2);  // first remove largest
            U.remove(i1);  // remove smallest
          }
          if(!suit_pair_found || !U.N)  finished = true;
        }
      }
    }
    E.reshape(E.N/2, 2);
    if(!U.N) {
      // G is a graph with edge from vertex r to vertex s if and only if
      // there is a pair containing points in the r'th and s'th groups.
      // If G is d-regular, output, otherwise return to Step 1.
      uintA degrees(N);
      degrees.setZero();
      for(j=0; j<E.d0; j++) {
        degrees(E(j, 0))++;
        degrees(E(j, 1))++;
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

  E.reshape(E.N/2, 2);
}

//===========================================================================
//
// explicit instantiations
// (in old versions, array.ipp was not included by array.h -- one could revive this)
//

//#include "array.ipp"
//#define T double
//#  include "array_instantiate.cxx"
//#undef T

//#define NOFLOAT
//#define T float
//#  include "array_instantiate.cxx"
//#undef T

//#define T uint
//#  include "array_instantiate.cxx"
//#undef T

//#define T uint16_t
//#  include "array_instantiate.cxx"
//#undef T

//#define T int
//#  include "array_instantiate.cxx"
//#undef T

//#define T long
//#  include "array_instantiate.cxx"
//#undef T
//#define T byte
//#  include "array_instantiate.cxx"
//#undef T
//#undef NOFLOAT

template rai::Array<rai::String>::Array();
template rai::Array<rai::String>::~Array();

template rai::Array<rai::String*>::Array();
template rai::Array<rai::String*>::~Array();

template arrL::Array();
template arrL::Array(uint);
template arrL::~Array();

template rai::Array<char const*>::Array();
template rai::Array<char const*>::Array(uint);
template rai::Array<char const*>::~Array();

template rai::Array<uintA>::Array();
template rai::Array<uintA>::Array(uint);
template rai::Array<uintA>::~Array();

template rai::Array<arr>::Array();
template rai::Array<arr>::Array(uint);
template rai::Array<arr>::~Array();

#include "util.ipp"

template rai::Array<double> rai::getParameter<arr>(char const*);
template rai::Array<double> rai::getParameter<arr>(char const*, const arr&);
template rai::Array<float> rai::getParameter<floatA>(char const*);
template rai::Array<uint> rai::getParameter<uintA>(char const*);
template bool rai::checkParameter<arr>(char const*);
template void rai::getParameter(uintA&, const char*, const uintA&);

void linkArray() { cout <<"*** libArray.so dynamically loaded ***" <<endl; }

//namespace rai{
//template<> template<> Array<rai::String>::Array(std::initializer_list<const char*> list) {
//  init();
//  for(const char* t : list) append(rai::String(t));
//}
//}

RUN_ON_INIT_BEGIN(array)
rai::Array<bool>::memMove=1;
rai::Array<char>::memMove=1;
rai::Array<unsigned char>::memMove=1;
rai::Array<int>::memMove=1;
rai::Array<unsigned int>::memMove=1;
rai::Array<short>::memMove=1;
rai::Array<unsigned short>::memMove=1;
rai::Array<long>::memMove=1;
rai::Array<unsigned long>::memMove=1;
rai::Array<float>::memMove=1;
rai::Array<double>::memMove=1;
RUN_ON_INIT_END(array)
