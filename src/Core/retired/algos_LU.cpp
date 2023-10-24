/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "array.h"
#include "algos.h"

#define TINY 1.0e-20

void ludcmp(double** a, int n, int* indx, double* d);
void lubksb(double** a, int n, int* indx, double b[]);
double* vector(uint i, uint j) { return new double[j]; }
void nrerror(const char* msg) { HALT(msg); }
void free_vector(double* p, uint i, uint j) { delete[] p; }

namespace rai {
double determinant_LU(const arr& X) {
  CHECK(X.nd==2 && X.d0==X.d1, "");
  uint n=X.d0, i;
  arr LU;
  LU=X;
  intA idx(n);
  doubleA d(n);
  rai::Array<double*> tmp;
  ludcmp(LU.getCarray(tmp), n, idx.p, d.p);
  double det=1.;
  for(i=0; i<n; i++) det *= LU(i, i);

  //double ddet=determinant(X); CHECK_EQ(det,ddet, "");
  return det;
}

void inverse_LU(arr& Xinv, const arr& X) {
  CHECK(X.nd==2 && X.d0==X.d1, "");
  uint n=X.d0, i, j;
  Xinv.resize(n, n);
  if(n==0) return;
  if(n==1) { Xinv(0, 0)=1./X(0, 0); return; }
  if(n==2) { inverse2d(Xinv, X); return; }
  arr LU;
  LU=X;
  intA idx(n);
  doubleA d(n);
  rai::Array<double*> tmp;
  ludcmp(LU.getCarray(tmp), n, idx.p, d.p);
  //--
  arr col(n);
  for(j=0; j<n; j++) {
    col.setZero();
    col(j)=1.0;
    lubksb(LU.getCarray(tmp), n, idx.p, col.p);
    for(i=0; i<n; i++) Xinv(i, j)=col(i);
  }

#ifdef RAI_CHECK_INVERSE
  arr D, _D; D.setId(n);
  uint me;
  _D=X*Xinv;
  double err=maxDiff(_D, D, &me);
  CHECK(err<RAI_CHECK_INVERSE, "inverting failed, error=" <<err <<" " <<_D.elem(me) <<"!=" <<D.elem(me));
#endif
}

void LU_decomposition(arr& L, arr& U, const arr& X) {
  CHECK(X.nd==2 && X.d0==X.d1, "");
  uint n=X.d0, i, j;
  arr LU;
  LU=X;
  intA idx(n);
  doubleA d(n);
  rai::Array<double*> tmp;

  ludcmp(LU.getCarray(tmp), n, idx.p, d.p);

  L.resizeAs(LU);  L.setZero();
  U.resizeAs(LU);  U.setZero();
  for(i=0; i<n; i++) {
    for(j=0; j<i; j++)  L(i, j) = LU(i, j);
    L(i, i)=1.;
    for(; j<n; j++)     U(i, j) = LU(i, j);
  }
  cout <<X <<endl <<L* U <<endl <<idx <<endl <<d <<endl;
}
}

void ludcmp(double** a, int n, int* indx, double* d) {
  int i, imax=0, j, k;
  double big, dum, sum, temp;
  double* vv;
  vv=vector(1, n);
  *d=1.0;
  for(i=0; i<n; i++) {
    big=     0.0;
    for(j=0; j<n; j++)
      if((temp=fabs(a[i][j])) > big) big=temp;
    if(big == 0.0) nrerror("Singular matrix in routine ludcmp");
    vv[i]=1.0/big;
  }
  for(j=0; j<n; j++) {
    for(i=0; i<j; i++) {
      sum=a[i][j];
      for(k=0; k<i; k++) sum -= a[i][k]*a[k][j];
      a[i][j]=sum;
    }
    big=0.0;
    for(i=j; i<n; i++) {
      sum=a[i][j];
      for(k=0; k<j; k++)
        sum -= a[i][k]*a[k][j];
      a[i][j]=sum;
      if((dum=vv[i]*fabs(sum)) >= big) {
        big=dum;
        imax=i;
      }
    }
    if(j != imax) {
      for(k=0; k<n; k++) {
        dum=a[imax][k];
        a[imax][k]=a[j][k];
        a[j][k]=dum;
      }
      *d = -(*d);
      vv[imax]=vv[j];
    }
    indx[j]=imax;
    if(a[j][j] == 0.0) a[j][j]=TINY;

    if(j != n) {
      dum=1.0/(a[j][j]);
      for(i=j+1; i<n; i++) a[i][j] *= dum;
    }
  }
  free_vector(vv, 1, n);
}

void lubksb(double** a, int n, int* indx, double b[]) {
  int i, ii=0, ip, j;
  double sum;
  for(i=0; i<n; i++) {
    ip=indx[i];
    sum=b[ip];
    b[ip]=b[i];
    if(ii)
      for(j=ii; j<=i-1; j++) sum -= a[i][j]*b[j];
    else if(sum) ii=i;
    b[i]=sum;
  }
  for(i=n; i--;) {
    sum=b[i];
    for(j=i+1; j<n; j++) sum -= a[i][j]*b[j];
    b[i]=sum/a[i][i];
  }
}
