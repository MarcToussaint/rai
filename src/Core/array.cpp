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
    -----------------------------------------------------------------  */


#include "array.h"
#include "util.h"

static double MT_SIGN_SVD(double a, double b) { return b>0 ? ::fabs(a) : -::fabs(a); }
#define MT_max_SVD(a, b) ( (a)>(b) ? (a) : (b) )
#define MT_SVD_MINVALUE .0 //1e-10
#ifndef MT_NOCHECK
//#  define MT_CHECK_INVERSE 1e-5
//#  define MT_CHECK_SVD 1e-5
#endif


namespace MT {
//===========================================================================

bool useLapack=true;
#ifdef MT_LAPACK
const bool lapackSupported=true;
#else
const bool lapackSupported=false;
#endif
uint64_t globalMemoryTotal=0, globalMemoryBound=1ull<<30; //this is 1GB
bool globalMemoryStrict=false;
const char* arrayElemsep=" ";
const char* arrayLinesep="\n ";
const char* arrayBrackets="[]";

//===========================================================================
}

arr& NoArr = *((arr*)NULL);
uintA& NoUintA = *((uintA*)NULL);

//int ARRAYOLDREAD=0;

/* LAPACK notes
Use the documentation at
  http://www.netlib.org/lapack/double/
  http://www.netlib.org/lapack/individualroutines.html
to find the right function! Also use the man tools with Debian package lapack-doc installed.

I've put the clapack.h directly into the MT directory - one only has to link to the Fortran lib
*/


//===========================================================================
//
/// @name cblas and Lapack support
//

//===========================================================================
//
/// @name matrix operations
//


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

namespace MT {
/// use this to turn on Lapack routines [default true if MT_LAPACK is defined]
extern bool useLapack;
}



//===========================================================================
//
/// @name SVD etc
//

/// called from svd if MT_LAPACK is not defined
uint own_SVD(
  arr& U,
  arr& w,
  arr& V,
  const arr& A,
  bool sort);

/** @brief Singular Value Decomposition (from Numerical Recipes);
  computes \f$U, D, V\f$ with \f$A = U D V^T\f$ from \f$A\f$ such that
  \f$U\f$ and \f$V\f$ are orthogonal and \f$D\f$ diagonal (the
  returned array d is 1-dimensional) -- uses LAPACK if MT_LAPACK is
  defined */
uint svd(arr& U, arr& d, arr& V, const arr& A, bool sort) {
  uint r;
#ifdef MT_LAPACK
  if(MT::useLapack) {
    r=lapack_SVD(U, d, V, A);
    V=~V;
  } else {
    r=own_SVD(U, d, V, A, sort);
  }
#else
  r=own_SVD(U, d, V, A, sort);
#endif
  
#ifdef MT_CHECK_SVD
  bool uselapack=MT::useLapack;
  MT::useLapack=false;
  double err;
  arr dD, I;
  setDiagonal(dD, d);
  //cout <<U <<dD <<Vt;
  //Atmp = V * D * U;
  arr Atmp;
  Atmp = U * dD * ~V;
  //cout <<"\nA=" <<A <<"\nAtmp=" <<Atmp <<"U=" <<U <<"W=" <<dD <<"~V=" <<~V <<endl;
  std::cout <<"SVD is correct:  " <<(err=maxDiff(Atmp, A)) <<' ' <<endl;    CHECK(err<MT_CHECK_SVD, "");
  if(A.d0<=A.d1) {
    I.setId(U.d0);
    std::cout <<"U is orthogonal: " <<(err=maxDiff(U * ~U, I)) <<' ' <<endl;  CHECK(err<MT_CHECK_SVD, "");
    I.setId(V.d1);
    std::cout <<"V is orthogonal: " <<(err=maxDiff(~V * V, I)) <<endl;        CHECK(err<MT_CHECK_SVD, "");
  } else {
    I.setId(U.d1);
    std::cout <<"U is orthogonal: " <<(err=maxDiff(~U * U, I)) <<' ' <<endl;  CHECK(err<MT_CHECK_SVD, "");
    I.setId(V.d0);
    std::cout <<"V is orthogonal: " <<(err=sqrDistance(V * ~V, I)) <<endl;        CHECK(err<1e-5, "");
  }
  MT::useLapack=uselapack;
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

void pca(arr &Y, arr &v, arr &W, const arr &X, uint npc) {
  CHECK(X.nd == 2 && X.d0 > 0 && X.d1 > 0, "Invalid data matrix X.");
  CHECK(npc <= X.d1, "More principal components than data matrix X can offer.");

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
#ifdef MT_CHECK_INVERSE
  arr D, _D; D.setId(A.d0);
  uint me;
  _D=A*Ainv;
  double err=maxDiff(_D, D, &me);
  cout <<"inverse is correct: " <<err <<endl;
  if(A.d0<10) {
    CHECK(err<MT_CHECK_INVERSE , "inverting failed, error=" <<err <<" " <<_D.elem(me) <<"!=" <<D.elem(me) <<"\nA=" <<A <<"\nAinv=" <<Ainv <<"\nA*Ainv=" <<_D);
  } else {
    CHECK(err<MT_CHECK_INVERSE , "inverting failed, error=" <<err <<" " <<_D.elem(me) <<"!=" <<D.elem(me));
  }
#endif
}

uint inverse(arr& Ainv, const arr& A) {
  uint r=inverse_SVD(Ainv, A);
  //MT::inverse_LU(inverse, A); return A.d0;
  return r;
}

/// calls inverse(B, A) and returns B
arr inverse(const arr& A) { arr B; inverse(B, A); return B; }

/// Pseudo Inverse based on SVD; computes \f$B\f$ such that \f$ABA = A\f$
uint inverse_SVD(arr& Ainv, const arr& A) {
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
  double *Ainvij=&Ainv(0, 0);
  for(i=0; i<n; i++) for(j=0; j<m; j++) {
      double* vi = &V(i, 0);
      double* uj = &U(j, 0);
      double  t  = 0.;
      for(k=0; k<w.N; k++) t += vi[k] * winv.p[k] * uj[k];
      *Ainvij = t;
      Ainvij++;
    }
#endif
  
#ifdef MT_CHECK_INVERSE
  check_inverse(Ainv, A);
#endif
  return r;
}

void mldivide(arr& X, const arr& A, const arr& b) {
#ifdef MT_LAPACK
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
  
#ifdef MT_CHECK_INVERSE
  check_inverse(Xinv, X);
#endif
#endif
}

void inverse_SymPosDef(arr& Ainv, const arr& A) {
  CHECK(A.d0==A.d1, "");
#ifdef MT_LAPACK
  lapack_inverseSymPosDef(Ainv, A);
#else
  inverse_SVD(Ainv, A);
#endif
#ifdef MT_CHECK_INVERSE
  check_inverse(Ainv, A);
#endif
}

arr pseudoInverse(const arr& A, const arr& Winv, double eps) {
  arr At, E, AAt, AAt_inv, Ainv;
  transpose(At, A);
  if(&Winv) AAt = A*Winv*At; else AAt = A*At;
  if(eps) for(uint i=0;i<AAt.d0;i++) AAt(i,i) += eps;
  inverse_SymPosDef(AAt_inv, AAt);
  Ainv = At * AAt_inv;
  if(&Winv) Ainv = Winv * Ainv;
  return Ainv;
}

/// the determinant of a 2D squared matrix
double determinant(const arr& A);

/** @brief the cofactor is the determinant of a 2D squared matrix after removing
  the ith row and the jth column */
double cofactor(const arr& A, uint i, uint j);

void gaussFromData(arr& a, arr& A, const arr& X) {
  CHECK(X.nd==2, "");
  uint N=X.d0, n=X.d1;
  arr ones(N); ones=1.;
  a = ones*X/(double)N; a.reshape(n);
  A = (~X*X)/(double)N - (a^a);
}

/* compute a rotation matrix that rotates a onto v in arbitrary dimensions */
void rotationFromAtoB(arr& R, const arr& a, const arr& v) {
  CHECK(a.N==v.N, "");
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

//===========================================================================
//
/// @name gnuplot fun
//

/// calls gnuplot to display the (n, 2) or (n, 3) array (n=number of points of line or surface)
void gnuplot(const arr& X);

/// write 2 arrays in parallel columns in a file
void write(const arr& X, const arr& Y, const char* name);

/// write 3 arrays in parallel columns in a file
void write(const arr& X, const arr& Y, const arr& Z, const char* name);


//===========================================================================
//
/// @name simple image formats
//

/** save data as ppm or pgm. Images are (height, width, [0, 2, 3, 4])-dim
  byte arrays, where the 3rd dimension determines whether it's a grey
  (0), grey-alpha (2), RGB (3), or RGBA (4) image */
void write_ppm(const byteA &img, const char *file_name, bool swap_rows);

/** read data from an ppm or pgm file */
void read_ppm(byteA &img, const char *file_name, bool swap_rows);

/// add an alpha channel to an image array
void add_alpha_channel(byteA &img, byte alpha);

/// make grey scale image
void make_grey(byteA &img);

/// make a grey image and RGA image
void make_RGB(byteA &img);

/// make a grey image and RGA image
void make_RGB2BGRA(byteA &img);


//===========================================================================
//
// Array class
//



uint own_SVD(
  arr& U,
  arr& w,
  arr& V,
  const arr& A,
  bool sort) {
  //MT::Array<double*> Apointers, Upointers, Vpointers;
  unsigned m = A.d0; /* rows */
  unsigned n = A.d1; /* cols */
  U.resize(m, n);
  V.resize(n, n);
  w.resize(n);
  MT::Array<double*> Ap, Up, Vp;
  double **a = A.getCarray(Ap); //Pointers(Apointers); /* input matrix */
  double **u = U.getCarray(Up); //Pointers(Upointers); /* left vectors */
  double **v = V.getCarray(Vp); //Pointers(Vpointers); /* right vectors */
  
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
        g = -MT_SIGN_SVD(sqrt(s), f);
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
        g = -MT_SIGN_SVD(sqrt(s), f);
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
    
    anorm = MT_max_SVD(anorm, fabs(w(i)) + fabs(rv1(i)));
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
      f = ((x - z) * (x + z) + h * ((y / (f + MT_SIGN_SVD(g, f))) - h)) / x;
      
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
  
  for(r=0; r<n && w(r)>MT_SVD_MINVALUE; r++) {};
  
  t = r < n ? fabs(w(n-1)) : 0.0;
  r = 0;
  s = 0.0;
  while(r<n && w(r)>t && w(r)+s>s) s += w(r++);
  
  return r;
}

double determinantSubroutine(double **A, uint n) {
  if(n==1) return A[0][0];
  if(n==2) return A[0][0]*A[1][1]-A[0][1]*A[1][0];
  uint i, j;
  double d=0;
  double **B=new double*[n-1];
  for(i=0; i<n; i++) {
    for(j=0; j<n; j++) {
      if(j<i) B[j]=&A[j][1];
      if(j>i) B[j-1]=&A[j][1];
    }
    d+=((i&1)?-1.:1.) * A[i][0] * determinantSubroutine(B, n-1);
  }
  delete[] B;
  return d;
}

double determinant(const arr& A) {
  CHECK(A.nd==2 && A.d0==A.d1, "determinants require a squared 2D matrix");
  MT::Array<double*> tmp;
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
void SUS(const arr& p, uint n, uintA& s) {
  //following T. Baeck "EA in Theo. and Prac." p120
  s.resize(n);
  double sum=0, ptr=MT::rnd.uni();
  uint i, j=0;
  for(i=0; i<p.N; i++) {
    sum+=p(i)*n;
    while(sum>ptr) { s(j)=i; j++; ptr+=1.; }
  }
  //now, 'sum' should = 'n' and 'ptr' has been 'n'-times increased -> 'j=n'
  CHECK(j==n, "error in rnd::SUS(p, n, s) -> p not normalized?");
}

uint SUS(const arr& p) {
  double sum=0, ptr=MT::rnd.uni();
  uint i;
  for(i=0; i<p.N; i++) {
    sum+=p(i);
    if(sum>ptr) return i;
  }
  HALT("error in rnd::SUS(p) -> p not normalized? " <<p);
  return 0;
}

void gnuplot(const arr& X) {
  MT::arrayBrackets="  ";
  if(X.nd==2 && X.d1!=2) {  //assume array -> splot
    FILE("z.pltX") <<X;
    gnuplot("splot 'z.pltX' matrix with pm3d, 'z.pltX' matrix with lines");
    return;
  }
  if(X.nd==2 && X.d1==2) {  //assume curve -> plot
    FILE("z.pltX") <<X;
    gnuplot("plot 'z.pltX' us 1:2");
    return;
  }
  if(X.nd==1) {  //assume curve -> plot
//    arr Y;
//    Y.referTo(X);
//    Y.resize(Y.N, 1);
    FILE("z.pltX") <<X;
    gnuplot("plot 'z.pltX' us 1");
    return;
  }
}

//void write(const arr& X, const char *filename, const char *ELEMSEP, const char *LINESEP, const char *BRACKETS, bool dimTag, bool binary) {
//  std::ofstream fil;
//  MT::open(fil, filename);
//  X.write(fil, ELEMSEP, LINESEP, BRACKETS, dimTag, binary);
//  fil.close();
//}

//void write(std::ostream& os, const arrL& X, const char *ELEMSEP, const char *LINESEP, const char *BRACKETS, bool dimTag, bool binary) {
//  catCol(X).write(os, ELEMSEP, LINESEP, BRACKETS, dimTag, binary);
//}

void write(const arrL& X, const char *filename, const char *ELEMSEP, const char *LINESEP, const char *BRACKETS, bool dimTag, bool binary) {
  std::ofstream fil;
  MT::open(fil, filename);
  catCol(X).write(fil, ELEMSEP, LINESEP, BRACKETS, dimTag, binary);
  fil.close();
}

void write_ppm(const byteA &img, const char *file_name, bool swap_rows) {
  if(!img.N) MT_MSG("empty image");
  CHECK(img.nd==2 || (img.nd==3 && img.d2==3), "only rgb or gray images to ppm");
  ofstream os;
  os.open(file_name, std::ios::out | std::ios::binary);
  if(!os.good()) HALT("could not open file `" <<file_name <<"' for output");
  switch(img.d2) {
    case 0:  os <<"P5 " <<img.d1 <<' ' <<img.d0 <<" 255\n";  break; //PGM
    case 3:  os <<"P6 " <<img.d1 <<' ' <<img.d0 <<" 255\n";  break; //PPM
  }
  if(!swap_rows) {
    os.write((char*)img.p, img.N);
  } else {
    for(uint i=img.d0; i--;) os.write((char*)&img(i, 0, 0), img.d1*img.d2);
  }
}

void read_ppm(byteA &img, const char *file_name, bool swap_rows) {
  uint mode, width, height, max;
  ifstream is;
  is.open(file_name, std::ios::in | std::ios::binary);
  if(!is.good()) HALT("could not open file `" <<file_name <<"' for input");
  if(is.get()!='P') HALT("NO PPM FILE:" <<file_name);
  is >>mode;
  if(MT::peerNextChar(is)=='#') MT::skipRestOfLine(is);
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

void add_alpha_channel(byteA &img, byte alpha) {
  uint w=img.d1, h=img.d0;
  img.reshape(h*w, 3);
  img.insColumns(3, 1);
  for(uint i=0; i<img.d0; i++) img(i, 3)=alpha;
  img.reshape(h, w, 4);
}

void flip_image(byteA &img) {
  if(!img.N) return;
  uint h=img.d0, n=img.N/img.d0;
  byteA line(n);
  byte *a, *b, *c;
  for(uint i=0; i<h/2; i++) {
    a=img.p+i*n;
    b=img.p+(h-1-i)*n;
    c=line.p;
    memmove(c, a, n);
    memmove(a, b, n);
    memmove(b, c, n);
  }
}

void make_grey(byteA &img) {
  CHECK(img.nd==3 && (img.d2==3 || img.d1==4), "makeGray requires color image as input");
  byteA tmp;
  tmp.resize(img.d0, img.d1);
  for(uint i=0; i<img.d0; i++) for(uint j=0; j<img.d1; j++) {
      tmp(i, j) = ((uint)img(i, j, 0) + img(i, j, 1) + img(i, j, 2))/3;
    }
  img=tmp;
}

void make_RGB(byteA &img) {
  CHECK(img.nd==2, "make_RGB requires grey image as input");
  byteA tmp;
  tmp.resize(img.d0, img.d1, 3);
  for(uint i=0; i<img.d0; i++) for(uint j=0; j<img.d1; j++) {
      tmp(i, j, 0) = img(i, j);
      tmp(i, j, 1) = img(i, j);
      tmp(i, j, 2) = img(i, j);
    }
  img=tmp;
}

void make_RGB2BGRA(byteA &img) {
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

#ifdef MT_EXPRESSIONS
void assign(arr& x) {
  CHECK(x.ex, "self-assignment only if it is an expression");
  MT::Ex *e=x.ex;
  x.init();
  x.ex=e;
  assign(x, x);
  delete x.ex;
  x.ex=0;
}

void assign(arr& x, const arr& a) {
  if(!a.ex) { x=a; return; }
  MT::Ex &e=*a.ex;
  if(e.op==MT::UNI) {
    arr *A=(arr*)e.A;
    if(A->ex) assign(*A);
    if(!e.trans && e.mul==1 && e.add==0) { x=*A; return; }
    if(!e.trans && e.mul==1) { scalarPlus(x, *A, *((double*)&e.add)); return; }
    if(!e.trans && e.add==0) { scalarMultiplication(x, *A, *((double*)&e.mul)); return; }
    if(e.mul==1 && e.add==0) { transpose(x, *A); return; }
    HALT("");
  } else {
    arr *A=(arr*)e.A, *B=(arr*)e.B;
    if(A->ex) assign(*A);
    if(B->ex) assign(*B);
    //bool at, bt;
    //double ac, bc, ap, bp;
    switch(e.op) {
      case MT::PROD:
        if(!A->ex && !B->ex) { innerProduct(x, *A, *B); return; }
        HALT("prod");
        break;
      case MT::MUL:
        if(!A->ex && !B->ex) { mult(x, *A, *B); return; }
        HALT("mult");
        break;
      case MT::Div:
        if(!A->ex && !B->ex) { div(x, *A, *B); return; }
        HALT("mult");
        break;
      case MT::OUT:
        if(!A->ex && !B->ex) { outerProduct(x, *A, *B); return; }
        HALT("out");
        break;
      case MT::PLUS:
        if(!A->ex && !B->ex) { plus(x, *A, *B); return; }
        //if(A->ex){ ap=A->ex->add; ac=A->ex->mul; at=A->ex->trans; A=(arr*)A->ex->A; }else{ ap=0; ac=1; at=false; }
        //if(B->ex){ bp=B->ex->add; bc=B->ex->mul; bt=B->ex->trans; B=(arr*)B->ex->A; }else{ bp=0; bc=1; bt=false; }
        //if(!at && !bt && !ap && !bp){ plus(x, ac, *A, bc, *B); return; }
        //if(!at && !bt && !B){ scalarPlus(x, *A, bc); return; }
        HALT("plus");
        break;
      case MT::MINUS:
        if(!A->ex && !B->ex) { minus(x, *A, *B); return; }
        //if(A->ex){ ap=A->ex->add; ac=A->ex->mul; at=A->ex->trans; A=(arr*)A->ex->A; }else{ ap=0; ac=1; at=false; }
        //if(B->ex){ bp=B->ex->add; bc=B->ex->mul; bt=B->ex->trans; B=(arr*)B->ex->A; }else{ bp=0; bc=1; bt=false; }
        //if(!at && !bt && !ap && !bp){ plus(x, ac, *A, -bc, *B); return; }
        //if(!at && !bt && !B){ scalarPlus(x, *A, bc); return; }
        HALT("minus");
        break;
      case MT::UNI:
        HALT("shouldn't be here!");
        break;
    }
    HALT("yet undefined expression");
  }
}
#endif



void getIndexTuple(uintA &I, uint i, const uintA &d) {
  uint j;
  CHECK(i<product(d), "out of range");
  I.resize(d.N);
  I.setZero();
  for(j=d.N; j--;) {
    I.p[j] = i%d.p[j];
    i -= I.p[j];
    i /= d.p[j];
  }
}

void lognormScale(arr& P, double& logP, bool force) {
#ifdef MT_NoLognormScale
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
    MT_MSG("ill-conditioned table factor for norm scaling");
  }
}

void sparseProduct(arr& y, arr& A, const arr& x) {
  CHECK(x.nd==1 && A.nd==2 && x.d0==A.d1, "not a proper matrix multiplication");
  if(A.special!=arr::sparseST && x.special!=arr::sparseST) {
    innerProduct(y, A, x);
    return;
  }
  if(A.special==arr::sparseST && x.special!=arr::sparseST) {
    uint i, j, *k, *kstop;
    y.resize(A.d0); y.setZero();
    double *Ap=A.p;
    uintA* elems = (uintA*)A.aux;
    for(k=elems->p, kstop=elems->p+elems->N; k!=kstop; Ap++) {
      i=*k; k++;
      j=*k; k++;
      y.p[i] += (*Ap) * x.p[j];
    }
    return;
  }
  if(A.special==arr::sparseST && x.special==arr::sparseST) {
    uint i, j, n, *k, *kstop, *l, *lstop;
    y.clear(); y.nd=1; y.d0=A.d0;
    uintA *y_sparse;
    y.aux=y_sparse=new uintA [2];
    y_sparse[1].resize(y.d0); y_sparse[1]=(uint)-1;
    double *xp=x.p;
    uintA *elems, *col;
    elems = (uintA*)x.aux;
    uint *slot;
    for(k=elems->p, kstop=elems->p+elems->N; k!=kstop; xp++) {
      j=*k; k++;
      col = (uintA*)A.aux+(1+j);
      for(l=col->p, lstop=col->p+col->N; l!=lstop;) {
        i =*l; l++;
        n =*l; l++;
        slot=&y_sparse[1](i);
        if(*slot==(uint)-1) {
          *slot=y.N;
          y.resizeMEM(y.N+1, true); y(y.N-1)=0.;
          y_sparse[0].append(i);
          CHECK(y_sparse[0].N==y.N, "");
        }
        i=*slot;
        y(i) += A.elem(n) * (*xp);
      }
    }
    return;
  }
  if(A.special!=arr::sparseST && x.special==arr::sparseST) {
    uint i, j, *k, *kstop, d1=A.d1;
    y.resize(A.d0); y.setZero();
    double *xp=x.p;
    uintA *elems;
    elems = (uintA*)x.aux;
    for(k=elems->p, kstop=elems->p+elems->N; k!=kstop; xp++) {
      j=*k; k++;
      for(i=0; i<A.d0; i++) {
        y.p[i] += A.p[i*d1+j] * (*xp);
      }
    }
    return;
  }
}

void scanArrFile(const char* name) {
  ifstream is(name, std::ios::binary);
  CHECK(is.good(), "couldn't open file " <<name);
  arr x;
  String tag;
  for(;;) {
    tag.read(is, " \n\r\t", " \n\r\t");
    if(!is.good() || tag.N==0) return;
    x.readTagged(is, NULL);
    x.writeTagged(cout, tag);  cout <<endl;
    if(!is.good()) return;
  }
}

#ifndef CHECK_EPS
#  define CHECK_EPS 1e-8
#endif

/// numeric (finite difference) check of the gradient of f at x
bool checkGradient(ScalarFunction &f,
                   const arr& x, double tolerance) {
  arr J, dx, JJ;
  double y, dy;
  y=f.fs(J, NoArr, x);

  JJ.resize(x.N);
  double eps=CHECK_EPS;
  uint i;
  for(i=0; i<x.N; i++) {
    dx=x;
    dx.elem(i) += eps;
    dy = f.fs(NoArr, NoArr, dx);
    dy = (dy-y)/eps;
    JJ(i)=dy;
  }
  JJ.reshapeAs(J);
  double md=maxDiff(J, JJ, &i);
//   J >>FILE("z.J");
//   JJ >>FILE("z.JJ");
  if(md>tolerance) {
    MT_MSG("checkGradient -- FAILURE -- max diff=" <<md <<" |"<<J.elem(i)<<'-'<<JJ.elem(i)<<"| (stored in files z.J_*)");
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

bool checkHessian(ScalarFunction &f, const arr& x, double tolerance) {
  arr g, H, dx, dy, Jg;
  f.fs(g, H, x);
  if(H.special==arr::RowShiftedPackedMatrixST) H = unpack(H);

  Jg.resize(g.N, x.N);
  double eps=CHECK_EPS;
  uint i, k;
  for(i=0; i<x.N; i++) {
    dx=x;
    dx.elem(i) += eps;
    f.fs(dy, NoArr, dx);
    dy = (dy-g)/eps;
    for(k=0; k<g.N; k++) Jg(k, i)=dy.elem(k);
  }
  Jg.reshapeAs(H);
  double md=maxDiff(H, Jg, &i);
  //   J >>FILE("z.J");
  //   JJ >>FILE("z.JJ");
  if(md>tolerance) {
    MT_MSG("checkHessian -- FAILURE -- max diff=" <<md <<" |"<<H.elem(i)<<'-'<<Jg.elem(i)<<"| (stored in files z.J_*)");
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

bool checkJacobian(VectorFunction &f,
                   const arr& x, double tolerance) {
  arr y, J, dx, dy, JJ;
  f.fv(y, J, x);
  if(J.special==arr::RowShiftedPackedMatrixST) J = unpack(J);

  JJ.resize(y.N, x.N);
  double eps=CHECK_EPS;
  uint i, k;
  for(i=0; i<x.N; i++) {
    dx=x;
    dx.elem(i) += eps;
    f.fv(dy, NoArr, dx);
    dy = (dy-y)/eps;
    for(k=0; k<y.N; k++) JJ(k, i)=dy.elem(k);
  }
  JJ.reshapeAs(J);
  double md=maxDiff(J, JJ, &i);
//   J >>FILE("z.J");
//   JJ >>FILE("z.JJ");
  if(md>tolerance) {
    MT_MSG("checkJacobian -- FAILURE -- max diff=" <<md <<" |"<<J.elem(i)<<'-'<<JJ.elem(i)<<"| (stored in files z.J_*)");
    J >>FILE("z.J_analytical");
    JJ >>FILE("z.J_empirical");
//    (J/JJ) >>FILE("z.J_ana_emp");
    return false;
  } else {
    cout <<"checkJacobian -- SUCCESS (max diff error=" <<md <<")" <<endl;
  }
  return true;
}

#define EXP ::exp //MT::approxExp

double NNinv(const arr& a, const arr& b, const arr& Cinv){
  double d=sqrDistance(Cinv, a, b);
  double norm = ::sqrt(lapack_determinantSymPosDef((1./MT_2PI)*Cinv));
  return norm*EXP(-.5*d);
}
double logNNinv(const arr& a, const arr& b, const arr& Cinv){
  NIY;
  return 1;
  /*
  arr d=a-b;
  double norm = ::sqrt(fabs(MT::determinant_LU((1./MT_2PI)*Cinv)));
  return ::log(norm) + (-.5*scalarProduct(Cinv, d, d));
  */
}
double logNNprec(const arr& a, const arr& b, double prec){
  uint n=a.N;
  arr d=a-b;
  double norm = pow(prec/MT_2PI, .5*n);
  return ::log(norm) + (-.5*prec*scalarProduct(d, d));
}
double logNN(const arr& a, const arr& b, const arr& C){
  arr Cinv;
  inverse_SymPosDef(Cinv, C);
  return logNNinv(a, b, Cinv);
}
double NN(const arr& a, const arr& b, const arr& C){
  arr Cinv;
  inverse_SymPosDef(Cinv, C);
  return NNinv(a, b, Cinv);
}
/// non-normalized!! Gaussian function (f(0)=1)
double NNNNinv(const arr& a, const arr& b, const arr& Cinv){
  double d=sqrDistance(Cinv, a, b);
  return EXP(-.5*d);
}
double NNNN(const arr& a, const arr& b, const arr& C){
  arr Cinv;
  inverse_SymPosDef(Cinv, C);
  return NNNNinv(a, b, Cinv);
}
double NNzeroinv(const arr& x, const arr& Cinv){
  double norm = ::sqrt(lapack_determinantSymPosDef((1./MT_2PI)*Cinv));
  return norm*EXP(-.5*scalarProduct(Cinv, x, x));
}
/// gradient of a Gaussian
double dNNinv(const arr& x, const arr& a, const arr& Ainv, arr& grad){
  double y=NNinv(x, a, Ainv);
  grad = y * Ainv * (a-x);
  return y;
}
/// gradient of a non-normalized Gaussian
double dNNNNinv(const arr& x, const arr& a, const arr& Ainv, arr& grad){
  double y=NNNNinv(x, a, Ainv);
  grad = y * Ainv * (a-x);
  return y;
}
double NNsdv(const arr& a, const arr& b, double sdv){
  double norm = 1./(::sqrt(MT_2PI)*sdv);
  return norm*EXP(-.5*sqrDistance(a, b)/(sdv*sdv));
}
double NNzerosdv(const arr& x, double sdv){
  double norm = 1./(::sqrt(MT_2PI)*sdv);
  return norm*EXP(-.5*sumOfSqr(x)/(sdv*sdv));
}


//===========================================================================
//
// RowShiftedPackedMatrix
//

RowShiftedPackedMatrix::RowShiftedPackedMatrix(arr& X):Z(X), real_d1(0), symmetric(false), nextInSum(NULL) {
  Z.special = arr::RowShiftedPackedMatrixST;
  Z.aux = this;
}

RowShiftedPackedMatrix::RowShiftedPackedMatrix(arr& X, RowShiftedPackedMatrix &aux):
  Z(X),
  real_d1(aux.real_d1),
  rowShift(aux.rowShift),
  colPatches(aux.colPatches),
  symmetric(aux.symmetric),
  nextInSum(NULL)
{
  Z.special = arr::RowShiftedPackedMatrixST;
  Z.aux=this;
}

RowShiftedPackedMatrix *auxRowShifted(arr& Z, uint d0, uint pack_d1, uint real_d1) {
  RowShiftedPackedMatrix *Zaux;
  if(Z.special==arr::noneST) {
    Zaux = new RowShiftedPackedMatrix(Z);
  } else {
    CHECK(Z.special==arr::RowShiftedPackedMatrixST,"");
    Zaux = (RowShiftedPackedMatrix*) Z.aux;
  }
  Z.resize(d0, pack_d1);
  Z.setZero();
  Zaux->real_d1=real_d1;
  Zaux->rowShift.resize(d0);
  Zaux->rowShift.setZero();
  Zaux->colPatches.resize(real_d1, 2);
  for(uint i=0; i<real_d1; i++) {
    Zaux->colPatches(i,0)=0;
    Zaux->colPatches(i,1)=d0;
  }
  return Zaux;
}

RowShiftedPackedMatrix::~RowShiftedPackedMatrix() {
  if(nextInSum) delete nextInSum;
  Z.special = arr::noneST;
  Z.aux = NULL;
}

double RowShiftedPackedMatrix::acc(uint i, uint j) {
  uint rs=rowShift(i);
  if(j<rs || j>=rs+Z.d1) return 0.;
  return Z(i, j-rs);
}

arr packRowShifted(const arr& X) {
  arr Z;
  RowShiftedPackedMatrix *Zaux = auxRowShifted(Z, X.d0, 0, X.d1);
  Z.setZero();
  //-- compute rowShifts and pack_d1:
  uint pack_d1=0;
  for(uint i=0; i<X.d0; i++) {
    uint j=0,rs;
    while(j<X.d1 && X(i,j)==0.) j++;
    Zaux->rowShift(i)=rs=j;
    j=X.d1;
    while(j>rs && X(i,j-1)==0.) j--;
    if(j-rs>pack_d1) pack_d1=j-rs;
  }
  
  Z.resize(X.d0,pack_d1);
  Z.setZero();
  for(uint i=0; i<Z.d0; i++) for(uint j=0; j<Z.d1 && Zaux->rowShift(i)+j<X.d1; j++)
      Z(i,j) = X(i,Zaux->rowShift(i)+j);
  Zaux->computeColPatches(false);
  return Z;
}

arr unpackRowShifted(const arr& Y) {
  CHECK(Y.special==arr::RowShiftedPackedMatrixST,"");
  RowShiftedPackedMatrix *Yaux = (RowShiftedPackedMatrix*)Y.aux;
  arr X(Y.d0, Yaux->real_d1);
  CHECK(!Yaux->symmetric || Y.d0==Yaux->real_d1,"cannot be symmetric!");
  X.setZero();
  for(uint i=0; i<Y.d0; i++) {
    uint rs=Yaux->rowShift(i);
    for(uint j=0; j<Y.d1 && rs+j<X.d1; j++) {
      X(i,j+rs) = Y(i,j);
      if(Yaux->symmetric) X(j+rs,i) = Y(i,j);
    }
  }
  if(Yaux->nextInSum){
    X += unpackRowShifted(*Yaux->nextInSum);
  }
  return X;
}

void RowShiftedPackedMatrix::computeColPatches(bool assumeMonotonic) {
  colPatches.resize(real_d1,2);
  uint a=0,b=Z.d0;
  if(!assumeMonotonic) {
    for(uint j=0; j<real_d1; j++) {
      a=0;
      while(a<Z.d0 && acc(a,j)==0) a++;
      b=Z.d0;
      while(b>0 && acc(b-1,j)==0) b--;
      colPatches(j,0)=a;
      colPatches(j,1)=b;
    }
  } else {
    for(uint j=0; j<real_d1; j++) {
      while(a<Z.d0 && j>=rowShift(a)+Z.d1) a++;
      colPatches(j,0)=a;
    }
    for(uint j=real_d1; j--;) {
      while(b>0 && j<rowShift(b-1)) b--;
      colPatches(j,1)=b;
    }
  }
}

arr RowShiftedPackedMatrix::At_A() {
  //TODO use blas DSYRK instead?
  arr R;
  RowShiftedPackedMatrix *Raux = auxRowShifted(R, real_d1, Z.d1, real_d1);
  R.setZero();
  for(uint i=0; i<R.d0; i++) Raux->rowShift(i) = i;
  Raux->symmetric=true;
  if(!Z.d1) return R; //Z is identically zero, all rows fully packed -> return zero R
  for(uint i=0; i<Z.d0; i++) {
    uint rs=rowShift(i);
    double* Zi=&Z(i,0);
    for(uint j=0; j<Z.d1; j++) {
      uint real_j=j+rs;
      if(real_j>=real_d1) break;
      double Zij=Zi[j];
      if(Zij!=0.){
        double* Rp=R.p + real_j*R.d1;
        double* Jp=Zi+j;
        double* Jpstop=Zi+Z.d1;
        for(; Jp!=Jpstop; Rp++,Jp++) if(*Jp!=0.) *Rp += Zij * *Jp;
      }
    }
  }
  if(nextInSum){
    arr R2 = comp_At_A(*nextInSum);
    CHECK(R2.special==arr::RowShiftedPackedMatrixST, "");
    CHECK(R2.d1<=R.d1,"NIY"); //swap...
    for(uint i=0;i<R2.d0;i++) for(uint j=0;j<R2.d1;j++){
      R(i,j) += R2(i,j);
    }
  }
  return R;
}

arr RowShiftedPackedMatrix::A_At() {
  //-- determine pack_d1 for the resulting symmetric matrix
  uint pack_d1=1;
  for(uint i=0; i<Z.d0; i++) {
    uint rs_i=rowShift(i);
    for(uint j=Z.d0-1; j>=i+pack_d1; j--) {
      uint rs_j=rowShift(j);
      uint a=MT::MAX(rs_i,rs_j);
      uint b=MT::MIN(rs_i+Z.d1,rs_j+Z.d1);
      b=MT::MIN(real_d1,b);
      if(a<b) if(pack_d1<j-i+1) pack_d1=j-i+1;
    }
  }

  arr R;
  RowShiftedPackedMatrix *Raux = auxRowShifted(R, Z.d0, pack_d1, Z.d0);
  R.setZero();
  for(uint i=0; i<R.d0; i++) Raux->rowShift(i) = i;
  Raux->symmetric=true;
  if(!Z.d1) return R; //Z is identically zero, all rows fully packed -> return zero R
  for(uint i=0; i<Z.d0; i++) {
    uint rs_i=rowShift(i);
    double* Zi=&Z(i,0);
    for(uint j=i; j<Z.d0 && j<i+pack_d1; j++) {
      uint rs_j=rowShift(j);
      double* Zj=&Z(j,0);
      double* Rij=&R(i,j-i);

      uint a=MT::MAX(rs_i,rs_j);
      uint b=MT::MIN(rs_i+Z.d1,rs_j+Z.d1);
      b=MT::MIN(real_d1,b);
      for(uint k=a;k<b;k++) *Rij += Zi[k-rs_i]*Zj[k-rs_j];
    }
  }
  if(nextInSum) NIY;
  return R;
}

arr RowShiftedPackedMatrix::At_x(const arr& x) {
  CHECK(x.N==Z.d0,"");
  arr y(real_d1);
  y.setZero();
  if(!Z.d1) return y; //Z is identically zero, all rows fully packed -> return zero y
  for(uint j=0; j<real_d1; j++) {
    double sum=0.;
    uint a=colPatches(j,0);
    uint b=colPatches(j,1);
    for(uint i=a; i<b; i++) {
      uint rs=rowShift.p[i];
      if(j<rs || j-rs>=Z.d1) continue;
      sum += Z.p[i*Z.d1+j-rs]*x.p[i]; // sum += acc(i,j)*x(i);
    }
    y(j) = sum;
  }
  if(nextInSum) y += comp_At_x(*nextInSum, x);
  return y;
}

arr RowShiftedPackedMatrix::A_x(const arr& x) {
  CHECK(x.N==real_d1,"");
  arr y(Z.d0);
  y.setZero();
  if(!Z.d1) return y; //Z is identically zero, all rows fully packed -> return zero y
  for(uint i=0; i<Z.d0; i++) {
    double sum=0.;
    uint rs=rowShift.p[i];
    for(uint j=0; j<Z.d1 && j+rs<x.N; j++) {
      sum += Z(i,j)*x(j+rs);
    }
    y(i) = sum;
  }
  if(nextInSum) NIY;
  return y;
}

arr unpack(const arr& X) {
  if(X.special==arr::noneST) HALT("this is not special");
  if(X.special==arr::RowShiftedPackedMatrixST) return unpackRowShifted(X);
  return NoArr;
}

arr comp_At_A(arr& A) {
  if(A.special==arr::noneST) { arr X; blas_At_A(X,A); return X; }
  if(A.special==arr::RowShiftedPackedMatrixST) return ((RowShiftedPackedMatrix*)A.aux)->At_A();
  return NoArr;
}

arr comp_A_At(arr& A) {
  if(A.special==arr::noneST) { arr X; blas_A_At(X,A); return X; }
  if(A.special==arr::RowShiftedPackedMatrixST) return ((RowShiftedPackedMatrix*)A.aux)->A_At();
  return NoArr;
}

arr comp_At_x(arr& A, const arr& x) {
  if(A.special==arr::noneST) { arr y; innerProduct(y, ~A, x); return y; }
  if(A.special==arr::RowShiftedPackedMatrixST) return ((RowShiftedPackedMatrix*)A.aux)->At_x(x);
  return NoArr;
}

arr comp_A_x(arr& A, const arr& x) {
  if(A.special==arr::noneST) { arr y; innerProduct(y, A, x); return y; }
  if(A.special==arr::RowShiftedPackedMatrixST) return ((RowShiftedPackedMatrix*)A.aux)->A_x(x);
  return NoArr;
}


//===========================================================================
//
// graphs
//

void graphRandomUndirected(uintA& E, uint n, double connectivity) {
  uint i, j;
  for(i=0; i<n; i++) for(j=i+1; j<n; j++) {
      if(rnd.uni()<connectivity) E.append(TUP(i,j));
    }
  E.reshape(E.N/2,2);
}

void graphRandomTree(uintA& E, uint N, uint roots) {
  uint i;
  CHECK(roots>=1, "");
  for(i=roots; i<N; i++) E.append(TUP(rnd(i), i));
  E.reshape(E.N/2,2);
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
  
  CHECK((N*d)%2==0, "It's impossible to create a graph with " <<N<<" nodes and fixed degree " <<d);
  
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
    E.reshape(E.N/2,2);
    if(!U.N) {
      // G is a graph with edge from vertex r to vertex s if and only if
      // there is a pair containing points in the r'th and s'th groups.
      // If G is d-regular, output, otherwise return to Step 1.
      uintA degrees(N);
      degrees.setZero();
      for(j=0; j<E.d0; j++) {
        degrees(E(j,0))++;
        degrees(E(j,1))++;
      }
      ready = true;
      for(uint n=0; n<N; n++) {
        CHECK(degrees(n)<=d, "");
        if(degrees(n)!=d) {
          ready = false;
          break;
        }
      }
    } else ready=false;
  }
  
  E.reshape(E.N/2,2);
}

//===========================================================================
//
// explicit instantiations
//

#include "array_t.h"
#define T double
#  include "array_instantiate.cxx"
#undef T

#define NOFLOAT
#define T float
#  include "array_instantiate.cxx"
#undef T

#define T uint
#  include "array_instantiate.cxx"
#undef T

#define T uint16
#  include "array_instantiate.cxx"
#undef T

#define T int
#  include "array_instantiate.cxx"
#undef T

#define T long
#  include "array_instantiate.cxx"
#undef T
#define T byte
#  include "array_instantiate.cxx"
#undef T
#undef NOFLOAT

template MT::Array<MT::String>::Array();
template MT::Array<MT::String>::~Array();

template MT::Array<MT::String*>::Array();
template MT::Array<MT::String*>::~Array();

template arrL::Array();
template arrL::Array(uint);
template arrL::~Array();

template MT::Array<char const*>::Array();
template MT::Array<char const*>::Array(uint);
template MT::Array<char const*>::~Array();

template MT::Array<uintA>::Array();
template MT::Array<uintA>::Array(uint);
template MT::Array<uintA>::~Array();

template MT::Array<arr>::Array();
template MT::Array<arr>::Array(uint);
template MT::Array<arr>::~Array();

#include "util_t.h"
template MT::Array<double> MT::getParameter<MT::Array<double> >(char const*);
template MT::Array<uint> MT::getParameter<MT::Array<uint> >(char const*);
template bool MT::checkParameter<MT::Array<double> >(char const*);
template void MT::getParameter(uintA&, const char*, const uintA&);

void linkArray() { cout <<"*** libArray.so dynamically loaded ***" <<endl; }

MT::Array<MT::String> STRINGS(){ return ARRAY<MT::String>(); }
MT::Array<MT::String> STRINGS(const char* s0){ return ARRAY<MT::String>(MT::String(s0)); }
MT::Array<MT::String> STRINGS(const char* s0, const char* s1){ return ARRAY<MT::String>(MT::String(s0), MT::String(s1)); }
MT::Array<MT::String> STRINGS(const char* s0, const char* s1, const char* s2){ return ARRAY<MT::String>(MT::String(s0), MT::String(s1), MT::String(s2)); }
