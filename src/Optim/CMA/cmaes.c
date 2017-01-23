/* --------------------------------------------------------- */
/* --- File: cmaes.c  -------- Author: Nikolaus Hansen   --- */
/* --------------------------------------------------------- */
/*   
     CMA-ES for non-linear function minimization. 

     Copyright (C) 1996, 2003  Nikolaus Hansen. 
     e-mail: hansen@bionik.tu-berlin.de
     
     This library is free software; you can redistribute it and/or
     modify it under the terms of the GNU Lesser General Public
     License as published by the Free Software Foundation; either
     version 2.1 of the License, or (at your option) any later 
     version (see http://www.gnu.org/copyleft/lesser.html).
     
     This library is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
     Lesser General Public License for more details.
   
*/
/* --- Changes : --- 
  03/03/21: argument const double *rgFunVal of
    cmaes_ReestimateDistribution() was treated incorrectly.
  03/03/29: restart via cmaes_resume_distribution() implemented.  
  03/03/30: Always max std dev / largest axis is printed first. 
  03/08/30: Damping is adjusted for large mueff. 
  03/10/30: Damping is adjusted for large mueff always. 
  04/04/22: Cumulation time and damping for step size adjusted. 
            No iniphase but conditional update of pc. 
  05/03/15: in ccov-setting mucov replaced by mueff. 
  05/10/05: revise comment on resampling in example.c
  05/10/13: output of "coorstddev" changed from sigma * C[i][i] 
            to correct sigma * sqrt(C[i][i]).  
  05/11/09: Numerical problems are not anymore handled by increasing 
            sigma, but lead to satisfy a stopping criterion in 
            cmaes_Test(). 
  05/11/09: Update of eigensystem and test for numerical problems 
            moved right before sampling. 
  Version 2.27. 
*/

#include <math.h>   /* sqrt() */
#include <stddef.h> /* size_t */
#include <stdlib.h> /* NULL */
#include <string.h> /* strlen() */
#include <stdio.h>  /* sprintf(), NULL? */
#include <time.h>   /* clock_t */
#include "cmaes_interface.h"

/* --------------------------------------------------------- */
/* ------------------- Declarations ------------------------ */
/* --------------------------------------------------------- */

/* ------------------- External Visibly -------------------- */

void   random_init( random_t *, long int seed /*= -1*/);
void   random_exit( random_t *);
double random_Gauss( random_t *); /* (0,1)-normally distributed */
double random_Uniform( random_t *);
void   random_Start( random_t *, long int seed /*= -1*/);

void readpara_init (readpara_t *, int dim, int seed,  const double * xstart, 
		    const double * rgcoordist, int lambda, int mu, const char * filename); //MT
void readpara_exit(readpara_t *);
void readpara_ReadFromFile(readpara_t *, const char *szFileName);
void readpara_SupplementDefaults(readpara_t *);
void readpara_SetWeights(readpara_t *, const char * mode);
void readpara_WriteToFile(readpara_t *, const char *filenamedest, 
			  const char *parafilesource);

void cmaes_WriteToFilePtr(cmaes_t *t, const char *key, FILE *fp);
void cmaes_ReadFromFilePtr( cmaes_t *t, FILE *fp);

/* ------------------- Local visibly ----------------------- */

static void TestMinStdDevs( cmaes_t *);
static void WriteMaxErrorInfo( cmaes_t *);

static int  Eigen( int N,  double **C, double *diag, double **Q, 
		  int niter/*=0*/, double *rgtmp/*=NULL*/);
static int  QLalgo( int N, double *diag, double **mq, 
		   int maxIter, double *neben);
static void Householder( int N, double **ma, double *diag, double *neben);
  
static void ERRORMESSAGE(char const *sz1, char const *s2, 
			 char const *s3, char const *s4);
static void FATAL(char const *sz1, char const *s2, 
		  char const *s3, char const *s4);

static void   Sorted_index( const double *rgFunVal, int *index, int n);
static int    SignOfDiff( const void *d1, const void * d2);
static double rgdouMax( const double *rgd, int len);
static double rgdouMin( const double *rgd, int len);
static double douMax( double d1, double d2);
static double douMin( double d1, double d2);
static int    intMin( int i, int j);
static int    MaxIdx( const double *rgd, int len);
static int    MinIdx( const double *rgd, int len);

static double * new_double( int n);
static void * new_void( size_t n, size_t size); 

/* --------------------------------------------------------- */
/* ---------------- Functions: cmaes_t --------------------- */
/* --------------------------------------------------------- */

void cmaes_init(cmaes_t *t, /* "this" */
		double(*pFun)(double *), 
		int dimension, 
		double *inxstart,
		double *inrgcoordist, 
		long inseed,
		int lambda, //MT
		int mu,
		const char *input_parameter_filename) 
{
  int i, j, N;
  double dtest, trace;
  t->pFitFunc = pFun;
  readpara_init (&t->sp, dimension, inseed, inxstart, inrgcoordist, lambda, mu, //MT
		 input_parameter_filename);
  random_init( &t->rand, t->sp.seed);
  N = t->sp.N; /* for convenience */
  
  /* initialization  */
  for (i = 0, trace = 0.; i < N; ++i)
    trace += t->sp.rgcoordist[i]*t->sp.rgcoordist[i];
  t->sigma = sqrt(trace/N); /* t->sp.mueff/(0.2*t->sp.mueff+sqrt(N)) * sqrt(trace/N); */

  t->maxErr = 1000;
  t->cErrors = 0; 
  t->sOutString = (char *)new_void(330, sizeof(char));

  t->chiN =  sqrt(N) * (1. - 1./(4.*N) + 1./(21.*N*N));
  t->flgEigensysIsUptodate = 1;
  t->genOfEigensysUpdate = 0;
  t->clockeigensum = 0;
  t->flgIniphase = 0; /* do not use iniphase */
  t->flgStop = 0;
  t->stopeval = t->sp.maxeval; 

  for (dtest = 1.; dtest && dtest < 1.1 * dtest; dtest *= 2.) 
    if (dtest == dtest + 1.)
      break;
  t->dMaxSignifKond = dtest / 100.;

  t->gen = 0;
  t->state = 0;
  t->dLastMinEWgroesserNull = 1.0;

  t->rgpc = new_double(N);
  t->rgps = new_double(N);
  t->rgdTmp = new_double(N+1);
  t->rgBDz = new_double(N);
  t->rgxmean = new_double(N+2); t->rgxmean[0] = N; ++t->rgxmean;
  t->rgxold = new_double(N);
  t->rgxbestever = new_double(N+3); t->rgxbestever[0] = N; ++t->rgxbestever; 
  t->rgout = new_double(N+2); t->rgout[0] = N; ++t->rgout;
  t->rgD = new_double(N);
  t->C = (double**)new_void(N, sizeof(double*));
  t->B = (double**)new_void(N, sizeof(double*));
  t->rgFuncValue = new_double(t->sp.lambda+1); 
  t->rgFuncValue[0]=t->sp.lambda; ++t->rgFuncValue;
  for (i = 0; i < N; ++i) {
      t->rgxmean[i] = t->sp.xstart[i]; 
      t->C[i] = new_double(i+1);
      t->B[i] = new_double(N);
    }
  t->index = (int *) new_void(t->sp.lambda, sizeof(int));
  t->rgrgx = (double **)new_void(t->sp.lambda, sizeof(double*));
  for (i = 0; i < t->sp.lambda; ++i) {
    t->rgrgx[i] = new_double(N+2);
    t->rgrgx[i][0] = N; 
    t->rgrgx[i]++;
  }

  /* Initialize newed space  */

  for (i = 0; i < N; ++i)
    for (j = 0; j < i; ++j)
       t->C[i][j] = t->B[i][j] = t->B[j][i] = 0.;
	
  for (i = 0; i < N; ++i)
    {
      t->B[i][i] = 1.;
      t->C[i][i] = t->rgD[i] = t->sp.rgcoordist[i] * sqrt(N / trace);
      t->C[i][i] *= t->C[i][i];
      t->rgpc[i] = t->rgps[i] = 0.;
    }
  if (strcmp(t->sp.resumefile, "_no_")  != 0)
    cmaes_resume_distribution(t, t->sp.resumefile);
} /* cmaes_init() */

/* --------------------------------------------------------- */
/* --------------------------------------------------------- */

void cmaes_resume_distribution(cmaes_t *t, const char *filename)
{
  int i, j, res, n; 
  double d; 
  FILE *fp = fopen( filename, "r"); 
  if(fp == NULL) {
    ERRORMESSAGE("cmaes_resume_distribution(): could not open '", 
		 filename, "'",0);
    return;
  }
  /* count number of "resume" entries */
  i = 0; res = 0;
  while (1) {
    if ((res = fscanf(fp, " resume %lg", &d)) == EOF)
      break;
    else if (res==0) 
      fscanf(fp, " %*s");
    else if(res > 0)
      i += 1;
  }

  /* go to last "resume" entry */
  n = i; i = 0; res = 0; rewind(fp);
  while (i<n) {
    if ((res = fscanf(fp, " resume %lg", &d)) == EOF)
      FATAL("cmaes_resume_distribution(): Unexpected error, bug",0,0,0); 
    else if (res==0) 
      fscanf(fp, " %*s");
    else if(res > 0)
      ++i;
  }
  if (d != t->sp.N)
    FATAL("cmaes_resume_distribution(): Dimension numbers do not match",0,0,0); 
  
  /* find next "xmean" entry */  
  while (1) {
    if ((res = fscanf(fp, " xmean %lg", &d)) == EOF)
      FATAL("cmaes_resume_distribution(): 'xmean' not found",0,0,0); 
    else if (res==0) 
      fscanf(fp, " %*s");
    else if(res > 0)
      break;
  }
  
  /* read xmean */
  t->rgxmean[0] = d; res = 1; 
  for(i = 1; i < t->sp.N; ++i)
    res += fscanf(fp, " %lg", &t->rgxmean[i]);
  if (res != t->sp.N)
    FATAL("cmaes_resume_distribution(): xmean: dimensions differ",0,0,0); 

  /* find next "path for sigma" entry */  
  while (1) {
    if ((res = fscanf(fp, " path for sigma %lg", &d)) == EOF)
      FATAL("cmaes_resume_distribution(): 'path for sigma' not found",0,0,0); 
    else if (res==0) 
      fscanf(fp, " %*s");
    else if(res > 0)
      break;
  }
  
  /* read ps */
  t->rgps[0] = d; res = 1;
  for(i = 1; i < t->sp.N; ++i)
    res += fscanf(fp, " %lg", &t->rgps[i]);
  if (res != t->sp.N)
    FATAL("cmaes_resume_distribution(): ps: dimensions differ",0,0,0); 
  
  /* find next "path for C" entry */  
  while (1) {
    if ((res = fscanf(fp, " path for C %lg", &d)) == EOF)
      FATAL("cmaes_resume_distribution(): 'path for C' not found",0,0,0); 
    else if (res==0) 
      fscanf(fp, " %*s");
    else if(res > 0)
      break;
  }
  /* read pc */
  t->rgpc[0] = d; res = 1;
  for(i = 1; i < t->sp.N; ++i)
    res += fscanf(fp, " %lg", &t->rgpc[i]);
  if (res != t->sp.N)
    FATAL("cmaes_resume_distribution(): pc: dimensions differ",0,0,0); 

  /* find next "sigma" entry */  
  while (1) {
    if ((res = fscanf(fp, " sigma %lg", &d)) == EOF)
      FATAL("cmaes_resume_distribution(): 'sigma' not found",0,0,0); 
    else if (res==0) 
      fscanf(fp, " %*s");
    else if(res > 0)
      break;
  }
  t->sigma = d;

  /* find next entry "covariance matrix" */
  while (1) {
    if ((res = fscanf(fp, " covariance matrix %lg", &d)) == EOF)
      FATAL("cmaes_resume_distribution(): 'covariance matrix' not found",0,0,0); 
    else if (res==0) 
      fscanf(fp, " %*s");
    else if(res > 0)
      break;
  }
  /* read C */
  t->C[0][0] = d; res = 1;
  for (i = 1; i < t->sp.N; ++i)
    for (j = 0; j <= i; ++j)
      res += fscanf(fp, " %lg", &t->C[i][j]);
  if (res != (t->sp.N*t->sp.N+t->sp.N)/2)
    FATAL("cmaes_resume_distribution(): C: dimensions differ",0,0,0); 
   
  t->flgIniphase = 0;
  t->flgEigensysIsUptodate = 0;
  cmaes_UpdateEigensystem(t, 1);
  
} /* cmaes_resume_distribution() */
/* --------------------------------------------------------- */
/* --------------------------------------------------------- */

void cmaes_exit(cmaes_t *t)
{
  int i, N = t->sp.N;
  
  free( t->rgpc);
  free( t->rgps);
  free( t->rgdTmp);
  free( t->rgBDz);
  free( --t->rgxmean);
  free( t->rgxold); 
  free( --t->rgxbestever); 
  free( --t->rgout); 
  free( t->rgD);
  for (i = 0; i < N; ++i) {
    free( t->C[i]);
    free( t->B[i]);
  }
  for (i = 0; i < t->sp.lambda; ++i) 
    free( --t->rgrgx[i]);
  free( t->rgrgx); 
  free( t->C);
  free( t->B);
  free( t->index);
  free( --t->rgFuncValue);
  free( t->sOutString);
  random_exit (&t->rand);
  readpara_exit (&t->sp); 
} /* cmaes_exit() */


/* --------------------------------------------------------- */
/* --------------------------------------------------------- */
double * const * 
cmaes_SampleDistribution(cmaes_t *t, const double *xmean)
{
  int iNk, i, j, N=t->sp.N;
  double sum;

  if (xmean == NULL)
    xmean = t->rgxmean; 
  else if (xmean != t->rgxmean)
    for(i = 0; i < N; ++i)
      t->rgxmean[i] = xmean[i];
 
  /* calculate eigensystem  */
  cmaes_UpdateEigensystem(t, 0);

  /* treat minimal standard deviations and numeric problems */
  TestMinStdDevs(t);

  for (iNk = 0; iNk < t->sp.lambda; ++iNk)
    { /* generate scaled random vector (D * z)    */
      for (i = 0; i < N; ++i)
	t->rgdTmp[i] = t->rgD[i] * random_Gauss(&t->rand);
      /* add mutation (sigma * B * (D*z)) */
      for (i = 0; i < N; ++i) {
	for (j = 0, sum = 0.; j < N; ++j)
	  sum += t->B[i][j] * t->rgdTmp[j];
	t->rgrgx[iNk][i] = xmean[i] + t->sigma * sum;
      }
    }
  if(t->state == 3 || t->gen == 0)
    ++t->gen;
  t->state = 1; 

  return(t->rgrgx);
} /* SampleDistribution() */

/* --------------------------------------------------------- */
/* --------------------------------------------------------- */
double * 
cmaes_ReSampleSingle( cmaes_t *t, double *rgx)
{
  int i, j, N=t->sp.N;
  double sum; 

  if (rgx == NULL)
    FATAL("cmaes_ReSampleSingle(): Missing input double *rgx",0,0,0);

  for (i = 0; i < N; ++i)
    t->rgdTmp[i] = t->rgD[i] * random_Gauss(&t->rand);
  /* add mutation (sigma * B * (D*z)) */
  for (i = 0; i < N; ++i) {
    for (j = 0, sum = 0.; j < N; ++j)
      sum += t->B[i][j] * t->rgdTmp[j];
    rgx[i] = t->rgxmean[i] + t->sigma * sum;
  }
  return rgx;
}

/* --------------------------------------------------------- */
/* --------------------------------------------------------- */
double *
cmaes_EvaluateSample(cmaes_t *t, double(*pFun)(double *))
{
  int i, N=t->sp.N;
  if(pFun==NULL)
    pFun = t->pFitFunc; 
  if(pFun==NULL)
    FATAL("cmaes_EvaluateSample() : pointer to function missing.",0,0,0);
  for(i = 0; i < t->sp.lambda; ++i) {
    t->rgFuncValue[i] = t->rgrgx[i][N] = (*pFun)(t->rgrgx[i]);
  }
  if(t->state == 3 || t->gen == 0)
    ++t->gen;
  t->state = 2; 
  return(t->rgFuncValue);
}
/* --------------------------------------------------------- */
/* --------------------------------------------------------- */
double *
cmaes_ReestimateDistribution( cmaes_t *t, const double *rgFunVal)
{
  int i, j, k, iNk, hsig, N=t->sp.N;
  double sum, tfac; 
  double psxps; 
  
  if(t->state == 3)
    FATAL("cmaes_ReestimateDistribution(): You need to call \n",
	  "SampleDistribution() before reestimation can take place.",0,0);
  if(rgFunVal == NULL) {
    if(t->state == 1) /* EvaluateSample() was not called */
      FATAL("cmaes_ReestimateDistribution(): ", 
	    "No function values available.",0,0);
    else
      rgFunVal = t->rgFuncValue; 
  }
  else /* take values from rgFunVal */
    for (i=0; i < t->sp.lambda; ++i) 
      t->rgrgx[i][N] = t->rgFuncValue[i] = rgFunVal[i];
  

  /* Generate index */
  Sorted_index(rgFunVal, t->index, t->sp.lambda);
  
  /* Test if function values are identical, escape flat fitness */
  if (t->rgFuncValue[t->index[0]] == 
      t->rgFuncValue[t->index[(int)t->sp.lambda/2]]) {
    t->sigma *= exp(0.2+1./t->sp.damp);
    ERRORMESSAGE("Warning: sigma increased due to equal function values\n",0,0,0);
  }
  
  /* update function value history */
  for(i = 3; i > 0; --i)
    t->arFuncValueHist[i] = t->arFuncValueHist[i-1];
  t->arFuncValueHist[0] = rgFunVal[t->index[0]];

  /* update xbestever */
  if (t->rgxbestever[N] > t->rgrgx[t->index[0]][N] || t->gen == 1)
    for (i = 0; i <= N; ++i) {
      t->rgxbestever[i] = t->rgrgx[t->index[0]][i];
      t->rgxbestever[N+1] = t->gen * t->sp.lambda;
    }

  /* calculate xmean and rgBDz~N(0,C) */
  for (i = 0; i < N; ++i) {
    t->rgxold[i] = t->rgxmean[i]; 
    t->rgxmean[i] = 0.;
    for (iNk = 0; iNk < t->sp.mu; ++iNk) 
      t->rgxmean[i] += t->sp.weights[iNk] * t->rgrgx[t->index[iNk]][i];
    t->rgBDz[i] = sqrt(t->sp.mueff)*(t->rgxmean[i] - t->rgxold[i])/t->sigma; 
  }

  /* calculate z := D^(-1) * B^(-1) * rgBDz into rgdTmp */
  for (i = 0; i < N; ++i) {
    for (j = 0, sum = 0.; j < N; ++j)
      sum += t->B[j][i] * t->rgBDz[j];
    t->rgdTmp[i] = sum / t->rgD[i];
  }
  
  /* cumulation for sigma (ps) using B*z */
  for (i = 0; i < N; ++i) {
    for (j = 0, sum = 0.; j < N; ++j)
      sum += t->B[i][j] * t->rgdTmp[j];
    t->rgps[i] = (1. - t->sp.ccumsig) * t->rgps[i] + 
      sqrt(t->sp.ccumsig * (2. - t->sp.ccumsig)) * sum;
  }
  
  /* calculate norm(ps)^2 */
  for (i = 0, psxps = 0.; i < N; ++i)
    psxps += t->rgps[i] * t->rgps[i];

  /* cumulation for covariance matrix (pc) using B*D*z~N(0,C) */
  hsig = sqrt(psxps) / sqrt(1. - pow(1.-t->sp.ccumsig, 2*t->gen)) / t->chiN
    < 1.5 + 1./(N-0.5);
  for (i = 0; i < N; ++i) {
    t->rgpc[i] = (1. - t->sp.ccumcov) * t->rgpc[i] + 
      hsig * sqrt(t->sp.ccumcov * (2. - t->sp.ccumcov)) * t->rgBDz[i];
  }
  
  /* stop initial phase */
  if (t->flgIniphase && 
      t->gen > douMin(1/t->sp.ccumsig, 1+N/t->sp.mucov)) 
    {
      if (psxps / (t->sp.ccumsig*t->sp.damp) 
	  / (1.-pow((1. - t->sp.ccumsig), t->gen)) < N * 1.05) 
	t->flgIniphase = 0;
    }
  
  /* remove momentum in ps, if ps is large and fitness is getting worse */
  /* TODO: reconsider (remove?) this. It is obsolete due to hsig and harmful 
     in a dynamic environment */
  if(psxps/N > 1.5 + 10.*sqrt(2./N) 
     && t->arFuncValueHist[0] > t->arFuncValueHist[1]
     && t->arFuncValueHist[0] > t->arFuncValueHist[2]) {
    tfac = sqrt((1 + douMax(0, log(psxps/N))) * N / psxps);
    for (i=0; i<N; ++i) 
      t->rgps[i] *= tfac;
    psxps *= tfac*tfac; 
  }
  
  /* update of C  */
  /* Adapt_C(t); not used anymore */
  if (t->sp.ccov != 0. && t->flgIniphase ==0) {
    t->flgEigensysIsUptodate = 0;

    /* update covariance matrix */
    for (i = 0; i < N; ++i)
      for (j = 0; j <=i; ++j) {
	t->C[i][j] = (1 - t->sp.ccov) * t->C[i][j] 
	  + t->sp.ccov * (1./t->sp.mucov) 
	    * (t->rgpc[i] * t->rgpc[j] 
               + (1-hsig)*t->sp.ccumcov*(2.-t->sp.ccumcov) * t->C[i][j]);
	for (k = 0; k < t->sp.mu; ++k) /* additional rank mu update */
	  t->C[i][j] += t->sp.ccov * (1-1./t->sp.mucov) * t->sp.weights[k]  
	    * (t->rgrgx[t->index[k]][i] - t->rgxold[i]) 
	    * (t->rgrgx[t->index[k]][j] - t->rgxold[j])
	    / t->sigma / t->sigma; 
      }
  }

  /* update of sigma */
  t->sigma *= exp(((sqrt(psxps)/t->chiN)-1.)/t->sp.damp);

  t->state = 3;

  return (t->rgxmean);

} /* cmaes_ReestimateDistribution() */


/* --------------------------------------------------------- */
/* --------------------------------------------------------- */

#if 0
void 
Adapt_C( cmaes_t *t)
{
  int i, j, k, N=t->sp.N; 

  if (t->sp.ccov == 0. || t->flgIniphase)
    return;
  
  t->flgEigensysIsUptodate = 0;

  /* update covariance matrix */
  for (i = 0; i < N; ++i)
    for (j = 0; j <=i; ++j) {
      t->C[i][j] = (1 - t->sp.ccov) * t->C[i][j] 
	+ t->sp.ccov * (1./t->sp.mucov) * t->rgpc[i] * t->rgpc[j];
      for (k = 0; k < t->sp.mu; ++k) /* additional rank mu update */
	t->C[i][j] += t->sp.ccov * (1-1./t->sp.mucov) * t->sp.weights[k]  
	  * (t->rgrgx[t->index[k]][i] - t->rgxold[i]) 
	  * (t->rgrgx[t->index[k]][j] - t->rgxold[j])
	  / t->sigma / t->sigma; 
    }
  /*
    {
      double trace; 
      for (i = 0, trace = 0; i < N; ++i)
	trace += t->C[i][i];
      trace = trace / N; 
      for (i = 0; i < N; ++i)
	for (j = 0; j <=i; ++j)
	  t->C[i][j] /= trace;
      printf("trace/N: %f\n", trace);
    }
  */
} /* Adapt_C() */
#endif 

/* --------------------------------------------------------- */
/* --------------------------------------------------------- */
static void 
TestMinStdDevs(cmaes_t *t)
  /* modifies sigma and C */
{
  int iKoo, N = t->sp.N; 
  if (t->sp.rgMinCoorStddev == NULL)
    return;

  for (iKoo = 0; iKoo < N; ++iKoo)
    if (t->sigma * sqrt(t->C[iKoo][iKoo]) < t->sp.rgMinCoorStddev[iKoo]) 
      break;

  if (iKoo < N) {
    t->sigma *= exp(0.05+1./t->sp.damp);
  }
} /* cmaes_TestMinStdDevs() */

/* --------------------------------------------------------- */
/* --------------------------------------------------------- */
void cmaes_WriteToFile(cmaes_t *t, const char *key, const char *name)
{ 
  const char *s = "tmpcmaes.dat"; 
  FILE *fp;
  if (name == NULL)
    name = s; 
  fp = fopen( name, "a"); 
  if(fp == NULL) {
    ERRORMESSAGE("cmaes_WriteToFile(): could not open '", name, "'",0);
    return;
  }
  cmaes_WriteToFilePtr(t, key, fp);
  fclose(fp);
} /* WriteToFile */

/* --------------------------------------------------------- */
void cmaes_WriteToFilePtr(cmaes_t *t, const char *key, FILE *fp)
{ 
  double min, max; 
  int i, k, N=t->sp.N;
  const char *s = "few"; 
  if (key == NULL)
    key = s; 
  
  do /* while *key=='+' */
    {
      if (strncmp(key, "axisratio", 9) == 0)
	{
	  min = rgdouMin(t->rgD, N);
	  fprintf(fp, "%.2e", rgdouMax(t->rgD, N)/min);
	  key += 9; fprintf(fp, "%c", (*key=='+') ? '\t':'\n');
	}
      /* new coordinate system == all eigenvectors */
      if (strncmp(key, "B", 1) == 0) 
	{
	  //int j, index[N]; 
	  int j, *index=(int*)(calloc(N,sizeof(int))); //MT
	  Sorted_index(t->rgD, index, N);
	  /* One eigenvector per row, sorted: largest eigenvalue first */
	  for (i = 0; i < N; ++i)
	    for (j = 0; j < N; ++j)
	      fprintf(fp, "%g%c", t->B[j][index[N-1-i]], (j==N-1)?'\n':'\t');
	  ++key;
	  free(index); //MT
	}
      /* covariance matrix */
      if (strncmp(key, "C", 1) == 0) 
	{
	  int j;
	  for (i = 0; i < N; ++i)
	    for (j = 0; j <= i; ++j)
	      fprintf(fp, "%g%c", t->C[i][j], (j==i)?'\n':'\t');
	  ++key; 
	}
      /* clock value, i.e. processor time used since begin of execution */ 
      if (strncmp(key, "clock", 4) == 0)
	{
	  fprintf(fp, "%.1f", (double)(clock())/CLOCKS_PER_SEC);
	  key += 5; fprintf(fp, "%c", (*key=='+') ? '\t':'\n');
	}

      /* standard deviations in coordinate directions (sigma*sqrt(C[i,i])) */
      if (strncmp(key, "coorstddev", 10) == 0) /* std dev in coordinate axes */
	{
	  for (i = 0; i < N; ++i)
	    fprintf(fp, "%s%g", (i==0) ? "":"\t", t->sigma*sqrt(t->C[i][i]));
	  key += 10; fprintf(fp, "%c", (*key=='+') ? '\t':'\n');
	}
      /* diagonal of D == roots of eigenvalues, sorted */
      if (strncmp(key, "diag(D)", 7) == 0)
	{
	  for (i = 0; i < N; ++i)
	    t->rgdTmp[i] = t->rgD[i];
	  qsort(t->rgdTmp, N, sizeof(double), &SignOfDiff);
	  for (i = 0; i < N; ++i)
	    fprintf(fp, "%s%g", (i==0) ? "":"\t", t->rgdTmp[N-1-i]);
	  key += 7; fprintf(fp, "%c", (*key=='+') ? '\t':'\n');
	}
      if (strncmp(key, "dimension", 9) == 0)
	{
	  fprintf(fp, "%d", N);
	  key += 9; fprintf(fp, "%c", (*key=='+') ? '\t':'\n');
	}
      if (strncmp(key, "dim", 3) == 0)
	{
	  fprintf(fp, "%d", N);
	  key += 4; fprintf(fp, "%c", (*key=='+') ? '\t':'\n');
	}
      if (strncmp(key, "eval", 4) == 0)
	{
	  fprintf(fp, "%.0f", t->gen*t->sp.lambda);
	  key += 4; fprintf(fp, "%c", (*key=='+') ? '\t':'\n');
	}
      if (strncmp(key, "few(diag(D))", 12) == 0)/* between four and six axes */
	{ 
	  int add = (int)(0.5 + (N + 1.) / 5.); 
	  for (i = 0; i < N; ++i)
	    t->rgdTmp[i] = t->rgD[i];
	  qsort(t->rgdTmp, N, sizeof(double), &SignOfDiff);
	  for (i = 0; i < N-1; i+=add)        /* print always largest */
	    fprintf(fp, "%s%g", (i==0) ? "":"\t", t->rgdTmp[N-1-i]);
	  fprintf(fp, "\t%g\n", t->rgdTmp[0]);        /* and smallest */
	  break; 
	}
      if (strncmp(key, "fewinfo", 7) == 0) { 
	fprintf(fp," Fevals Function Value         Sigma   ");
	fprintf(fp, "MaxCoorDev MinCoorDev AxisRatio MinDii\n");
	key += 7; 
      }
      if (strncmp(key, "few", 3) == 0) { 
	fprintf(fp, " %5.0f ", t->gen*t->sp.lambda); 
	fprintf(fp, "%.15e", t->rgFuncValue[t->index[0]]);
	max = t->C[0][0]; for(i=1;i<N;++i) max=(max<t->C[i][i])?t->C[i][i]:max;
	min = t->C[0][0]; for(i=1;i<N;++i) min=(min>t->C[i][i])?t->C[i][i]:min;
	fprintf(fp, "  %.2e  %.2e %.2e", t->sigma, t->sigma*sqrt(max), 
		t->sigma*sqrt(min));
	min = rgdouMin(t->rgD, N);
	fprintf(fp, "  %.2e  %.2e", rgdouMax(t->rgD, N)/min, min);
	key += 3; fprintf(fp, "%c", (*key=='+') ? '\t':'\n');
      }
      if (strncmp(key, "funval", 6) == 0)
	{
	  fprintf(fp, "%.0f", t->rgFuncValue[t->index[0]]);
	  key += 6; fprintf(fp, "%c", (*key=='+') ? '\t':'\n');
	}
      if (strncmp(key, "arfunval", 6) == 0)
	{
	  for (i = 0; i < N; ++i)
	    fprintf(fp, "%s%.0f", (i==0) ? "" : "\t", 
		    t->rgFuncValue[t->index[i]]);
	  key += 8; fprintf(fp, "%c", (*key=='+') ? '\t':'\n');
	}
      if (strncmp(key, "gen", 1) == 0)
	{
	  fprintf(fp, "%.0f", t->gen);
	  key += 3; fprintf(fp, "%c", (*key=='+') ? '\t':'\n');
	}
      if (strncmp(key, "lambda", 1) == 0)
	{
	  fprintf(fp, "%d", t->sp.lambda);
	  key += 6; fprintf(fp, "%c", (*key=='+') ? '\t':'\n');
	}
      if (strncmp(key, "N", 1) == 0)
	{
	  fprintf(fp, "%d", N);
	  key += 1; fprintf(fp, "%c", (*key=='+') ? '\t':'\n');
	}
      if (strcmp(key, "resume") == 0)
	{
	  fprintf(fp, "\n# resume %d\n", N);
	  fprintf(fp, "xmean\n");
	  cmaes_WriteToFilePtr(t, "xmean", fp);
	  fprintf(fp, "path for sigma\n");
	  for(i=0; i<N; ++i)
	    fprintf(fp, "%g%s", t->rgps[i], (i==N-1) ? "\n":"\t");
	  fprintf(fp, "path for C\n");
	  for(i=0; i<N; ++i)
	    fprintf(fp, "%g%s", t->rgpc[i], (i==N-1) ? "\n":"\t");
	  fprintf(fp, "sigma %g\n", t->sigma);
	  /* note than B and D might not be uptodate */
	  fprintf(fp, "covariance matrix\n"); 
	  cmaes_WriteToFilePtr(t, "C", fp);
	}
      if (strcmp(key, "xbest") == 0) { /* best x in recent generation */
	for(i=0; i<N; ++i)
	  fprintf(fp, "%s%g", (i==0) ? "":"\t", t->rgrgx[t->index[0]][i]);
	key += 5; fprintf(fp, "%c", (*key=='+') ? '\t':'\n');
      }
      if (strcmp(key, "xmean") == 0) {
	for(i=0; i<N; ++i)
	  fprintf(fp, "%s%g", (i==0) ? "":"\t", t->rgxmean[i]);
	key += 5; fprintf(fp, "%c", (*key=='+') ? '\t':'\n');
      }
      if (strcmp(key, "all") == 0)
	{
	  time_t ti = time(NULL);
	  fprintf(fp, "\n# --------- %s\n", asctime(localtime(&ti)));
	  fprintf(fp, " N %d\n", N);
	  fprintf(fp, " seed %d\n", t->sp.seed);
	  fprintf(fp, "function evaluations %.0f\n", t->gen*t->sp.lambda);
	  fprintf(fp, "elapsed CPU time [s] %.2f\n", 
		  (double) clock() / CLOCKS_PER_SEC);
	  fprintf(fp, "function value f(x)=%g\n", t->rgrgx[t->index[0]][N]);
	  max = t->C[0][0]; for(i=1;i<N;++i) max=(max<t->C[i][i])?t->C[i][i]:max;
	  min = t->C[0][0]; for(i=1;i<N;++i) min=(min>t->C[i][i])?t->C[i][i]:min;
	  fprintf(fp, "maximal standard deviation %g\n", t->sigma*sqrt(max));
	  fprintf(fp, "minimal standard deviation %g\n", t->sigma*sqrt(min));
	  fprintf(fp, "sigma %g\n", t->sigma);
	  fprintf(fp, "axisratio %g\n", rgdouMax(t->rgD, N)/rgdouMin(t->rgD, N));
	  fprintf(fp, "xbestever found after %.0f evaluations, function value %g\n", 
		  t->rgxbestever[N+1], t->rgxbestever[N]);
	  for(i=0; i<N; ++i)
	    fprintf(fp, " %12g%c", t->rgxbestever[i], 
		    (i%5==4||i==N-1)?'\n':' ');
	  fprintf(fp, "xbest (of last generation, function value %g)\n", 
		  t->rgrgx[t->index[0]][N]); 
	  for(i=0; i<N; ++i)
	    fprintf(fp, " %12g%c", t->rgrgx[t->index[0]][i], 
		    (i%5==4||i==N-1)?'\n':' ');
	  fprintf(fp, "xmean \n");
	  for(i=0; i<N; ++i)
	    fprintf(fp, " %12g%c", t->rgxmean[i], 
		    (i%5==4||i==N-1)?'\n':' ');
	  fprintf(fp, "Standard deviation of coordinate axes (sigma*sqrt(diag(C)))\n");
	  for(i=0; i<N; ++i)
	    fprintf(fp, " %12g%c", t->sigma*sqrt(t->C[i][i]), 
		    (i%5==4||i==N-1)?'\n':' ');
	  fprintf(fp, "Main axis lengths of mutation ellipsoid (sigma*diag(D))\n");
	  for (i = 0; i < N; ++i)
	    t->rgdTmp[i] = t->rgD[i];
	  qsort(t->rgdTmp, N, sizeof(double), &SignOfDiff);
	  for(i=0; i<N; ++i)
	    fprintf(fp, " %12g%c", t->sigma*t->rgdTmp[N-1-i], 
		    (i%5==4||i==N-1)?'\n':' ');
	  fprintf(fp, "Longest axis (b_i where d_ii=max(diag(D))\n");
	  k = MaxIdx(t->rgD, N);
	  for(i=0; i<N; ++i)
	    fprintf(fp, " %12g%c", t->B[i][k], (i%5==4||i==N-1)?'\n':' ');
	  fprintf(fp, "Shortest axis (b_i where d_ii=max(diag(D))\n");
	  k = MinIdx(t->rgD, N);
	  for(i=0; i<N; ++i)
	    fprintf(fp, " %12g%c", t->B[i][k], (i%5==4||i==N-1)?'\n':' ');
	  key += 3; /* "all" */
	}
    } while (*key++=='+');
  
} /* WriteToFilePtr */

/* --------------------------------------------------------- */
double  
cmaes_Get( cmaes_t *t, char const *s)
{
  double min, max;
  int i, N=t->sp.N;

  if (strncmp(s, "axisratio", 5) == 0) {
    return (rgdouMax(t->rgD, N)/rgdouMin(t->rgD, N));
  }
  else if (strncmp(s, "eval", 4) == 0) { /* number of function evaluations */
    return (t->gen * t->sp.lambda);
  }
  else if (strncmp(s, "fctvalue", 6) == 0) {
    return(t->rgFuncValue[t->index[0]]);
  }
  else if (strncmp(s, "funcvalue", 6) == 0) {
    return(t->rgFuncValue[t->index[0]]);
  }
  else if (strncmp(s, "funvalue", 6) == 0) {
    return(t->rgFuncValue[t->index[0]]);
  }
  else if (strncmp(s, "generation", 3) == 0) {
    return(t->gen);
  }
  else if (strncmp(s, "lambda", 3) == 0) {
    return(t->sp.lambda);
  }
  else if (strncmp(s, "maxeval", 4) == 0) {
    return(t->sp.maxeval);
  }
  else if (strncmp(s, "maxgen", 4) == 0) {
    return(ceil(t->sp.maxeval / t->sp.lambda));
  }
  else if (strncmp(s, "maxmainaxislength", 4) == 0) {
    return(t->sigma * rgdouMax(t->rgD, N));
  }
  else if (strncmp(s, "maxstddev", 4) == 0) {
    max = t->sigma * sqrt(t->C[0][0]);
    for (i = 1; i < N; ++i)
      if (max < t->sigma * sqrt(t->C[i][i]))
	max = t->sigma * sqrt(t->C[i][i]);
    return(max);
  }
  else if (strncmp(s, "minmainaxislength", 4) == 0) {
    return(t->sigma * rgdouMin(t->rgD, N));
  }
  else if (strncmp(s, "minstddev", 4) == 0) {
    min = t->sigma * sqrt(t->C[0][0]);
    for (i = 1; i < N; ++i)
      if (min > t->sigma * sqrt(t->C[i][i]))
	min = t->sigma * sqrt(t->C[i][i]);
    return(min);
  }
  else if (strcmp(s, "N") == 0 || strcmp(s, "n") == 0 || 
	   strncmp(s, "dimension", 3) == 0) {
    return (N);
  }
  else if (strncmp(s, "samplesize", 8) == 0) {
    return(t->sp.lambda);
  }
  else if (strncmp(s, "sigma", 3) == 0) {
    return(t->sigma);
  }
  ERRORMESSAGE( "cmaes_Get(): No match found for '", s, "'",0);
  return(0);
} /* cmaes_Get() */

/* --------------------------------------------------------- */
const double * 
cmaes_Getp( cmaes_t *t, char const *s)
{
  int i, N=t->sp.N;
  if (strncmp(s, "diag(C)", 7) == 0) {
    for (i = 0; i < N; ++i)
      t->rgout[i] = t->C[i][i]; 
    return(t->rgout);
  }
  else if (strncmp(s, "diag(D)", 7) == 0) {
    return(t->rgD);
  }
  else if (strncmp(s, "stddev", 3) == 0) {
    for (i = 0; i < N; ++i)
      t->rgout[i] = t->sigma * sqrt(t->C[i][i]); 
    return(t->rgout);
  }
  else if (strncmp(s, "xbestever", 7) == 0)
    return(t->rgxbestever);
  else if (strcmp(s, "xbest") == 0)
    return(t->rgrgx[t->index[0]]);
  else if (strncmp(s, "xmean", 1) == 0)
    return(t->rgxmean);
  
  return(NULL);
}

/* --------------------------------------------------------- */
const char *
cmaes_Test( cmaes_t *t, const char *s)
{
  double max, range, fac, minEW, maxEW;
  int iAchse, iKoo;
  static char sTestOutString[324];
  char * cp = sTestOutString;
  int i, N=t->sp.N; 
  cp[0] = '\0';

  if (strcmp(s, "stop") == 0) 
    {
      /* function value reached */
      if ((t->gen > 1 || t->state > 1) && t->sp.stStopFuncValue.flg && 
	  t->rgFuncValue[t->index[0]] <= t->sp.stStopFuncValue.val) 
	cp += sprintf(cp, " Function value %7.2e <= stopfunval (%7.2e)\n", 
		      t->rgFuncValue[t->index[0]], t->sp.stStopFuncValue.val);
      
      /* TolFun */
      range = douMax(rgdouMax(t->arFuncValueHist, 4), 
		     rgdouMax(t->rgFuncValue, t->sp.lambda)) -
	douMin(rgdouMin(t->arFuncValueHist, 4), 
	       rgdouMin(t->rgFuncValue, t->sp.lambda));
      if (t->gen > 4 && range < t->sp.stopFuncValDiff)
	{
	  cp += sprintf(cp, 
             " Function value differences %7.2e < stopfunvaldiff (%7.2e)\n", 
			range, t->sp.stopFuncValDiff);
	}

      /* Condition of C greater than dMaxSignifKond */
      minEW = rgdouMin(t->rgD, N);
      maxEW = rgdouMax(t->rgD, N);

      if (1.001 * maxEW >= minEW * sqrt(t->dMaxSignifKond)) {
	  cp += sprintf(cp, 
             " Maximal condition number %7.2e reached\n", 
			t->dMaxSignifKond);
	  } /* if */

      /* Principal axis i has no effect on xmean, ie. x == x + 0.1 * sigma * rgD[i] * B[i] */
      for (iAchse = 0; iAchse < N; ++iAchse)
	{
	  fac = 0.1 * t->sigma * t->rgD[iAchse];
	  for (iKoo = 0; iKoo < N; ++iKoo){ 
	    if (t->rgxmean[iKoo] != t->rgxmean[iKoo] + fac * t->B[iKoo][iAchse])
	      break;
	  }
	  if (iKoo == N)	
	    {
	      /* t->sigma *= exp(0.2+1./t->sp.damp); */
	      cp += sprintf(cp, 
			    " Standard deviation 0.1*%7.2e in principal axis %d without effect\n", 
			    fac/0.1, iAchse);
	      break;
	    } /* if (iKoo == N) */
	} /* for iAchse	     */

      /* Component of xmean is not changed anymore */
      for (iKoo = 0; iKoo < N; ++iKoo)
	{
	  if (t->rgxmean[iKoo] == t->rgxmean[iKoo] + 
	      0.2*t->sigma*sqrt(t->C[iKoo][iKoo]))
	    {
	      /* t->C[iKoo][iKoo] *= (1 + t->sp.ccov); */
	      /* flg = 1; */
	      cp += sprintf(cp, 
			    " Standard deviation 0.2*%7.2e in coordinate %d without effect\n", 
			    t->sigma*sqrt(t->C[iKoo][iKoo]), iKoo); 
	      break;
	    }
	  
	} /* for iKoo */
      /* if (flg) t->sigma *= exp(0.05+1./t->sp.damp); */

      max = t->C[0][0]; for(i=1;i<N;++i) max=douMax(max,t->C[i][i]); /* useless? */
      if(t->gen*t->sp.lambda >= t->sp.maxeval) 
	cp += sprintf(cp, " Function evaluations %.0f >= maxeval\n", 
		      t->gen*t->sp.lambda);
      else if(t->gen*t->sp.lambda >= t->stopeval) 
	cp += sprintf(cp, " Function evaluations %.0f >= stopeval=%.0f\n", 
		      t->gen*t->sp.lambda, t->stopeval);
      if(t->flgStop)
	cp += sprintf(cp, " Stop flag set\n");
    } /*  */
#if 0
  else if (0) {
    for(i=0, cTemp=0; i<N; ++i) {
      cTemp += (sigma * sqrt(C[i][i]) < stopdx) ? 1 : 0;
      cTemp += (sigma * rgpc[i] < stopdx) ? 1 : 0;
    }
    if (cTemp == 2*N)
      flgStop = 1;
  }
#endif

  if (cp - sTestOutString>320)
    ERRORMESSAGE("Bug in cmaes_t:Test(): sTestOutString too short",0,0,0);

  if (cp != sTestOutString) {
    return sTestOutString;
  }

  return(NULL);
  
} /* cmaes_Test() */

/* --------------------------------------------------------- */
void cmaes_ReadSignals(cmaes_t *t, const char *filename)
{
  const char *s = "signals.par"; 
  FILE *fp;
  if (filename == NULL)
    filename = s;
  fp = fopen( filename, "r"); 
  if(fp == NULL) {
    return;
  }
  cmaes_ReadFromFilePtr( t, fp);
  fclose(fp);
}
/* --------------------------------------------------------- */
void cmaes_ReadFromFilePtr( cmaes_t *t, FILE *fp)
{
  const char *keys[5];
  char s[199], sin1[99], sin2[129], sin3[99], sin4[99];
  int ikey, ckeys, nb; 
  double d; 
  static time_t printtime = 0; 
  static time_t writetime = 0;
  static int flglockprint = 0;
  static int flglockwrite = 0;
  int flgprinted = 0;
  int flgwritten = 0;
  keys[0] = " stop %98s %98s";        /* s=="now" or "%lg"-number */
  keys[1] = " print %98s %98s";       /* s==keyword for WriteFile */
  keys[2] = " write %98s %128s %98s"; /* s1==keyword, s2==filename */
  ckeys = 3; 
  strcpy(sin2, "tmpcmaes.dat");

  if (t->flgStop) 
    {
      printtime = 0; /* forces printing */
      writetime = 0;
    }
  while(fgets(s, sizeof(s), fp) != NULL)
    { /* skip comments  */
      if (s[0] == '#' || s[0] == '%')
	continue;
      sin1[0] = sin2[0] = sin3[0] = sin4[0] = '\0';
      for (ikey=0; ikey < ckeys; ++ikey)
	{
	  if((nb=sscanf(s, keys[ikey], sin1, sin2, sin3, sin4)) >= 1) 
	    {
	      switch(ikey) {
	      case 0 : /* "stop" */
		if (strncmp(sin1, "now", 3) == 0) 
		  t->flgStop = 1; 
		else if (sscanf(sin1, "%lg", &d) == 1) 
		  t->stopeval = d; 
		else if (strncmp(sin1, "gen", 3) == 0)
		  if (sscanf(sin2, " %lg", &d) == 1) 
		    t->stopeval = d*t->sp.lambda; 
		break;
	      case 1 : /* "print" */
		d = 1; 
		sscanf(sin2, "%lg", &d);
		if (time(NULL)-printtime >= d && !flglockprint) {
		  cmaes_WriteToFilePtr(t, sin1, stdout);
		  flgprinted = 1;
		}
		if(d < 0) 
		  flglockprint += 2;
		break; 
	      case 2 : /* "write" */
		d = 1; 
		sscanf(sin3, "%lg", &d);
		if(d < 0) 
		  flglockwrite += 2;
		if (time(NULL)-writetime >= d && !flglockwrite) {
		  cmaes_WriteToFile(t, sin1, sin2);
		  flgwritten = 1; 
		}
		break; 
	      default :
		break; 
	      }
	      break; /* for ikey */
	    }
	} /* for */
    } /* while */
  if (flgprinted)
    printtime = time(NULL);
  if (flgwritten)
    writetime = time(NULL);
  --flglockprint;
  --flglockwrite;
  flglockprint = (flglockprint > 0) ? 1 : 0;
  flglockwrite = (flglockwrite > 0) ? 1 : 0;
} /*  cmaes_ReadFromFilePtr */ 

/* --------------------------------------------------------- */
/* --------------------------------------------------------- */
void 
cmaes_UpdateEigensystem(cmaes_t *t, short flgforce)
{
  double minEW, maxEW, summand, tmp;
  int i, N=t->sp.N;
  int cIterEig, iEigenCalcVers;
  const int maxIterEig = 30*N;
  const int maxEigenCalcVers = 10;
  clock_t clockeigenbegin;

  if(flgforce == 0) {
    if (t->flgEigensysIsUptodate == 1)
      return; 
    if (t->sp.updateCmode.flgalways == 0 /* not implemented, always ==0 */
	&& t->gen < t->genOfEigensysUpdate + t->sp.updateCmode.modulo
	)
      return; 
    if (t->clockeigensum > t->sp.updateCmode.maxtime*clock()/100.
	&& (double)(t->clockeigensum)/CLOCKS_PER_SEC > 0.1)
      return; 
  }
  clockeigenbegin = clock();
  /*  printf("\n%f / %f", double(clockeigensum)/CLOCKS_PER_SEC,  */
  /*	 (double)(clock())/CLOCKS_PER_SEC); */
  cIterEig = maxIterEig;
  minEW = maxEW = 0;

  /* Versuche fuer gueltige Berechnung */
  for (iEigenCalcVers = 0; iEigenCalcVers < maxEigenCalcVers; 
       ++iEigenCalcVers)
    {
      cIterEig = Eigen( N, t->C, t->rgD, t->B, maxIterEig, t->rgdTmp);
      
      if (cIterEig < maxIterEig) 
	{
	  /* Groessten und kleinsten Eigenwert finden */
	  minEW = rgdouMin(t->rgD, N);
	  maxEW = rgdouMax(t->rgD, N);

	  /* Limit Condition of C to dMaxSignifKond+1 */
	  if (maxEW > minEW * t->dMaxSignifKond) {
	    tmp = maxEW/t->dMaxSignifKond - minEW;
	    minEW += tmp;
	    for (i=0;i<N;++i) {
	      t->C[i][i] += tmp;
	      t->rgD[i] += tmp;
	    }
	  } /* if */

	  t->dLastMinEWgroesserNull = minEW;
	  
	  for (i = 0; i < N; ++i)
	    t->rgD[i] = sqrt(t->rgD[i]);
	  
	  t->flgEigensysIsUptodate = 1;
	  t->genOfEigensysUpdate = t->gen; 
	  t->clockeigensum += clock() - clockeigenbegin;
	  return;
	} /* if cIterEig < ... */
      
      /* Eigenzerlegung numerische nicht OK */
      {
	//char s[160+40*N]; s[0] = '\0';
	char *s=(char*)(calloc(160+40*N,1)); s[0] = '\0'; //MT
	sprintf( s, "\n%04d.Num.Error--Eigen-Decomp. invalid\n", t->cErrors+1);
	sprintf( s+strlen(s)," Gen      ==%f\n", t->gen);
	sprintf( s+strlen(s)," iter     ==%d\n", cIterEig);
	sprintf( s+strlen(s)," maxIter  ==%d\n", maxIterEig);
	sprintf( s+strlen(s)," maxEW    ==%e\n", maxEW);
	sprintf( s+strlen(s)," minEW    ==%e\n", minEW);
	sprintf( s+strlen(s)," lastminEW==%e\n", 
		 t->dLastMinEWgroesserNull);
	sprintf( s+strlen(s)," maxCond  ==%e\n", t->dMaxSignifKond);
	sprintf( s+strlen(s)," sigma    ==%e\n", t->sigma);
#if 0
	sprintf( s+strlen(s), " AxisMaxEW   AxisMinEW");
	for (i = 0; i < N; ++i)
	  sprintf (s+strlen(s), "\n %10.2e %10.2e", 
		   B[i][GetMaxEwIndex()], B[i][GetMinEwIndex()]);
#endif
	ERRORMESSAGE( s,0,0,0);
	if (++t->cErrors > t->maxErr)
	  {
	      WriteMaxErrorInfo(t);
	      FATAL("cmaes_UpdateEigensystem() : To many errors.",0,0,0);
	    }
	free(s); //MT
      }
      
      /* Addition des letzten minEW auf die Diagonale von C */
      summand = t->dLastMinEWgroesserNull * exp(iEigenCalcVers);
      for (i = 0; i < N; ++i)
	t->C[i][i] += summand;

    } /* for iEigenCalcVers */
  {
    char s[100];
    sprintf(s, " %d times no success.", iEigenCalcVers);
    WriteMaxErrorInfo(t);
    FATAL("cmaes_UpdateEigensystem_C () : Eigendecomposition:", s,0,0);
  }
} /* cmaes_UpdateEigensystem() */

/* ========================================================= */
/* 
   Calculating eigenvalues and vectors. 
   Input: 
     N: dimension.
     C: symmetric NxN-matrix.
     niter: number of maximal iterations for QL-Algorithm. 
     rgtmp: N+1-dimensional vector for temporal use. 
   Output: 
     diag: N eigenvalues.
     Q: Columns are normalized eigenvectors.
     return: number of iterations in QL-Algorithm.
   Q == C on input is possible and destroys C. 
 */
static int 
Eigen( int N,  double **C, double *diag, double **Q, 
       int niter, double *rgtmp)
{
  int ret;
  int i, j;

  if (niter == 0) niter = 30*N;

  if (C != Q)
  {
    for (i=0; i < N; ++i)
      {
	for (j = 0; j <= i; ++j)
	  Q[i][j] = Q[j][i] = C[i][j];
      }
  }
  
  if (rgtmp != NULL)
  {
    Householder( N, Q, diag, rgtmp);
    ret = QLalgo( N, diag, Q, niter, rgtmp+1);
  }
  else
  {
    rgtmp = new_double(N+1);
    Householder( N, Q, diag, rgtmp);
    ret = QLalgo( N, diag, Q, niter, rgtmp+1);
    free( rgtmp);
  }
  return ret;
}  

/* ========================================================= */
/* 
   Householder Transformation einer symmetrischen Matrix
   auf tridiagonale Form.
   -> n             : Dimension
   -> ma            : symmetrische nxn-Matrix
   <- ma            : Transformationsmatrix (ist orthogonal): 
                      Tridiag.-Matrix == <-ma * ->ma * (<-ma)^t
   <- diag          : Diagonale der resultierenden Tridiagonalmatrix
   <- neben[0..n-1] : Nebendiagonale (==1..n-1) der res. Tridiagonalmatrix

   */
static void 
Householder( int N, double **ma, double *diag, double *neben)
{
  double epsilon; 
  int i, j, k;
  double h, sum, tmp, tmp2;

  for (i = N-1; i > 0; --i)
  {
    h = 0.0;
    if (i == 1)
      neben[i] = ma[i][i-1];
    else
    {
      for (k = i-1, epsilon = 0.0; k >= 0; --k)
        epsilon += fabs(ma[i][k]);

      if (epsilon == 0.0)
        neben[i] = ma[i][i-1];
      else
      {
        for(k = i-1, sum = 0.0; k >= 0; --k)
	{ /* i-te Zeile von i-1 bis links normieren */
          ma[i][k] /= epsilon;
	  sum += ma[i][k]*ma[i][k];
	} 
	tmp = (ma[i][i-1] > 0) ? -sqrt(sum) : sqrt(sum);
	neben[i] = epsilon*tmp;
	h = sum - ma[i][i-1]*tmp;
	ma[i][i-1] -= tmp;
	for (j = 0, sum = 0.0; j < i; ++j)
	{
	  ma[j][i] = ma[i][j]/h;
	  tmp = 0.0;
	  for (k = j; k >= 0; --k)
	    tmp += ma[j][k]*ma[i][k];
	  for (k = j+1; k < i; ++k)
	    tmp += ma[k][j]*ma[i][k];
	  neben[j] = tmp/h;
	  sum += neben[j] * ma[i][j];
	} /* for j */
	sum /= 2.*h;
	for (j = 0; j < i; ++j)
	{
	  neben[j] -= ma[i][j]*sum;
	  tmp = ma[i][j];
	  tmp2 = neben[j];
	  for (k = j; k >= 0; --k)
	    ma[j][k] -= (tmp*neben[k] + tmp2*ma[i][k]);
	} /* for j */
      } /* else epsilon */
    } /* else i == 1 */
    diag[i] = h;
  } /* for i */

  diag[0] = 0.0;
  neben[0] = 0.0;
  
  for (i = 0; i < N; ++i)
  {
    if(diag[i] != 0.0)
      for (j = 0; j < i; ++j)
      {
	for (k = i-1, tmp = 0.0; k >= 0; --k)
	  tmp += ma[i][k] * ma[k][j];
        for (k = i-1; k >= 0; --k)
	  ma[k][j] -= tmp*ma[k][i];
      } /* for j   */
    diag[i] = ma[i][i];
    ma[i][i] = 1.0;
    for (k = i-1; k >= 0; --k)
      ma[k][i] = ma[i][k] = 0.0;
  } /* for i */
}

/*
  QL-Algorithmus mit implizitem Shift, zur Berechnung von Eigenwerten
  und -vektoren einer symmetrischen Tridiagonalmatrix. 
  -> n     : Dimension. 
  -> diag        : Diagonale der Tridiagonalmatrix. 
  -> neben[0..n-1] : Nebendiagonale (==0..n-2), n-1. Eintrag beliebig
  -> mq    : Matrix output von Householder() 
  -> maxIt : maximale Zahl der Iterationen 
  <- diag  : Eigenwerte
  <- neben : Garbage
  <- mq    : k-te Spalte ist normalisierter Eigenvektor zu diag[k]

  */

static int 
QLalgo( int N, double *diag, double **mq, 
	int maxIter, double *neben)
{
  int i, j, k, kp1, l;
  double tmp, diff, cneben, c1, c2, p;
  int iter;

  neben[N-1] = 0.0;
  for (i = 0, iter = 0; i < N && iter < maxIter; ++i)
    do /* while j != i */
    {
      for (j = i; j < N-1; ++j)
      {
	tmp = fabs(diag[j]) + fabs(diag[j+1]);
	if (fabs(neben[j]) + tmp == tmp)
	  break;
      }
      if (j != i)
      {
	if (++iter > maxIter) return maxIter-1;
	diff = (diag[i+1]-diag[i])/neben[i]/2.0;
	if (diff >= 0)
	  diff = diag[j] - diag[i] + 
	    neben[i] / (diff + sqrt(diff * diff + 1.0));
	else
	  diff = diag[j] - diag[i] + 
	    neben[i] / (diff - sqrt(diff * diff + 1.0));
	c2 = c1 = 1.0;
	p = 0.0;
	for (k = j-1; k >= i; --k)
	{
	  kp1 = k + 1;
	  tmp = c2 * neben[k];
	  cneben = c1 * neben[k];
	  if (fabs(tmp) >= fabs(diff))
	  {
	    c1 = diff / tmp;
	    c2 = 1. / sqrt(c1*c1 + 1.0);
	    neben[kp1] = tmp / c2;
	    c1 *= c2;
	  }
	  else
	  {
	    c2 = tmp / diff;
	    c1 = 1. / sqrt(c2*c2 + 1.0);
	    neben[kp1] = diff / c1;
	    c2 *= c1;
	  } /* else */
	  tmp = (diag[k] - diag[kp1] + p) * c2 + 2.0 * c1 * cneben;
	  diag[kp1] += tmp * c2 - p;
	  p = tmp * c2;
	  diff = tmp * c1 - cneben;
	  for (l = N-1; l >= 0; --l) /* TF-Matrix Q */
	  { 
	    tmp = mq[l][kp1];
	    mq[l][kp1] = c2 * mq[l][k] + c1 * tmp;
	    mq[l][k] = c1 * mq[l][k] - c2 * tmp;
	  } /* for l */
	} /* for k */
	diag[i] -= p;
	neben[i] = diff;
	neben[j] = 0.0;
      } /* if */
    } while (j != i);
  return iter;
} /* QLalgo() */

/* ========================================================= */
static void
WriteMaxErrorInfo(cmaes_t *t)
{
  int i,j, N=t->sp.N; 
  char *s = (char *)new_void(200+30*(N+2), sizeof(char)); s[0] = '\0';
  
  sprintf( s+strlen(s),"\nKomplett-Info\n");
  sprintf( s+strlen(s)," Gen       %20.12g\n", t->gen);
  sprintf( s+strlen(s)," Dimension %d\n", N);
  sprintf( s+strlen(s)," sigma     %e\n", t->sigma);
  sprintf( s+strlen(s)," lastminEW %e\n", 
	   t->dLastMinEWgroesserNull);
  sprintf( s+strlen(s)," maxKond   %e\n\n", t->dMaxSignifKond);
  sprintf( s+strlen(s),"     x-Vektor          rgD     Basis...\n");
  ERRORMESSAGE( s,0,0,0);
  s[0] = '\0';
  for (i = 0; i < N; ++i)
    {
      sprintf( s+strlen(s), " %20.12e", t->rgxmean[i]);
      sprintf( s+strlen(s), " %10.4e", t->rgD[i]);
      for (j = 0; j < N; ++j)
	sprintf( s+strlen(s), " %10.2e", t->B[i][j]);
      ERRORMESSAGE( s,0,0,0);
      s[0] = '\0';
    }
  ERRORMESSAGE( "\n",0,0,0);
  free( s);
} /* WriteMaxErrorInfo() */

/* --------------------------------------------------------- */
/* ---------------- Functions: random_t -------------------- */
/* --------------------------------------------------------- */
/* --------------------------------------------------------- */
/* X_1 exakt :          0.79788456)  */
/* chi_eins simuliert : 0.798xx   (seed -3) */
/*                    +-0.001 */
/* --------------------------------------------------------- */
/* 
   Gauss() liefert normalverteilte Zufallszahlen
   bei vorgegebenem seed.
*/
/* --------------------------------------------------------- */
/* --------------------------------------------------------- */

void 
random_init( random_t *t, long int inseed)
{
  clock_t cloc = clock();

  t->flgstored = 0;
  t->rgrand = (long *) new_void(32, sizeof(long));
  if (inseed < 1) {
    while ((long) (cloc - clock()) == 0)
      ;
    inseed = (long)abs(100*time(NULL)+clock());
  }
  random_Start(t, inseed);
}

void
random_exit(random_t *t)
{
  free( t->rgrand);
}

/* --------------------------------------------------------- */
void random_Start( random_t *t, long int inseed)
{
  long tmp;
  int i;

  t->flgstored = 0;
  t->startseed = inseed;
  if (inseed < 1)
    inseed = 1; 
  t->aktseed = inseed;
  for (i = 39; i >= 0; --i)
  {
    tmp = t->aktseed/127773;
    t->aktseed = 16807 * (t->aktseed - tmp * 127773)
      - 2836 * tmp;
    if (t->aktseed < 0) t->aktseed += 2147483647;
    if (i < 32)
      t->rgrand[i] = t->aktseed;
  }
  t->aktrand = t->rgrand[0];
}

/* --------------------------------------------------------- */
double random_Gauss(random_t *t)
{
  double x1, x2, rquad, fac;

  if (t->flgstored)
  {    
    t->flgstored = 0;
    return t->hold;
  }
  do 
  {
    x1 = 2.0 * random_Uniform(t) - 1.0;
    x2 = 2.0 * random_Uniform(t) - 1.0;
    rquad = x1*x1 + x2*x2;
  } while(rquad >= 1 || rquad <= 0);
  fac = sqrt(-2.0*log(rquad)/rquad);
  t->flgstored = 1;
  t->hold = fac * x1;
  return fac * x2;
}

/* --------------------------------------------------------- */
double random_Uniform( random_t *t)
{
  long tmp;

  tmp = t->aktseed/127773;
  t->aktseed = 16807 * (t->aktseed - tmp * 127773)
    - 2836 * tmp;
  if (t->aktseed < 0) t->aktseed += 2147483647;
  tmp = t->aktrand / 67108865;
  t->aktrand = t->rgrand[tmp];
  t->rgrand[tmp] = t->aktseed;
  return (double)(t->aktrand)/(2.147483647e9);
}

static char *
szCat(const char *sz1, const char*sz2, 
      const char *sz3, const char *sz4);

/* --------------------------------------------------------- */
/* -------------- Functions: readpara_t -------------------- */
/* --------------------------------------------------------- */
void
readpara_init (readpara_t *t,
	       int dim, 
	       int inseed, 
	       const double * inxstart, 
	       const double * inrgcoordist,
	       int lambda, //MT
	       int mu,
	       const char * filename)
{
  int i, N;
  t->rgsformat = (char **) new_void(25, sizeof(char *));
  t->rgpadr = (void **) new_void(25, sizeof(void *)); 
  t->rgskeyar = (char **) new_void(5, sizeof(char *));
  t->rgp2adr = (double ***) new_void(10, sizeof(double **));
  t->weigkey = (char *)new_void(7, sizeof(char)); 

  /* All scalars:  */
  t->rgsformat[0] = (char*)" N %d";        t->rgpadr[0] = (void *) &t->N;
  t->rgsformat[1] = (char*)" seed %d";    t->rgpadr[1] = (void *) &t->seed;
  t->rgsformat[2] = (char*)" maxeval %lg"; t->rgpadr[2] = (void *) &t->maxeval;
  t->rgsformat[3] = (char*)" stopfunval %lg"; t->rgpadr[3]=(void *) &t->stStopFuncValue.val;
  t->rgsformat[4] = (char*)" stopfunvaldiff %lg"; t->rgpadr[4]=(void *) &t->stopFuncValDiff;
  t->rgsformat[5] = (char*)" lambda %d";      t->rgpadr[5] = (void *) &t->lambda;
  t->rgsformat[6] = (char*)" mu %d";          t->rgpadr[6] = (void *) &t->mu;
  t->rgsformat[7] = (char*)" weights %5s";    t->rgpadr[7] = (void *) t->weigkey;
  t->rgsformat[8] = (char*)" fac*ccumsig %lg";t->rgpadr[8] = (void *) &t->ccumsig;
  t->rgsformat[9] = (char*)" fac*damp %lg";   t->rgpadr[9] = (void *) &t->damp;
  t->rgsformat[10] = (char*)" ccumcov %lg";    t->rgpadr[10] = (void *) &t->ccumcov;
  t->rgsformat[11] = (char*)" mucov %lg";     t->rgpadr[11] = (void *) &t->mucov;
  t->rgsformat[12] = (char*)" fac*ccov %lg";  t->rgpadr[12]=(void *) &t->ccov;
  t->rgsformat[13] = (char*)" updatecov %lg"; t->rgpadr[13]=(void *) &t->updateCmode.modulo;
  t->rgsformat[14] = (char*)" updatecovmaxtime %lg"; t->rgpadr[14]=(void *) &t->updateCmode.maxtime;
  t->rgsformat[15] = (char*)" resume %59s";    t->rgpadr[15] = (void *) t->resumefile;
  t->rgsformat[16] = (char*)" fac*maxeval %lg";   t->rgpadr[16] = (void *) &t->facmaxeval;
  t->rgsformat[17] = (char*)" fac*updatecov %lg"; t->rgpadr[17]=(void *) &t->facupdateCmode;
  t->n1para = 18; 
  t->n1outpara = 16; /* disregard last parameters in WriteToFile() */

  /* arrays */
  t->rgskeyar[0]  = (char*)" xstart %d";   t->rgp2adr[0] = &t->xstart;
  t->rgskeyar[1]  = (char*)" sigma %d"; t->rgp2adr[1] = &t->rgcoordist;
  t->rgskeyar[2]  = (char*)" mincoorstddev %d"; t->rgp2adr[2] = &t->rgMinCoorStddev;
  t->n2para = 3;  

  t->N = dim;
  t->seed = inseed; 
  t->xstart = NULL; 
  t->rgcoordist = NULL; 
  t->rgMinCoorStddev = NULL; 
  t->maxeval = -1;
  t->facmaxeval = 1; 
  t->stStopFuncValue.flg = -1;
  t->stopFuncValDiff = -1; /* never stop on funvaldiff*/
  
  t->lambda = lambda; //MT
  t->mu = mu; //MT
  t->mucov = -1;
  t->weights = NULL;
  strcpy(t->weigkey, "log");

  t->ccumsig = -1;
  t->ccumcov = -1;
  t->damp = -1;
  t->ccov = -1;

  t->updateCmode.modulo = -1;  
  t->updateCmode.maxtime = -1;
  t->updateCmode.flgalways = 0;
  t->facupdateCmode = 1;
  strcpy(t->resumefile, "_no_");

  readpara_ReadFromFile(t, filename);
  N = t->N; 
  if (N == 0)
    FATAL("readpara_readpara_t(): problem dimension N undefined.\n",
	  "  (no default value available).",0,0); 
  if (t->xstart == NULL && inxstart == NULL) {
    ERRORMESSAGE("Warning: xstart undefined. 0.5...0.5 used.","","","");
    printf("\nWarning: xstart undefined. 0.5...0.5 used.\n");
  }
  if (t->rgcoordist == NULL && inrgcoordist == NULL) {
    ERRORMESSAGE("Warning: sigma undefined. 0.3...0.3 used.","","","");
    printf("\nWarning: sigma undefined. 0.3...0.3 used.\n");
  }

  if (t->xstart == NULL) {
    t->xstart = new_double(N);
    for (i=0; i<N; ++i)
      t->xstart[i] = (inxstart == NULL) ? 0.5 : inxstart[i];
  }
  if (t->rgcoordist == NULL) {
    t->rgcoordist = new_double(N);
    for (i=0; i<N; ++i)
      t->rgcoordist[i] = (inrgcoordist == NULL) ? 0.3 : inrgcoordist[i];
  }
  readpara_SupplementDefaults(t);
  readpara_WriteToFile(t, "z.actparcmaes.par", filename);
} /* readpara_init */
/* --------------------------------------------------------- */
/* --------------------------------------------------------- */
void readpara_exit(readpara_t *t)
{
  if (t->xstart != NULL)
    free( t->xstart);
  if (t->rgcoordist != NULL)
    free( t->rgcoordist);
  if (t->rgMinCoorStddev != NULL)
    free( t->rgMinCoorStddev);
  if (t->weights != NULL)
    free( t->weights);

  free(t->rgsformat);
  free(t->rgpadr);
  free(t->rgskeyar);
  free(t->rgp2adr);
  free(t->weigkey);
}

/* --------------------------------------------------------- */
/* --------------------------------------------------------- */
void 
readpara_ReadFromFile(readpara_t *t, const char * filename)
{
  char s[1000];
  const char *ss = "incmaes.par";
  int ipara, i;
  int size;
  FILE *fp;
  if (filename == NULL)
    filename = ss;
  fp = fopen( filename, "r"); 
  if(fp == NULL) {
    ERRORMESSAGE("cmaes_ReadFromFile(): could not open '", filename, "'",0);
    return;
  }
  for (ipara=0; ipara < t->n1para; ++ipara)
    {
      rewind(fp);
      while(fgets(s, sizeof(s), fp) != NULL)
	{ /* skip comments  */
	  if (s[0] == '#' || s[0] == '%')
	    continue;
	  if(sscanf(s, t->rgsformat[ipara], t->rgpadr[ipara]) == 1) {
	    if (strncmp(t->rgsformat[ipara], " stopfunval ", 12) == 0)
	      t->stStopFuncValue.flg = 1;
	    break;
	  }
	}
    } /* for */
  if (t->N <= 0)
    FATAL("readpara_ReadFromFile(): No valid dimension N",0,0,0); 
  for (ipara=0; ipara < t->n2para; ++ipara)
    {
      rewind(fp);
      while(fgets(s, sizeof(s), fp) != NULL)
	{ /* skip comments  */
	  if (s[0] == '#' || s[0] == '%')
	    continue;
	  if(sscanf(s, t->rgskeyar[ipara], &size) == 1) {
	    if (size > 0) {
	      *t->rgp2adr[ipara] = new_double(t->N);
	      for (i=0;i<size&&i<t->N;++i)
		if (fscanf(fp, " %lf", &(*t->rgp2adr[ipara])[i]) != 1)
		  break;
	      if (i<size && i < t->N) {
		ERRORMESSAGE("readpara_ReadFromFile ", filename, ": ",0); 
		FATAL( "'", t->rgskeyar[ipara], 
		       "' not enough values found.\n", 
		       "   Remove all comments between numbers.");
	      }
	      for (; i < t->N; ++i) /* recycle */
		(*t->rgp2adr[ipara])[i] = (*t->rgp2adr[ipara])[i%size];
	    }
	  }
	}  
    } /* for */
  fclose(fp);
  return;
} /* readpara_ReadFromFile() */

/* --------------------------------------------------------- */
/* --------------------------------------------------------- */
void
readpara_WriteToFile(readpara_t *t, const char *filenamedest, 
		     const char *filenamesource)
{
  int ipara, i; 
  size_t len;
  time_t ti = time(NULL);
  FILE *fp = fopen( filenamedest, "a"); 
  if(fp == NULL) {
    ERRORMESSAGE("cmaes_WriteToFile(): could not open '", 
		 filenamedest, "'",0);
    return;
  }
  fprintf(fp, "\n# Read from %s at %s\n", filenamesource, 
	  asctime(localtime(&ti)));
  for (ipara=0; ipara < 1; ++ipara) {
    fprintf(fp, t->rgsformat[ipara], *(int *)t->rgpadr[ipara]);
    fprintf(fp, "\n");
  }
  for (ipara=0; ipara < t->n2para; ++ipara) {
    if(*t->rgp2adr[ipara] == NULL)
      continue;
    fprintf(fp, t->rgskeyar[ipara], t->N);
    fprintf(fp, "\n");
    for (i=0; i<t->N; ++i)
      fprintf(fp, "%7.3g%c", (*t->rgp2adr[ipara])[i], (i%5==4)?'\n':' ');
    fprintf(fp, "\n");
  }
  for (ipara=1; ipara < t->n1outpara; ++ipara) {
    if (strncmp(t->rgsformat[ipara], " stopfunval ", 12) == 0)
      if(t->stStopFuncValue.flg == 0) {
	fprintf(fp, " stopfunval\n");
	continue;
      }
    len = strlen(t->rgsformat[ipara]);
    if (t->rgsformat[ipara][len-1] == 'd') /* read integer */
      fprintf(fp, t->rgsformat[ipara], *(int *)t->rgpadr[ipara]);
    else if (t->rgsformat[ipara][len-1] == 's') /* read string */
      fprintf(fp, t->rgsformat[ipara], (char *)t->rgpadr[ipara]);
    else { 
      if (strncmp(" fac*", t->rgsformat[ipara], 5) == 0) {
	fprintf(fp, " ");
	fprintf(fp, t->rgsformat[ipara]+5, *(double *)t->rgpadr[ipara]);
      } else
	fprintf(fp, t->rgsformat[ipara], *(double *)t->rgpadr[ipara]);
    }
    fprintf(fp, "\n");
  } /* for */
  fprintf(fp, "\n");
  fclose(fp); 
} /* readpara_WriteToFile() */

/* --------------------------------------------------------- */
/* --------------------------------------------------------- */
void 
readpara_SupplementDefaults(readpara_t *t)
{
  double t1, t2, maxgen;
  int N = t->N; 
  clock_t cloc = clock();
  
  if (t->seed < 1) {
    while ((int) (cloc - clock()) == 0)
      ;
    t->seed = (unsigned int)abs(100*time(NULL)+clock());
  }

  if (t->stStopFuncValue.flg == -1)
    t->stStopFuncValue.flg = 0;

  if (t->lambda < 2)
    t->lambda = 4+(int)(3*log(N));
  if (t->mu == -1) {
    t->mu = (int)floor(t->lambda/2); //MT
    readpara_SetWeights(t, t->weigkey);
  }
  if (t->weights == NULL)
    readpara_SetWeights(t, t->weigkey);

  if (t->ccumsig > 0) /* factor was read */
    t->ccumsig *= (t->mueff + 2.) / (N + t->mueff + 3.);
  if (t->ccumsig <= 0 || t->ccumsig >= 1)
    t->ccumsig = (t->mueff + 2.) / (N + t->mueff + 3.);

  if (t->ccumcov <= 0 || t->ccumcov > 1)
    t->ccumcov = 4. / (N + 4);
  
  if (t->mucov < 1) {
    t->mucov = t->mueff;
  }
  t1 = 2. / ((N+1.4142)*(N+1.4142));
  t2 = (2.*t->mueff-1.) / ((N+2.)*(N+2.)+t->mueff);
  t2 = (t2 > 1) ? 1 : t2;
  t2 = (1./t->mucov) * t1 + (1.-1./t->mucov) * t2;
  if (t->ccov >= 0) /* ccov holds the read factor */
    t->ccov *= t2;
  if (t->ccov < 0 || t->ccov > 1) /* set default in case */
    t->ccov = t2;
  
  if (t->maxeval == -1)  /* may depend on ccov in near future */
    t->maxeval = t->facmaxeval*900*(N+3)*(N+3); 
  else
    t->maxeval *= t->facmaxeval;

  maxgen = (double)t->maxeval / t->lambda; 
  if (t->damp > 0) /* factor read */
    t->damp = 1 + t->damp*douMax(0.3, (1.-(double)N/maxgen)) 
      * (1+2*douMax(0,sqrt((t->mueff-1.)/(N+1.))-1)) /* limit sigma increase */
      / t->ccumsig;

  if (t->damp < 1) /* set default */
    t->damp = 1 + douMax(0.3,(1.-(double)N/maxgen)) 
      * (1+2*douMax(0,sqrt((t->mueff-1.)/(N+1.))-1)) /* limit sigma increase */
      / t->ccumsig;

  if (t->updateCmode.modulo < 1)
    t->updateCmode.modulo = 1./t->ccov/(double)(N)/10.;
  t->updateCmode.modulo *= t->facupdateCmode;
  if (t->updateCmode.maxtime < 0)
    t->updateCmode.maxtime = 20; /* maximal 20% of CPU-time */

} /* readpara_SupplementDefaults() */

   
/* --------------------------------------------------------- */
/* --------------------------------------------------------- */
void 
readpara_SetWeights(readpara_t *t, const char * mode)
{
  double s1, s2;
  int i;

  if(t->weights != NULL)
    free( t->weights); 
  t->weights = new_double(t->mu);
  if (strcmp(mode, "lin") == 0)
    for (i=0; i<t->mu; ++i) 
      t->weights[i] = t->mu - i;
  else if (strncmp(mode, "equal", 3) == 0)
    for (i=0; i<t->mu; ++i) 
      t->weights[i] = 1;
  else if (strcmp(mode, "log") == 0) 
    for (i=0; i<t->mu; ++i) 
      t->weights[i] = log(t->mu+1.)-log(i+1); 
  else
    for (i=0; i<t->mu; ++i) 
      t->weights[i] = log(t->mu+1.)-log(i+1); 

  /* normalize weights vector and set mueff */
  for (i=0, s1=0, s2=0; i<t->mu; ++i) {
    s1 += t->weights[i];
    s2 += t->weights[i]*t->weights[i];
  }
  t->mueff = s1*s1/s2;
  for (i=0; i<t->mu; ++i) 
    t->weights[i] /= s1;

  if(t->mu < 1 || t->mu > t->lambda || 
     (t->mu==t->lambda && t->weights[0]==t->weights[t->mu-1]))
    FATAL("readpara_SetWeights(): invalid setting of mu or lambda",0,0,0);

} /* readpara_SetWeights() */

/* --------------------------------------------------------- */
/* --------------------------------------------------------- */
static int 
intMin( int i, int j)
{
  return i < j ? i : j;
}
static double
douMax( double i, double j)
{
  return i > j ? i : j;
}
static double
douMin( double i, double j)
{
  return i < j ? i : j;
}
static double
rgdouMax( const double *rgd, int len)
{
  int i;
  double max = rgd[0];
  for (i = 1; i < len; ++i)
    max = (max < rgd[i]) ? rgd[i] : max;
  return max;
}

static double
rgdouMin( const double *rgd, int len)
{
  int i;
  double min = rgd[0];
  for (i = 1; i < len; ++i)
    min = (min > rgd[i]) ? rgd[i] : min;
  return min;
}

static int    
MaxIdx( const double *rgd, int len)
{
  int i, res;
  for(i=1, res=0; i<len; ++i)
    if(rgd[i] > rgd[res])
      res = i;
  return res;
}
static int    
MinIdx( const double *rgd, int len)
{
  int i, res;
  for(i=1, res=0; i<len; ++i)
    if(rgd[i] < rgd[res])
      res = i;
  return res;
}


static int SignOfDiff(const void *d1, const void * d2) 
{ 
  return *((double *) d1) > *((double *) d2) ? 1 : -1; 
} 

#if 1
/* dirty index sort */
static void Sorted_index(const double *rgFunVal, int *index, int n)
{
  int i, j;
  for (i=1, index[0]=0; i<n; ++i) {
    for (j=i; j>0; --j) {
      if (rgFunVal[index[j-1]] < rgFunVal[i])
	break;
      index[j] = index[j-1]; 
    }
    index[j] = i;
  }
}
#endif 

static void * new_void(size_t n, size_t size)
{
  static char s[70];
  void *p = calloc(n, size);
  if (p == NULL) {
    sprintf(s, "new_void(): calloc(%ld,%ld) failed",(long)n,(long)size);
    FATAL(s,0,0,0);
  }
  return p;
}
static double * new_double(int n)
{
  static char s[70];
  double *p = (double *) calloc(n, sizeof(double));
  if (p == NULL) {
    sprintf(s, "new_double(): calloc(%ld,%ld) failed",
	    (long)n,(long)sizeof(double));
    FATAL(s,0,0,0);
  }
  return p;
}

/* --------------------------------------------------------- */
/* --------------------------------------------------------- */

/* ========================================================= */
static void 
FATAL(char const *s1, char const *s2, char const *s3, 
      char const *s4)
{
  ERRORMESSAGE( s1, s2, s3, s4);
  ERRORMESSAGE("*** Exiting cmaes_t ***",0,0,0);
  printf("\n%s\n", s2 ? szCat(s1, s2, s3, s4) : s1);
  printf("\n *** Confirm EXIT *** ");
  getc(stdin);
  exit(1);
}

/* ========================================================= */
void ERRORMESSAGE( char const *s1, char const *s2, 
		   char const *s3, char const *s4)
{
#if 1
  /*  static char szBuf[700];  desirable but needs additional input argument 
      sprintf(szBuf, "%f:%f", gen, gen*lambda);
  */
  time_t t = time(NULL);
  FILE *fp = fopen( "z.errcmaes.err", "a");
  if (!fp)
    {
      printf("\n%s\n", s2 ? szCat(s1, s2, s3, s4) : s1);
      printf("cmaes_t could not open file 'errcmaes.err'.");
      printf("\n *** Confirm EXIT *** ");
      getc(stdin);
      exit(1);
    }
  fprintf( fp, "\n -- %s %s\n", asctime(localtime(&t)), 
	   s2 ? szCat(s1, s2, s3, s4) : s1);
  fclose (fp);
#endif
}

/* ========================================================= */
char *szCat(const char *sz1, const char*sz2, 
	    const char *sz3, const char *sz4)
{
  static char szBuf[700];

  if (!sz1)
    FATAL("szCat() : Invalid Arguments",0,0,0);

  strncpy ((char *)szBuf, sz1, intMin( strlen(sz1), 698));
  szBuf[intMin( strlen(sz1), 698)] = '\0';
  if (sz2)
    strncat ((char *)szBuf, sz2, 
	     intMin((int)strlen(sz2)+1, 698 - (int)strlen((char const *)szBuf)));
  if (sz3)
    strncat((char *)szBuf, sz3, 
	    intMin((int)strlen(sz3)+1, 698 - (int)strlen((char const *)szBuf)));
  if (sz4)
    strncat((char *)szBuf, sz4, 
	    intMin((int)strlen(sz4)+1, 698 - strlen((char const *)szBuf)));
  return (char *) szBuf;
}
