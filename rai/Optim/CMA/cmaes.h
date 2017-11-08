/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#ifndef NH_cmaes_h //MT
#define NH_cmaes_h //MT
/* --------------------------------------------------------- */
/* --- File: cmaes.h ----------- Author: Nikolaus Hansen --- */
/* ---------------------- last modified: III 2003        --- */
/* --------------------------------- by: Nikolaus Hansen --- */
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

#include <time.h>

typedef struct /* random_t */
{
  /* Variables for Uniform() */
  long int startseed;
  long int aktseed;
  long int aktrand;
  long int *rgrand;
  
  /* Variables for Gauss() */
  short flgstored;
  double hold;
} random_t;

typedef struct /* readpara_t */
{
  /* External parameter */
  int N; /* problem dimension, must stay constant */
  unsigned int seed; 
  double * xstart; 
  double * rgcoordist;
  double * rgMinCoorStddev; 
  double maxeval; 
  double facmaxeval;
  struct { int flg; double val; } stStopFuncValue; 
  double stopFuncValDiff;

  /* Internal evolution strategy parameter */
  int lambda;          /* -> mu */
  int mu;              /* -> weights, lambda */
  double mucov, mueff; /* <- weights */
  double *weights;     /* <- mu, -> mueff -> mucov -> ccov */
  double damp;         /* <- ccumsig, maxeval, lambda */
  double ccumsig;      /* -> damp, <- N */
  double ccumcov; 
  double ccov;         /* <- mucov, N */
  struct { int flgalways; double modulo; double maxtime; } updateCmode;
  double facupdateCmode;

  /* supplementary variables */

  char *weigkey; 
  char resumefile[99];
  char **rgsformat;
  void **rgpadr;
  char **rgskeyar;
  double ***rgp2adr;
  int n1para, n1outpara;
  int n2para;
} readpara_t;

typedef struct /* cmaes_t */
{
  double(*pFitFunc)(double *); /* pointer to objective function */
  double sigma; /* step size */

  double *rgxmean;  /* mean x vector, parent */
  double *rgxbestever; 
  double **rgrgx;   /* range of x-vectors, lambda offspring */
  int *index;       /* sorting index of sample pop. */
  double arFuncValueHist[4];

  short flgIniphase; 
  short flgStop; 
  double stopeval; 
  double chiN; 
  double **C;  /* lower triangular matrix: i>=j fuer C[i][j] */
  double **B;  /* matrix with normalize eigenvectors in columns */
  double *rgD; /* axis lengths */

  double *rgpc;
  double *rgps;
  double *rgxold; 
  double *rgout; 
  double *rgBDz;   /* for B*D*z */
  double *rgdTmp;  /* temporary (random) vector used in different places */
  double *rgFuncValue; 
  struct { int flgalways; double modulo; double maxtime;} updateCmode;

  double gen; /* Generation number */
  double state; 

  int cErrors; 
  int maxErr;

  char *sOutString; /* 4x80 */
  short flgEigensysIsUptodate;
  double genOfEigensysUpdate; 
  clock_t clockeigensum;

  double dMaxSignifKond; 				     
  double dLastMinEWgroesserNull;

  readpara_t sp;
  random_t rand;
} cmaes_t; 


#endif //MT
