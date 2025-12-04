/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/util.h"
#include "../Core/array.h"

//===========================================================================

struct Gaussian;
namespace rai {
template<class T> struct Array;
}

//typedef struct arr;
typedef unsigned int uint;
typedef rai::Array<byte> byteA;
typedef rai::Array<uint> uintA;
typedef rai::Array<Gaussian> GaussianA;
typedef rai::Array<Gaussian*> GaussianL;

//===========================================================================

typedef enum { opengl, xfig, gnupl } PlotMode;

namespace rai {

struct PlotModule {
  std::unique_ptr<struct sPlotModule> self;
  PlotMode mode;
  bool light, grid, colors, drawBox, drawDots, perspective;
  uint thickLines;//display options
  PlotModule();
  ~PlotModule();

  void Gnuplot();
  void Opengl(bool perspective=false, double xl=-1., double xh=1., double yl=-1., double yh=1., double zl=-1., double zh=1.);
  void update(bool wait=false, const char* txt=0);
  void Close();
  void writeGnuplotFiles();

  void Clear();
  void Function(const arr& f, double x0=0., double x1=0.);
  void FunctionPoints(const arr& x, const arr& f);
  void Functions(const arr& f, double x0=0., double x1=0.);
  void Function(const arr& x, const arr& f);
  void FunctionPrecision(const arr& x, const arr& f, const arr& h, const arr& l);
  void Surface(const arr& X);
  void Array(const arr& X);
  void Point(double x, double y, double z);
  void Point(const arr& x);
  void Points(const arr& X);
  void ClearPoints();
  void Line(const arr& X, bool closed=false);
  void Points(const arr& X, const arr& Y);
  void Covariance(const arr& mean, const arr& cov);
  void VectorField(const arr& X, const arr& dX);
  void VectorField(arr& dX);
  void MatrixFlow(uintA& M, double len);
  void Gaussians(const GaussianA& G);
  void Gaussians(const GaussianL& G);
  void Image(const byteA& x);
};

struct Gnuplot {
  str cmd;
  uint data_count=0;
  arr B;


  void plot(const arr& bounds={});
  void splot(const arr& bounds, bool contour=false);

  void function(const arr& X, const arr& f, const char* style="");
  void functionConfidence(const arr& X, const arr& f, const arr& low, const arr& up, const char* style="");
  void heightField(const arr& Z, const char* style="");
  void points(const arr& X, const char* style="");

  void show();

private:
  str dump(const arr& X);
};

}

extern rai::Singleton<rai::PlotModule> plot;

//===========================================================================
