/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef RAI_plot_h
#define RAI_plot_h
#include <Core/util.h>

//===========================================================================

struct OpenGL;
struct Gaussian;
namespace rai {  template<class T> struct Array;  }

typedef unsigned int uint;
typedef rai::Array<double> arr;
typedef rai::Array<uint> uintA;
typedef rai::Array<Gaussian> GaussianA;
typedef rai::Array<Gaussian*> GaussianL;

//===========================================================================

typedef enum { opengl, xfig, gnupl } PlotMode;

struct PlotModule {
  struct sPlotModule *s;
  PlotMode mode;
  OpenGL *gl;
  bool light, grid, colors, drawBox, drawDots, perspective;
  uint thickLines;//display options
  PlotModule();
  ~PlotModule();
};
extern PlotModule plotModule;

//===========================================================================

void plotGnuplot();
void plotOpengl();
void plotOpengl(bool perspective, double xl=-1., double xh=1., double yl=-1., double yh=1., double zl=-1., double zh=1.);
void plot(bool wait=true, const char* txt=0);
void plotClose();

void plotClear();
void plotFunction(const arr& f, double x0=0., double x1=0.);
void plotFunctionPoints(const arr& x, const arr& f);
void plotFunctions(const arr& f, double x0=0., double x1=0.);
void plotFunction(const arr& x, const arr& f);
void plotFunctionPrecision(const arr& x, const arr& f, const arr& h, const arr& l);
void plotSurface(const arr& X);
void plotArray(const arr& X);
void plotPoint(double x, double y, double z);
void plotPoint(const arr& x);
void plotPoints(const arr& X);
void plotClearPoints();
void plotLine(const arr& X, bool closed=false);
void plotPoints(const arr& X, const arr& Y);
void writeGnuplotFiles();
void plotCovariance(const arr& mean, const arr& cov);
void plotVectorField(const arr& X, const arr& dX);
void plotVectorField(arr& dX);
void plotMatrixFlow(uintA& M, double len);
void plotGaussians(const GaussianA& G);
void plotGaussians(const GaussianL& G);

void glDrawPlot(void *module, OpenGL&);

#endif

