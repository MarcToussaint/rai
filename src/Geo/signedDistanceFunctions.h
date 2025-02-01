/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "geo.h"
#include "mesh.h"

//===========================================================================
//
// analytic distance functions
//

struct SDF : ScalarFunction {
  SDF(const rai::Transformation& _pose)
    : ScalarFunction(std::bind(&SDF::f, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)),
      pose(_pose) {}
  ~SDF() {}
  rai::Transformation pose;
  arr lo, up;
  virtual double f(arr& g, arr& H, const arr& x);
  virtual double f_raw(arr& g, arr& H, const arr& x) { NIY; }

  arr eval(const arr& samples);
  floatA evalFloat(const arr& samples);
  floatA evalGrid(uint d0, int d1=-1, int d2=-1);

  void viewSlice(OpenGL& gl, double z, const arr& lo, const arr& hi);
  void animateSlices(const arr& lo, const arr& hi, double wait=0.);
  void view(double wait=0.);

  arr projectNewton(const arr& x0, double stepMax=.1, double regularization=1e-1);

  virtual void write(std::ostream& os) const { NIY; }
  virtual void read(std::istream& is) { NIY; }
};

struct SDF_Sphere : SDF {
  double r;
  SDF_Sphere(const rai::Transformation& _pose, double _r)
    : SDF(_pose), r(_r) {}
  double f(arr& g, arr& H, const arr& x);
};

struct SDF_ssBox : SDF {
  arr size;
  double r;
  SDF_ssBox(const rai::Transformation& _pose, const arr& _size, double _r=0.)
    : SDF(_pose), size(_size), r(_r) { if(size.N==4) { r=size(3); size.resizeCopy(3); } }
  double f(arr& g, arr& H, const arr& x);
};

struct SDF_SuperQuadric : SDF {
  arr size;
  double degree;
  SDF_SuperQuadric(const rai::Transformation& _pose, const arr& _size, double _degree=5.)
    : SDF(_pose), size(_size), degree(_degree) {}
  double f(arr& g, arr& H, const arr& x);
};

struct SDF_ssSomething : SDF {
  std::shared_ptr<SDF> something;
  double r;
  SDF_ssSomething(const std::shared_ptr<SDF>& _something, double _r)
    : SDF(0), something(_something), r(_r) {}
  double f(arr& g, arr& H, const arr& x);
};

struct SDF_Cylinder : SDF {
  double size_z, r;
  SDF_Cylinder(const rai::Transformation& _pose, double _size_z, double _r)
    : SDF(_pose), size_z(_size_z), r(_r) {}
  double f(arr& g, arr& H, const arr& x);
};

struct SDF_Capsule : SDF {
  double size_z, r;
  SDF_Capsule(const rai::Transformation& _pose, double _size_z, double _r)
    : SDF(_pose), size_z(_size_z), r(_r) {}
  double f(arr& g, arr& H, const arr& x);
};

struct SDF_Blobby : SDF {
  SDF_Blobby() : SDF(0) {}
  double f_raw(arr& g, arr& H, const arr& _x) {
    double x=_x(0), y=_x(1), z=_x(2);
    return x*x*x*x - 5*x*x+ y*y*y*y - 5*y*y + z*z*z*z - 5*z*z + 11.8;
  }
};

struct SDF_Torus : SDF {
  double r1, r2;
  SDF_Torus(double _r1=1., double _r2=.1);
  double f_raw(arr& g, arr& H, const arr& _x);
};

struct DensityDisplayData {
  rai::Mesh box;
  byteA volumeImgZ, volumeImgY, volumeImgX;
  rai::Array<rai::Mesh> volumeZ, volumeY, volumeX;

  DensityDisplayData(struct TensorShape& sdf);
};

struct TensorShape : SDF {
  floatA gridData;
  shared_ptr<DensityDisplayData> _densityDisplayData;

  TensorShape(const rai::Transformation& _pose, const floatA& _data, const arr& _lo, const arr& _up)
    : SDF(_pose), gridData(_data) {  lo = _lo;  up = _up;  }
  TensorShape(uint N, const arr& _lo, const arr& _up, bool isoGrid=true);
  TensorShape(SDF& f, const arr& _lo, const arr& _up, const uintA& res);
  TensorShape() : SDF(0) {}
  TensorShape(istream& is) : SDF(0) { read(is); }

  double f(arr& g, arr& H, const arr& x);

  //manipulations
  void resample(uint d0, int d1=-1, int d2=-1);

  void smooth(uint width=3, uint iters=2);

  //helper
  void getNeighborsAndWeights(uintA& neigh, arr& weights, const arr& x_rel);
  arr getGridPosition(const uintA& idx) {
    arr res = (up-lo) / arr{(double)gridData.d0-1, (double)gridData.d1-1, (double)gridData.d2-1};
    arr x(3);
    for(uint i=0; i<3; i++) x(i) = lo(i) + idx(i)*res(i);
    return x;
  }

  //IO
  void write(std::ostream& os) const;
  void read(std::istream& is);
};
stdPipes(TensorShape)

//===========================================================================

struct PCL2Field {
  TensorShape& field;
  floatA source;
  double alpha = .1;
  double lastErr = -1.;

  PCL2Field(TensorShape& _field):field(_field) {}

  double stepDiffusion(const arr& pts, const arr& values, double boundValue);

  double runDiffusion(const arr& pts, const arr& values, uint iters=30, double boundValue=0.);
};

//===========================================================================

extern ScalarFunction DistanceFunction_SSBox;
