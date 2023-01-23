#pragma once

#include "geo.h"

//===========================================================================
//
// analytic distance functions
//

struct SDF : ScalarFunction {
  SDF() : ScalarFunction( std::bind(&SDF::f, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3) ) {}
  ~SDF(){}
  virtual double f(arr& g, arr& H, const arr& x) = 0;

  arr eval(const arr& samples);
  floatA evalFloat(const arr& samples);
  void viewSlice(OpenGL& gl, double z, const arr& lo, const arr& hi);
  void animateSlices(const arr& lo, const arr& hi, double wait=0.);
  void view(double wait=0.);

  arr projectNewton(const arr& x0, double maxStep=.1, double regularization=1e-1);
};

struct SDF_Sphere : SDF {
  rai::Transformation pose; double r;
  SDF_Sphere(const rai::Transformation& _pose, double _r)
    : pose(_pose), r(_r) {}
  double f(arr& g, arr& H, const arr& x);
};

struct SDF_ssBox : SDF {
  rai::Transformation pose;
  arr size;
  double r;
  SDF_ssBox(const rai::Transformation& _pose, const arr& _size, double _r=0.)
      : pose(_pose), size(_size), r(_r) { if(size.N==4){ r=size(3); size.resizeCopy(3); } }
  double f(arr& g, arr& H, const arr& x);
};

struct SDF_SuperQuadric : SDF {
  rai::Transformation pose;
  arr size;
  double degree;
  SDF_SuperQuadric(const rai::Transformation& _pose, const arr& _size, double _degree=5.)
    : pose(_pose), size(_size), degree(_degree) {}
  double f(arr& g, arr& H, const arr& x);
};

struct SDF_ssSomething : SDF {
  std::shared_ptr<SDF> something;
  double r;
  SDF_ssSomething(const std::shared_ptr<SDF>& _something, double _r)
    : something(_something), r(_r) {}
  double f(arr& g, arr& H, const arr& x);
};

struct SDF_Cylinder : SDF {
  rai::Transformation pose; double size_z, r;
  SDF_Cylinder(const rai::Transformation& _pose, double _size_z, double _r)
    : pose(_pose), size_z(_size_z), r(_r) {}
  double f(arr& g, arr& H, const arr& x);
};

struct SDF_Capsule : SDF {
  rai::Transformation pose; double size_z, r;
  SDF_Capsule(const rai::Transformation& _pose, double _size_z, double _r)
    : pose(_pose), size_z(_size_z), r(_r) {}
  double f(arr& g, arr& H, const arr& x);
};

struct SDF_Blobby : SDF {
  double f(arr& g, arr& H, const arr& _x){
    double x=_x(0), y=_x(1), z=_x(2);
    return x*x*x*x - 5*x*x+ y*y*y*y - 5*y*y + z*z*z*z - 5*z*z + 11.8;
  }
};

struct SDF_Torus : SDF {
  double f(arr& g, arr& H, const arr& _x);
};

struct SDF_Transformed : SDF {
  rai::Transformation pose;
  std::shared_ptr<SDF> sdf;
  SDF_Transformed(const rai::Transformation& _pose, const std::shared_ptr<SDF>& _sdf)
    : pose(_pose), sdf(_sdf) {}
  double f(arr& g, arr& H, const arr& x);
};

struct SDF_GridData : SDF {
  rai::Transformation pose=0;
  floatA gridData;
  arr lo, up;

  SDF_GridData(const rai::Transformation& _pose, const floatA& _data, const arr& _lo, const arr& _up)
    : pose(_pose), gridData(_data), lo(_lo), up(_up) {}
  SDF_GridData(uint N, const arr& _lo, const arr& _up)
    : lo(_lo), up(_up) { gridData.resize(N,N,N).setZero(); }

  SDF_GridData(SDF& f, const arr& _lo, const arr& _up, const uintA& res);
  SDF_GridData() {}
  SDF_GridData(istream& is) { read(is); }

  double f(arr& g, arr& H, const arr& x);

  //manipulations
  void resample(uint d0, int d1=-1, int d2=-1);

  void smooth(uint width=3, uint iters=2);

  //helper
  void getNeighborsAndWeights(uintA& neigh, arr& weights, const arr& x_rel);
  arr getGridPosition(const uintA& idx){
    arr res = (up-lo) / arr{(double)gridData.d0-1, (double)gridData.d1-1, (double)gridData.d2-1};
    arr x(3);
    for(uint i=0;i<3;i++) x(i) = lo(i) + idx(i)*res(i);
    return x;
  }

  //IO
  void write(std::ostream& os) const;
  void read(std::istream& is);
};
stdPipes(SDF_GridData)

struct PCL2Field {
  SDF_GridData& field;
  floatA source;

  PCL2Field(SDF_GridData& _field):field(_field) {}

  double stepDiffusion(const arr& pts, const arr& values);

  double runDiffusion(const arr& pts, const arr& values, uint iters=30){
    double err;
    for(uint k=0;k<iters;k++){
      err = stepDiffusion(pts, values);
//      cout <<k <<" err: " <<err <<endl;
    }
    return err;
  }
};

//===========================================================================

extern ScalarFunction DistanceFunction_SSBox;
