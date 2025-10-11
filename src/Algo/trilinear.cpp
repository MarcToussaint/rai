#include "trilinear.h"

namespace rai {

double interpolate1D(double v0, double v1, double x) {
  return v0 * (1. - x) + v1 * x;
}

double interpolate2D(double v00, double v10, double v01, double v11, double x, double y) {
  double s = interpolate1D(v00, v10, x);
  double t = interpolate1D(v01, v11, x);
  return interpolate1D(s, t, y);
}

double interpolate3D(double v000, double v100, double v010, double v110, double v001, double v101, double v011, double v111, double x, double y, double z) {
  double s = interpolate2D(v000, v100, v010, v110, x, y);
  double t = interpolate2D(v001, v101, v011, v111, x, y);
  return interpolate1D(s, t, z);
}

template<class T>
rai::Array<T> trilinear_interpolate(const arr& pts, const rai::Array<T>& grid_values, const arr& grid_res) {

  arr frac(3), idx(3);
  rai::Array<T> pts_values(pts.d0);

  for (uint i = 0; i < pts.d0; i++) {
    arr idx_float = pts[i] / grid_res;
    rai::clip(idx_float(0), 0., grid_values.d0 - 1.);
    rai::clip(idx_float(1), 0., grid_values.d1 - 1.);
    rai::clip(idx_float(2), 0., grid_values.d2 - 1.);

    {
      for (uint i = 0; i < 3; i++) frac(i) = modf(idx_float(i), &idx(i));

      int _x = idx(0);
      int _y = idx(1);
      int _z = idx(2);
      double& dx = frac(0);
      double& dy = frac(1);
      double& dz = frac(2);

      if (_x + 1 == (int)grid_values.d0 && dx < 1e-10) { _x--;	dx = 1.; }
      if (_y + 1 == (int)grid_values.d1 && dy < 1e-10) { _y--;	dy = 1.; }
      if (_z + 1 == (int)grid_values.d2 && dz < 1e-10) { _z--;	dz = 1.; }

      double v000 = grid_values(_x + 0, _y + 0, _z + 0);
      double v100 = grid_values(_x + 1, _y + 0, _z + 0);
      double v010 = grid_values(_x + 0, _y + 1, _z + 0);
      double v110 = grid_values(_x + 1, _y + 1, _z + 0);
      double v001 = grid_values(_x + 0, _y + 0, _z + 1);
      double v101 = grid_values(_x + 1, _y + 0, _z + 1);
      double v011 = grid_values(_x + 0, _y + 1, _z + 1);
      double v111 = grid_values(_x + 1, _y + 1, _z + 1);

      pts_values.elem(i) = interpolate3D(v000, v100, v010, v110, v001, v101, v011, v111,
                                          dx, dy, dz);
    }
  }
  return pts_values;
}

template Array<double> trilinear_interpolate(const arr& pts, const rai::Array<double>& grid_values, const arr& grid_res);
template Array<float> trilinear_interpolate(const arr& pts, const rai::Array<float>& grid_values, const arr& grid_res);

} //namespace
