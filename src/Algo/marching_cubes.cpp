#include "marching_cubes.h"

#ifdef RAI_Lewiner
#  include "Lewiner/MarchingCubes.h"
#endif

namespace rai {

std::tuple<arr, uintA> marching_cubes(const floatA& grid_values, const arr& size) {
  CHECK_EQ(grid_values.nd, 3, "");

  MarchingCubes mc(grid_values.d0, grid_values.d1, grid_values.d2);
  mc.init_all() ;
  uint k=0, j=0, i=0;
  for(k=0; k<grid_values.d2; k++) {
    for(j=0; j<grid_values.d1; j++) {
      for(i=0; i<grid_values.d0; i++) {
        mc.set_data(grid_values(i, j, k), i, j, k) ;
      }
    }
  }

  mc.run();
  mc.clean_temps();

  //convert to Mesh
  arr V(mc.nverts(), 3);
  uintA T(mc.ntrigs(), 3);
  for(i=0; i<V.d0; i++) {
    V(i, 0)=(mc.vert(i)->x/(grid_values.d0-1) - .5)*size(0);
    V(i, 1)=(mc.vert(i)->y/(grid_values.d1-1) - .5)*size(1);
    V(i, 2)=(mc.vert(i)->z/(grid_values.d2-1) - .5)*size(2);
  }
  for(i=0; i<T.d0; i++) {
    T(i, 0)=mc.trig(i)->v1;
    T(i, 1)=mc.trig(i)->v2;
    T(i, 2)=mc.trig(i)->v3;
  }
  return std::tuple(V,T);
}

} //namespace
