/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"
#include "../Gui/opengl.h"

struct SurfelStatistics {
  float n, x, y, z, xx, xy, xz, yy, yz, zz, nx, ny, nz, r, g, b;
  float rad;
  SurfelStatistics():n(0), x(0), y(0), z(0), xx(0), xy(0), xz(0), yy(0), yz(0), zz(0), nx(0), ny(0), nz(0), r(0), g(0), b(0) {}
  void add(float X, float Y, float Z, float R, float G, float B) {
    n++;
    x+=X; y+=Y; z+=Z;
    xx+=X*X; yy+=Y*Y; zz+=Z*Z;
    xy+=X*Y; xz+=X*Z; yz+=Y*Z;
    r+=R; g+=G; b+=B;
  }
  void mean(float& mu_x, float& mu_y, float& mu_z) { mu_x=x/n; mu_y=y/n; mu_z=z/n; }
  void meanRGB(float& R, float& G, float& B) { R=r/n; G=g/n; B=b/n; }
  void norm(float& nx, float& ny, float& nz) {
    arr XX(3, 3), mu(3);
    mu(0)=x/n; mu(1)=y/n; mu(2)=z/n;
    XX(0, 0)=xx; XX(1, 1)=yy; XX(2, 2)=zz;
    XX(0, 1)=XX(1, 0)=xy; XX(0, 2)=XX(2, 0)=xz; XX(1, 2)=XX(2, 1)=yz;
    XX/=(double)n;
    XX-=mu*~mu;
    arr U, D, tV;
    svd(U, D, tV, XX); tV=~tV;
    nx = tV(1, 0);  ny = tV(1, 1);  nz = tV(1, 2); //last row of tV
  }

  void discount(float a) { n*=a; x*=a; y*=a; z*=a; xx*=a; xy*=a; xz*=a; yy*=a; yz*=a; zz*=a; r*=a; g*=a; b*=a; }

};

struct Surfels {
  OpenGL gl;
  rai::Mutex mx;
  arr pos, norm, col, rad;
  rai::Array<SurfelStatistics> D;
  uint32A surfelIdx, rndPerm;
  byteA mask, idxImage;

  Surfels():gl("Surfel Internel Renderer", 640, 480) {}
  uint N() { return pos.d0; }
  void setRandom(uint N);
  void glDraw(bool renderIndex);

  void recomputeSurfelIndices();
  void pointCloud2Surfels(const arr& pts, const arr& cols, OpenGL& gl);
};

void glDrawSurfels(void* classP);
void glDrawSurfelIndices(void* classP);
