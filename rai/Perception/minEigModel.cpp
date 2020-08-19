/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "minEigModel.h"
#include "../Gui/opengl.h"
#include "../Geo/geo.h"
#include "../Geo/qhull.h"
#include <GL/gl.h>

void MinEigModel::setPoints(const uintA& points) {
  pts = points;
  fringe = pts;
  included.resize(data.n());
  included.setZero();
  for(uint i:pts) included(i)=true;
  weights.resize(data.n());
  weights.setZero();
  setWeightsToOne();
}

void MinEigModel::setWeightsToOne() {
  setWeightsToZero();
  for(uint i:pts) weights(i)=1.;
  addStatistics(pts);
}

void MinEigModel::setWeightsToZero() {
  weights.setZero();
  stat_n = 0;
  stat_x = zeros(data.d());
  stat_xx = zeros(data.d(), data.d());
}

void MinEigModel::addStatistics(const uintA& points, bool minus) {
  arr wXi, wXiXi;
  for(uint i:points) {
    double w=weights(i);
    if(!w) continue;
    const arr& Xi=data.X[i];
    wXi=Xi;  wXi*=w;
    outerProduct(wXiXi, wXi, Xi);
    if(!minus) {
      stat_n += w;
      stat_x += wXi;
      stat_xx += wXiXi;
    } else {
      stat_n -= w;
      stat_x -= wXi;
      stat_xx -= wXiXi;
    }
  }
}

void MinEigModel::calc(bool update) {
#if 0
  resetStatistics();
  addStatistics(pts);
#endif
  mean = stat_x/stat_n;
  eig.A = stat_xx/stat_n - (mean^mean);
  if(bias_xx.N) eig.A += bias_xx;
  if(!update) {
    try {
      eig.computeExact();
    } catch(...) {}
  } else {
    eig.stepPowerMethod(3);
  }
}

void MinEigModel::expand(uint steps) {
  for(uint s=0; s<steps; s++) {
    data.expandFringe(fringe, pts, included);
    reweightWithError(fringe);
  }
}

void MinEigModel::reweightWithError(uintA& pts) {
  if(&pts==&this->pts) {
    setWeightsToZero();
  } else {
    addStatistics(pts, true); //subtract the current points
  }
  arr Xi;
  for(uint j=pts.N; j--;) {
    uint i=pts(j);
    Xi = data.X[i];
    Xi -= mean;
#if 1
    double coeff = -.1 * rai::sqr(scalarProduct(Xi, eig.x_lo)/margin);
    weights(i) = ::exp(coeff);
#else
    weights(i) = fabs(scalarProduct(Xi, eig.x_lo))<margin ? 1. : 0.;
#endif
    if(weights(i)<.1) {
      pts.remove(j);
      included(j)=false;
      weights(i)=0.;
    }
  }
  addStatistics(pts);
}

arr MinEigModel::getInliers() {
  arr X;
  for(uint i:pts) if(weights(i)>.5) X.append(data.X[i]);
  X.reshape(X.N/3, 3);
  return X;
}

void MinEigModel::computeConvexHull() {
  convexHull.V = getInliers();
  convexHull.makeConvexHull();
}

void MinEigModel::computeConvexHull2() {
  convexHull.V.clear();
  convexHull.T.clear();
  if(!eig.x_lo.N) return;
  arr b0, b1;
  if(eig.x_lo.argmax()==0) b0 = ARR(0, 1, 0) - eig.x_lo*eig.x_lo(1);
  else                       b0 = ARR(1, 0, 0) - eig.x_lo*eig.x_lo(0);
  b0 /= length(b0);
  b1 = crossProduct(eig.x_lo, b0);
  b1 /= length(b1);

  arr Xi, hull;
  for(uint i:pts) if(weights(i)>.5) {
      Xi = data.X[i];
      Xi -= mean;
      hull.append(scalarProduct(b0, Xi));
      hull.append(scalarProduct(b1, Xi));
    }
  if(hull.N<5) return;
  hull.reshape(hull.N/2, 2);
  try {
    hull = getHull(hull, convexHull.T);
  } catch(...) { return; }
  convexHull.V.resize(hull.d0, 3);
  for(uint i=0; i<hull.d0; i++) {
    convexHull.V[i] = mean + b0*hull(i, 0) + b1*hull(i, 1);
  }
}

double MinEigModel::coveredData(bool novelDataOnly) {
  double coveredData=0.;
  if(!novelDataOnly)
    for(uint i:pts) coveredData += weights(i)*data.costs(i);
  else
    for(uint i:pts) coveredData += weights(i)*data.costs(i)*(1.-data.isModelledWeights(i));
  return coveredData;
}

void MinEigModel::calcDensity() {
  computeConvexHull2();
//  density = stat_n * rai::sqr(mean(2)) / convexHull.getArea();
  if(!convexHull.V.N) { density=0.; return; }
  double circum = convexHull.getCircum();
  density = coveredData() / circum*circum;
}

void MinEigModel::glDraw(OpenGL& gl) {
  if(eig.x_lo.N!=3) return;
  if(!convexHull.V.N) {
    rai::Quaternion rot;
    rot.setDiff(Vector_z, rai::Vector(eig.x_lo));
    glPushMatrix();
    glTranslatef(mean(0), mean(1), mean(2));
    glRotate(rot);
    glDrawBox(.5, .5, .01);
    glBegin(GL_LINES);
    glVertex3d(0., 0., 0.);
    glVertex3d(0., 0., .1);
    glEnd();
    glPopMatrix();
  } else {
    convexHull.glDraw(gl);
  }
}

void MinEigModel::report(ostream& os, bool mini) {
  if(mini) {
    os <<"#p " <<pts.N <<" dy " <<density <<" lm " <<::sqrt(eig.lambda_lo) <<" lM " <<::sqrt(eig.lambda_hi) <<"  " <<label <<endl;
  } else {
    os <<"model-report" <<endl;
    os <<"  #pts=" <<pts.N <<" #fringe=" <<fringe.N <<endl;
    os <<"  STATS: n=" <<stat_n <<" <x>=" <<stat_x/stat_n /*<<" <xx>=" <<stat_xx/stat_n*/ <<endl;
    os <<"  EIG: lambda_lo=" <<eig.lambda_lo <<" x_lo=" <<eig.x_lo <<endl;
    os <<"  QUALITY: density=" <<density <<" coveredData=" <<coveredData() <<" area=" <<convexHull.getCircum() <<endl;
  }
}

void MinEigModel::colorPixelsWithWeights(arr& cols) {
  for(uint i:pts) {
    cols(data.idx2pixel(i), 0) = weights(i);
    cols(data.idx2pixel(i), 2) = data.isModelledWeights(i);
  }
}
