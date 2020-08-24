/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "modelEnsemble.h"

#define QUALITY_THRESHOLD 0.01
#define NOISE_MARGIN 0.1

ModelEnsemble::ModelEnsemble() {
  vert= {0.0477157, -0.617799, -0.784887};
}

ModelEnsemble::~ModelEnsemble() {
  listDelete(models);
}

bool ModelEnsemble::addNewRegionGrowingModel(DataNeighbored& data) {
  MinEigModel* model = new MinEigModel(data, NOISE_MARGIN);

//  if(models.N>5) return false;

  //-- find a random seed
  uint i;
  uintA idel;
  for(uint k=20; k--;) {
    if(!k) {
//      LOG(0) <<"no unused data points found";
      return false;
    }

    i=rnd(data.n());
    if(!data.valid(i)) continue;
    if(data.isModelledWeights(i)>.1) continue;

    idel = data.getKneighborhood(i, 20);
    double w=0.;
    for(uint j:idel) w += data.isModelledWeights(j);
    w/=idel.N;
    cout <<"SEED TESTED. w=" <<w <<endl;
    if(w>.1) continue;

    break; //success!
  }

  //-- initialize with neighborhood of size 400
  model->setPoints(data.getKneighborhood(i, 400));
  model->calc(false);

  //-- expand until fringe is empty
  for(uint k=0;; k++) {
    model->expand(10);
    if(!model->fringe.N) break;
    model->calc(true);
  }

  //-- check success
  model->calcDensity();
  cout <<"TESTED MODEL: ";
  model->report(cout, true);
  if(model->density<QUALITY_THRESHOLD) {
    cout <<"-- DISCARD" <<endl;
    delete model;
    return false;
  }
  cout <<"-- ACCEPT" <<endl;

  for(uint i:model->pts) if(data.isModelledWeights(i)<model->weights(i)) data.isModelledWeights(i)=model->weights(i);
  model->label=models.N;
  models.append(model);
  return true;
}

void ModelEnsemble::reoptimizeModels(DataNeighbored& data) {
  data.isModelledWeights.setZero();
  for(MinEigModel* model:models) {
//    model->expand(10);
//    model->setWeightsToOne();
//    model->calc(false);
    model->reweightWithError(model->pts);
    model->calc(false);
    model->fringe = model->pts;
    for(uint k=0;; k++) {
      model->expand(10);
      if(!model->fringe.N) break;
      model->calc(true);
    }
//    model->calc(true);
    model->calcDensity();
    if(model->density<QUALITY_THRESHOLD) { model->setWeightsToZero(); model->density=0.; continue; }
    for(uint i:model->pts) if(data.isModelledWeights(i)<model->weights(i)) data.isModelledWeights(i)=model->weights(i);
  }
  for(uint i=models.N; i--;) if(!models(i)->density) { delete models(i);  models.remove(i); }
}

void ModelEnsemble::reestimateVert() {
  for(auto m:models) {
    double sp = scalarProduct(m->eig.x_lo, vert);
    if(sp<0.) { m->eig.x_lo *= -1.; sp *=-1; }
    if(sp>.95) m->label=1;
  }

  double n=0.;
  arr mu=zeros(3);
  for(auto m:models) if(m->label==1) {
      n += ::sqrt(m->stat_n);
      mu += ::sqrt(m->stat_n) * m->eig.x_lo;
    }
  vert = mu/n;
  vert /= length(vert);
  for(auto m:models) if(m->label==1) m->bias_xx = -1e-1 * (vert^vert);
}

void ModelEnsemble::glDraw(OpenGL& gl) {
  for(MinEigModel* m:models) {
    glColor(m->label);
    m->glDraw(gl);
  }
}

void ModelEnsemble::report(ostream& os, bool mini) {
  uint c=0;
  for(MinEigModel* m:models) { os <<c++ <<' '; m->report(os, mini); }
  os <<"vert=" <<vert <<endl;
}
